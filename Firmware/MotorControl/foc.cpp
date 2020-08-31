
#include "foc.hpp"
#include <board.h>

Motor::Error AlphaBetaFrameController::on_measurement(
            float vbus_voltage, std::array<float, 3> currents,
            uint32_t input_timestamp) {
    // Clarke transform
    float Ialpha = currents[0];
    float Ibeta = one_by_sqrt3 * (currents[1] - currents[2]);
    return on_measurement(vbus_voltage, Ialpha, Ibeta, input_timestamp);
}

Motor::Error AlphaBetaFrameController::get_output(
            uint32_t output_timestamp, float (&pwm_timings)[3], float* ibus) {
    float mod_alpha = NAN;
    float mod_beta = NAN;

    Motor::Error status = get_alpha_beta_output(output_timestamp, &mod_alpha, &mod_beta, ibus);
    
    if (status != Motor::ERROR_NONE) {
        return status;
    } else if (std::isnan(mod_alpha) || std::isnan(mod_alpha)) {
        return Motor::ERROR_MODULATION_IS_NAN;
    } else if (SVM(mod_alpha, mod_beta, &pwm_timings[0], &pwm_timings[1], &pwm_timings[2]) != 0) {
        return Motor::ERROR_MODULATION_MAGNITUDE;
    }

    return Motor::ERROR_NONE;
}

void FieldOrientedController::reset() {
    v_current_control_integral_d_ = 0.0f;
    v_current_control_integral_q_ = 0.0f;
    vbus_voltage_measured_ = NAN;
    Ialpha_measured_ = NAN;
    Ibeta_measured_ = NAN;
}

Motor::Error FieldOrientedController::on_measurement(
            float vbus_voltage, float Ialpha, float Ibeta,
            uint32_t input_timestamp) {
    // Store the measurements for later processing.
    i_timestamp_ = input_timestamp;
    vbus_voltage_measured_ = vbus_voltage;
    Ialpha_measured_ = Ialpha;
    Ibeta_measured_ = Ibeta;

    return Motor::ERROR_NONE;
}

ODriveIntf::MotorIntf::Error FieldOrientedController::get_alpha_beta_output(
        uint32_t output_timestamp, float* mod_alpha, float* mod_beta, float* ibus) {

    if (std::isnan(vbus_voltage_measured_) || std::isnan(Ialpha_measured_) || std::isnan(Ibeta_measured_)) {
        // FOC didn't receive a current measurement yet.
        return Motor::ERROR_CONTROLLER_INITIALIZING;
    } else if (abs((int32_t)(i_timestamp_ - ctrl_timestamp_)) > MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA) {
        // Data from control loop and current measurement are too far apart.
        return Motor::ERROR_BAD_TIMING;
    }

    // TODO: improve efficiency in case PWM updates are requested at a higher
    // rate than current sensor updates. In this case we can reuse mod_d and
    // mod_q from a previous iteration.

    // Fetch member variables into local variables to make the optimizer's life easier.
    float vbus_voltage = vbus_voltage_measured_;
    float Ialpha = Ialpha_measured_;
    float Ibeta = Ibeta_measured_;
    float Vd = Vd_setpoint_;
    float Vq = Vq_setpoint_;
    float Id_setpoint = Id_setpoint_;
    float Iq_setpoint = Iq_setpoint_;
    float phase = phase_;
    float phase_vel = phase_vel_;

    if (std::isnan(phase) || std::isnan(phase_vel)) {
        return Motor::ERROR_UNKNOWN_PHASE;
    }

    // Park transform
    float I_phase = phase + phase_vel * ((float)(int32_t)(i_timestamp_ - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
    float c_I = our_arm_cos_f32(I_phase);
    float s_I = our_arm_sin_f32(I_phase);
    float Id = c_I * Ialpha + s_I * Ibeta;
    float Iq = c_I * Ibeta - s_I * Ialpha;
    Iq_measured_ += I_measured_report_filter_k_ * (Iq - Iq_measured_);
    Id_measured_ += I_measured_report_filter_k_ * (Id - Id_measured_);

    // Current error
    float Ierr_d = Id_setpoint - Id;
    float Ierr_q = Iq_setpoint - Iq;


    if (enable_current_control_) {
        // Check for current sense saturation
        if (std::isnan(Ierr_d) || std::isnan(Ierr_q)) {
            return Motor::ERROR_UNKNOWN_CURRENT;
        }

        // Apply PI control (V{d,q}_setpoint act as feed-forward terms in this mode)
        Vd += v_current_control_integral_d_ + Ierr_d * p_gain_;
        Vq += v_current_control_integral_q_ + Ierr_q * p_gain_;
    }

    if (std::isnan(vbus_voltage)) {
        return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
    }

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    if (enable_current_control_) {
        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
        if (mod_scalefactor < 1.0f) {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
            // TODO make decayfactor configurable
            v_current_control_integral_d_ *= 0.99f;
            v_current_control_integral_q_ *= 0.99f;
        } else {
            v_current_control_integral_d_ += Ierr_d * (i_gain_ * current_meas_period);
            v_current_control_integral_q_ += Ierr_q * (i_gain_ * current_meas_period);
        }
    }

    // Inverse park transform
    float pwm_phase = phase_ + phase_vel_ * ((float)(int32_t)(output_timestamp - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha_temp = c_p * mod_d - s_p * mod_q;
    float mod_beta_temp = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorless estimator)
    final_v_alpha_ = mod_to_V * mod_alpha_temp;
    final_v_beta_ = mod_to_V * mod_beta_temp;

    *mod_alpha = mod_alpha_temp;
    *mod_beta = mod_beta_temp;
    *ibus = mod_d * Id + mod_q * Iq;
    return Motor::ERROR_NONE;
}

void FieldOrientedController::update(uint32_t timestamp) {
    CRITICAL_SECTION() {
        ctrl_timestamp_ = timestamp;
        enable_current_control_ = enable_current_control_src_;
        Id_setpoint_ = Id_setpoint_src_ ? *Id_setpoint_src_ : NAN;
        Iq_setpoint_ = Iq_setpoint_src_ ? *Iq_setpoint_src_ : NAN;
        Vd_setpoint_ = Vd_setpoint_src_ ? *Vd_setpoint_src_ : NAN;
        Vq_setpoint_ = Vq_setpoint_src_ ? *Vq_setpoint_src_ : NAN;
        phase_ = phase_src_ ? *phase_src_ : NAN;
        phase_vel_ = phase_vel_src_ ? *phase_vel_src_ : NAN;
    }
}
