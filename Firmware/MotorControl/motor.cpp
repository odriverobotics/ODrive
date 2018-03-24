
#include <algorithm>

#include "drv8301.h"
//#include "motor.hpp"
#include "odrive_main.hpp"


Motor::Motor(const MotorHardwareConfig_t& hw_config,
         const GateDriverHardwareConfig_t& gate_driver_config,
         MotorConfig_t& config) :
        hw_config_(hw_config),
        gate_driver_config_(gate_driver_config),
        config_(config),
        gate_driver_({
            .spiHandle = gate_driver_config_.spi,
            .EngpioHandle = gate_driver_config_.enable_port,
            .EngpioNumber = gate_driver_config_.enable_pin,
            .nCSgpioHandle = gate_driver_config_.nCS_port,
            .nCSgpioNumber = gate_driver_config_.nCS_pin,
        })
{
}

// @brief Arms the PWM outputs that belong to this motor.
//
// Note that this does not yet activate the PWM outputs, it just unlocks them.
//
// While the motor is armed, the control loop must set new modulation timings
// between any two interrupts (that is, enqueue_modulation_timings must be executed).
// If the control loop fails to do so, the next interrupt handler floats the
// phases. Once this happens, missed_control_deadline is set to true and
// the motor can be considered disarmed.
//
// @returns: True on success, false otherwise
bool Motor::arm() {
    // Wait until the interrupt handler triggers twice. After the first wait there is an
    // undefined period until the next trigger. After the second wait we know for sure
    // that we have exactly one full interrupt period until the third trigger. This gives
    // the control loop the correct time quota to set up modulation timings.
    if (!(axis_->wait_for_current_meas() && axis_->wait_for_current_meas()))
        return false;
    next_timings_valid_ = false;
    axis_->missed_control_deadline_ = false;
    return true;
}

// @brief Floats the phases of this motor immediately and updates
// the brake current accordingly.
void Motor::disarm() {
    // disable pwm
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(hw_config_.timer);
    // set this motor's contribution to 0
    current_control_.Ibus = 0.0f;
    update_brake_current();
    // ensure the PWM is not re-enabled without the state machine explicitly
    // calling motor.arm()
    axis_->missed_control_deadline_ = true;
}

// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
void Motor::update_current_controller_gains() {
    // Calculate current control gains
    float current_control_bandwidth = 1000.0f;  // [rad/s]
    current_control_.p_gain = current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.i_gain = plant_pole * current_control_.p_gain;
}

// @brief Set up the gate drivers
void Motor::DRV8301_setup() {
    DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;

    DRV8301_enable(&gate_driver_);
    DRV8301_setupSpi(&gate_driver_, local_regs);

    // TODO we can use reporting only if we actually wire up the nOCTW pin
    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A
    local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;
    // local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_20VpV;

    switch (local_regs->Ctrl_Reg_2.GAIN) {
        case DRV8301_ShuntAmpGain_10VpV:
            phase_current_rev_gain_ = 1.0f / 10.0f;
            break;
        case DRV8301_ShuntAmpGain_20VpV:
            phase_current_rev_gain_ = 1.0f / 20.0f;
            break;
        case DRV8301_ShuntAmpGain_40VpV:
            phase_current_rev_gain_ = 1.0f / 40.0f;
            break;
        case DRV8301_ShuntAmpGain_80VpV:
            phase_current_rev_gain_ = 1.0f / 80.0f;
            break;
    }

    float margin = 0.90f;
    float max_input = margin * 0.3f * hw_config_.shunt_conductance;
    float max_swing = margin * 1.6f * hw_config_.shunt_conductance * phase_current_rev_gain_;
    current_control_.max_allowed_current = std::min(max_input, max_swing);

    local_regs->SndCmd = true;
    DRV8301_writeData(&gate_driver_, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(&gate_driver_, local_regs);
}

// @brief Checks if the gate driver is in operational state.
// @returns: true if the gate driver is OK (no fault), false otherwise
bool Motor::check_DRV_fault() {
    //TODO: make this pin configurable per motor ch
    GPIO_PinState nFAULT_state = HAL_GPIO_ReadPin(gate_driver_config_.nFAULT_port, gate_driver_config_.nFAULT_pin);
    if (nFAULT_state == GPIO_PIN_RESET) {
        // Update DRV Fault Code
        drv_fault_ = DRV8301_getFaultType(&gate_driver_);
        // Update/Cache all SPI device registers
        DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
        local_regs->RcvCmd = true;
        DRV8301_readData(&gate_driver_, local_regs);
        return false;
    };
    return true;
}

bool Motor::do_checks() {
    if (!check_DRV_fault()) {
        error_ = ERROR_DRV_FAULT;
        return false;
    }
    return true;
}

void Motor::log_timing(TimingLog_t log_idx) {
    TIM_HandleTypeDef* htim = hw_config_.timer;
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t delta = TIM_1_8_PERIOD_CLOCKS - timing;
        timing = TIM_1_8_PERIOD_CLOCKS + delta;
    }

    if (log_idx < TIMING_LOG_NUM_SLOTS) {
        timing_log_[log_idx] = timing;
    }
}

float Motor::phase_current_from_adcval(uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * phase_current_rev_gain_;
    float current = shunt_volt * hw_config_.shunt_conductance;
    return current;
}

//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    static const float kI = 10.0f;                                 // [(V/s)/A]
    static const int num_test_cycles = 3.0f / CURRENT_MEAS_PERIOD; // Test runs for 3s
    float test_voltage = 0.0f;
    
    size_t i = 0;
    axis_->run_control_loop([&](){
        float Ialpha = -(current_meas_.phB + current_meas_.phC);
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);
        if (test_voltage > max_voltage || test_voltage < -max_voltage)
            return error_ = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE, false;

        // Test voltage along phase A
        enqueue_voltage_timings(test_voltage, 0.0f);
        log_timing(TIMING_LOG_MEAS_R);

        return ++i < num_test_cycles;
    });
    if (axis_->error_ != Axis::ERROR_NO_ERROR)
        return false;

    //// De-energize motor
    //enqueue_voltage_timings(motor, 0.0f, 0.0f);

    float R = test_voltage / test_current;
    config_.phase_resistance = R;
    return true; // if we ran to completion that means success
}

bool Motor::measure_phase_inductance(float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    size_t t = 0;
    axis_->run_control_loop([&](){
        int i = t & 1;
        Ialphas[i] += -current_meas_.phB - current_meas_.phC;

        // Test voltage along phase A
        enqueue_voltage_timings(test_voltages[i], 0.0f);
        log_timing(TIMING_LOG_MEAS_L);

        return ++t < (num_cycles << 1);
    });
    if (axis_->error_ != Axis::ERROR_NO_ERROR)
        return false;

    //// De-energize motor
    //enqueue_voltage_timings(motor, 0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    config_.phase_inductance = L;
    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f)
        return error_ = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE, false;
    return true;
}


bool Motor::run_calibration() {
    error_ = ERROR_NO_ERROR;

    float R_calib_max_voltage = config_.resistance_calib_max_voltage;
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {
        if (!measure_phase_resistance(config_.calibration_current, R_calib_max_voltage))
            return false;
        if (!measure_phase_inductance(-R_calib_max_voltage, R_calib_max_voltage))
            return false;
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        // no calibration needed
    } else {
        return false;
    }

    update_current_controller_gains();
    
    is_calibrated_ = true;
    return true;
}

void Motor::enqueue_modulation_timings(float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    next_timings_[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_valid_ = true;
}

void Motor::enqueue_voltage_timings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    enqueue_modulation_timings(mod_alpha, mod_beta);
    log_timing(TIMING_LOG_FOC_VOLTAGE);
}

// TODO: This doesn't update brake current
// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool Motor::FOC_voltage(float v_d, float v_q, float phase) {
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float v_alpha = c*v_d - s*v_q;
    float v_beta  = c*v_q + s*v_d;
    enqueue_voltage_timings(v_alpha, v_beta);
    return true;
}

bool Motor::FOC_current(float Id_des, float Iq_des, float phase) {
    Current_control_t* ictrl = &current_control_;

    // For Reporting
    ictrl->Iq_setpoint = Iq_des;

    // Clarke transform
    float Ialpha = -current_meas_.phB - current_meas_.phC;
    float Ibeta = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);

    // Park transform
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float Id = c * Ialpha + s * Ibeta;
    float Iq = c * Ibeta - s * Ialpha;
    ictrl->Iq_measured = Iq;

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl->v_current_control_integral_d + Ierr_d * ictrl->p_gain;
    float Vq = ictrl->v_current_control_integral_q + Ierr_q * ictrl->p_gain;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl->v_current_control_integral_d *= 0.99f;
        ictrl->v_current_control_integral_q *= 0.99f;
    } else {
        ictrl->v_current_control_integral_d += Ierr_d * (ictrl->i_gain * current_meas_period);
        ictrl->v_current_control_integral_q += Ierr_q * (ictrl->i_gain * current_meas_period);
    }

    // Compute estimated bus current
    ictrl->Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float mod_alpha = c * mod_d - s * mod_q;
    float mod_beta = c * mod_q + s * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl->final_v_alpha = mod_to_V * mod_alpha;
    ictrl->final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    enqueue_modulation_timings(mod_alpha, mod_beta);
    log_timing(TIMING_LOG_FOC_CURRENT);

    return true;
}


bool Motor::update(float current_setpoint, float phase) {
    current_setpoint *= config_.direction;
    phase *= config_.direction;

    // Execute current command
    // TODO: move this into the mot
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {
        if(!FOC_current(0.0f, current_setpoint, phase)){
            return false;
        }
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        //In gimbal motor mode, current is reinterptreted as voltage.
        if(!FOC_voltage(0.0f, current_setpoint, phase))
            return false;
    } else {
        error_ = ERROR_NOT_IMPLEMENTED_MOTOR_TYPE;
        return false;
    }
    return true;
}
