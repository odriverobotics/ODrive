
//#include "encoder.hpp"
#include "odrive_main.hpp"


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                EncoderConfig_t& config) :
        hw_config_(hw_config),
        config_(config)
{
    // Calculate encoder pll gains
    // This calculation is currently identical to the PLL in SensorlessEstimator
    float pll_bandwidth = 1000.0f;  // [rad/s]
    pll_kp_ = 2.0f * pll_bandwidth;

    // Critically damped
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_);
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(hw_config_.timer, TIM_CHANNEL_ALL);
    GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_NOPULL,
            enc_index_cb_wrapper, this);
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// TODO: disable interrupt once we found the index
void Encoder::enc_index_cb() {
    if (!index_found_) {
        set_count(0);
        index_found_ = true;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    state_ = count;
    hw_config_.timer->Instance->CNT = count;
    pll_pos_ = (float)count;
    __set_PRIMASK(prim);
}


// TODO: Do the scan with current, not voltage!
// TODO: add check_timing
bool Encoder::calib_enc_offset(float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const float scan_omega = 4.0f * M_PI;
    static const float scan_distance = 16.0f * M_PI;
    static const int num_steps = scan_distance / scan_omega * current_meas_hz;

    // go to motor zero phase for start_lock_duration to get ready to scan
    int i = 0;
    axis_->run_control_loop([&](){
        axis_->motor_.enqueue_voltage_timings(voltage_magnitude, 0.0f);
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);
        return ++i < start_lock_duration * current_meas_hz;
    });
    if (axis_->error_ != Axis::ERROR_NO_ERROR)
        return false;

    int32_t init_enc_val = (int16_t)hw_config_.timer->Instance->CNT;
    int64_t encvaluesum = 0;

    // scan forward
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(scan_distance * (float)i / (float)num_steps - scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta);
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += (int16_t)hw_config_.timer->Instance->CNT;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NO_ERROR)
        return false;

    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = scan_distance / elec_rad_per_enc;
    float actual_encoder_delta_abs = fabsf((int16_t)hw_config_.timer->Instance->CNT-init_enc_val);
    if(fabsf(actual_encoder_delta_abs - expected_encoder_delta)/expected_encoder_delta > config_.calib_range)
    {
        error_ = ERROR_CPR_OUT_OF_RANGE;
        return false;
    }
    // check direction
    if ((int16_t)hw_config_.timer->Instance->CNT > init_enc_val + 8) {
        // motor same dir as encoder
        axis_->motor_.config_.direction = 1;
    } else if ((int16_t)hw_config_.timer->Instance->CNT < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis_->motor_.config_.direction = -1;
    } else {
        // Encoder response error
        error_ = ERROR_RESPONSE;
        return false;
    }

    // scan backwards
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(-scan_distance * (float)i / (float)num_steps + scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta);
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += (int16_t)hw_config_.timer->Instance->CNT;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NO_ERROR)
        return false;

    int offset = encvaluesum / (num_steps * 2);
    config_.offset = offset;
    is_calibrated_ = true;
    return true;
}

bool Encoder::scan_for_enc_idx(float omega, float voltage_magnitude) {
    index_found_ = false;
    float phase = 0.0f;
    axis_->run_control_loop([&](){
        phase = wrap_pm_pi(phase + omega * current_meas_period);

        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta);
        axis_->motor_.log_timing(Motor::TIMING_LOG_IDX_SEARCH);

        // continue until the index is found
        return !index_found_;
    });
    return axis_->error_ == Axis::ERROR_NO_ERROR;
}

bool Encoder::run_calibration() {
    float enc_calibration_voltage;
    if (axis_->motor_.config_.motor_type == MOTOR_TYPE_HIGH_CURRENT)
        enc_calibration_voltage = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == MOTOR_TYPE_GIMBAL)
        enc_calibration_voltage = axis_->motor_.config_.calibration_current;
    else
        return false;

    if (config_.use_index && !index_found_)
        if (!scan_for_enc_idx(
                (float)(axis_->motor_.config_.direction) * config_.idx_search_speed,
                enc_calibration_voltage))
            return false;
    if (!config_.hand_calibrated) // TODO: discuss what logic we want here
        if (!calib_enc_offset(enc_calibration_voltage))
            return false;
    return true;
}

bool Encoder::update(float* pos_estimate, float* vel_estimate, float* phase_output) {
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        error_ = ERROR_NUMERICAL;
        return false;
    }

    // update internal encoder state
    int16_t delta_enc = (int16_t)hw_config_.timer->Instance->CNT - (int16_t)state_;
    state_ += (int32_t)delta_enc;

    // compute electrical phase
    int corrected_enc = state_ % config_.cpr;
    corrected_enc -= config_.offset;
    //corrected_enc *= axis_->motor_.config_.direction; TODO: verify if this still works
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (float)corrected_enc;
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);

    // run pll (for now pll is in units of encoder counts)
    // TODO pll_pos runs out of precision very quickly here! Perhaps decompose into integer and fractional part?
    // Predict current pos
    pll_pos_ += current_meas_period * pll_vel_;
    // discrete phase detector
    float delta_pos = (float)(state_ - (int32_t)floorf(pll_pos_));
    // pll feedback
    pll_pos_ += current_meas_period * pll_kp_ * delta_pos;
    pll_vel_ += current_meas_period * pll_ki_ * delta_pos;

    // Assign output arguments
    if (pos_estimate) *pos_estimate = pll_pos_;
    if (vel_estimate) *vel_estimate = pll_vel_;
    if (phase_output) *phase_output = phase_;
    return true;
}
