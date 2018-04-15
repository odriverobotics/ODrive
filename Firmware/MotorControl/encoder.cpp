
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
    if (config_.use_index && !index_found_) {
        set_circular_count(0);
        if (config_.pre_calibrated) {
            offset_ = config_.offset;
            is_ready_ = true;
        }
        index_found_ = true;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    // Update states
    shadow_count_ = count;
    pos_estimate_ = (float)count;
    //Write hardware last
    hw_config_.timer->Instance->CNT = count;

    __set_PRIMASK(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    // Offset and state must be shifted by the same amount
    offset_ += count - count_in_cpr_;
    offset_ = mod(offset_, config_.cpr);
    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr = (float)count_in_cpr_;

    __set_PRIMASK(prim);
}


// @brief Slowly turns the motor in one direction until the
// encoder index is found.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_index_search() {
    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;
    
    float omega = (float)(axis_->motor_.config_.direction) * config_.idx_search_speed;

    index_found_ = false;
    float phase = 0.0f;
    axis_->run_control_loop([&](){
        phase = wrap_pm_pi(phase + omega * current_meas_period);

        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_IDX_SEARCH);

        // continue until the index is found
        return !index_found_;
    });
    return axis_->error_ != Axis::ERROR_NO_ERROR;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_offset_calibration() {
    static const float start_lock_duration = 1.0f;
    static const float scan_omega = 4.0f * M_PI;
    static const float scan_distance = 16.0f * M_PI;
    static const int num_steps = scan_distance / scan_omega * current_meas_hz;

    // Temporarily disable index search so it doesn't mess
    // with the offset calibration
    bool old_use_index = config_.use_index;
    config_.use_index = false;

    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;

    // go to motor zero phase for start_lock_duration to get ready to scan
    int i = 0;
    axis_->run_control_loop([&](){
        if (!axis_->motor_.enqueue_voltage_timings(voltage_magnitude, 0.0f))
            return false; // error set inside enqueue_voltage_timings
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
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
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
        error_ |= ERROR_CPR_OUT_OF_RANGE;
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
        error_ |= ERROR_RESPONSE;
        return false;
    }

    // scan backwards
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(-scan_distance * (float)i / (float)num_steps + scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += (int16_t)hw_config_.timer->Instance->CNT;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NO_ERROR)
        return false;

    offset_ = encvaluesum / (num_steps * 2);
    is_ready_ = true;
    config_.use_index = old_use_index;
    return true;
}

bool Encoder::update(float* pos_estimate, float* vel_estimate, float* phase_output) {
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        error_ |= ERROR_NUMERICAL;
        return false;
    }

    // update internal encoder state
    int16_t delta_enc_16 = (int16_t)hw_config_.timer->Instance->CNT - (int16_t)shadow_count_;
    int32_t delta_enc = (int32_t)delta_enc_16; //sign extend
    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    // compute electrical phase
    int corrected_enc = count_in_cpr_ - offset_;
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (float)corrected_enc;
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);


    // run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_ += current_meas_period * pll_vel_;
    pos_cpr       += current_meas_period * pll_vel_;
    // discrete phase detector
    float delta_pos     = (float)(shadow_count_ - (int32_t)floorf(pos_estimate_));
    float delta_pos_cpr = (float)(count_in_cpr_ - (int32_t)floorf(pos_cpr));
    delta_pos_cpr = wrap_pm(delta_pos_cpr, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_ += current_meas_period * pll_kp_ * delta_pos;
    pos_cpr       += current_meas_period * pll_kp_ * delta_pos_cpr;
    pos_cpr = fmodf_pos(pos_cpr, (float)(config_.cpr));
    pll_vel_      += current_meas_period * pll_ki_ * delta_pos_cpr;
    if (fabsf(pll_vel_) < 0.5f * current_meas_period * pll_ki_)
        pll_vel_ = 0.0f; //align delta-sigma on zero to prevent jitter

    // Assign output arguments
    if (pos_estimate) *pos_estimate = pos_estimate_;
    if (vel_estimate) *vel_estimate = pll_vel_;
    if (phase_output) *phase_output = phase_;
    return true;
}
