
#include "odrive_main.h"


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                Config_t& config) :
        hw_config_(hw_config),
        config_(config)
{
    // Calculate encoder pll gains
    // This calculation is currently identical to the PLL in SensorlessEstimator
    float pll_bandwidth = 1000.0f;  // [rad/s]
    pll_kp_ = 2.0f * pll_bandwidth;

    // Critically damped
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_);

    if (config.pre_calibrated && (config.mode == Encoder::MODE_HALL)) {
        offset_ = config.offset;
        is_ready_ = true;
    }
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(hw_config_.timer, TIM_CHANNEL_ALL);
    GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_NOPULL,
            enc_index_cb_wrapper, this);
}

void Encoder::set_error(Encoder::Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
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
    pos_cpr_ = (float)count_in_cpr_;

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
    return true;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_offset_calibration() {
    static const float start_lock_duration = 1.0f;
    static const float scan_omega = 4.0f * M_PI;
    static const float scan_distance = 16.0f * M_PI;
    static const int num_steps = (int)(scan_distance / scan_omega * (float)current_meas_hz);

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
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    int32_t init_enc_val = shadow_count_;
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

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = scan_distance / elec_rad_per_enc;
    float actual_encoder_delta_abs = fabsf(shadow_count_-init_enc_val);
    if(fabsf(actual_encoder_delta_abs - expected_encoder_delta)/expected_encoder_delta > config_.calib_range)
    {
        set_error(ERROR_CPR_OUT_OF_RANGE);
        return false;
    }
    // check direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        axis_->motor_.config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis_->motor_.config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_RESPONSE);
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

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    offset_ = encvaluesum / (num_steps * 2);
    config_.offset = offset_;
    int32_t residual = encvaluesum - ((int64_t)offset_ * (int64_t)(num_steps * 2));
    config_.offset_float = (float)residual / (float)(num_steps * 2) + 0.5f; // add 0.5 to center-align state to phase
    is_ready_ = true;
    config_.use_index = old_use_index;
    return true;
}

static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}

bool Encoder::update() {
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
        return false;
    }

    // update internal encoder state.
    int32_t delta_enc = 0;
    switch (config_.mode) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)hw_config_.timer->Instance->CNT - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;

        case MODE_HALL: {
            int32_t hall_cnt;
            if (decode_hall(hall_state_, &hall_cnt)) {
                delta_enc = hall_cnt - count_in_cpr_;
                delta_enc = mod(delta_enc, 6);
                if (delta_enc > 3)
                    delta_enc -= 6;
            } else {
                set_error(ERROR_ILLEGAL_HALL_STATE);
                return false;
            }
        } break;
        
        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_ += current_meas_period * pll_vel_;
    pos_cpr_      += current_meas_period * pll_vel_;
    // discrete phase detector
    float delta_pos     = (float)(shadow_count_ - (int32_t)floorf(pos_estimate_));
    float delta_pos_cpr = (float)(count_in_cpr_ - (int32_t)floorf(pos_cpr_));
    delta_pos_cpr = wrap_pm(delta_pos_cpr, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_ += current_meas_period * pll_kp_ * delta_pos;
    pos_cpr_      += current_meas_period * pll_kp_ * delta_pos_cpr;
    pos_cpr_ = fmodf_pos(pos_cpr_, (float)(config_.cpr));
    pll_vel_      += current_meas_period * pll_ki_ * delta_pos_cpr;
    bool snap_to_zero_vel = false;
    if (fabsf(pll_vel_) < 0.5f * current_meas_period * pll_ki_) {
        pll_vel_ = 0.0f; //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - offset_;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using pll_vel,
        interpolation_ += current_meas_period * pll_vel_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.offset_float);
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);

    return true;
}
