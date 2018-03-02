
//#include "encoder.hpp"
#include "axis.hpp"


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                EncoderConfig_t& config) :
        hw_config(hw_config),
        config(config)
{
    // Calculate encoder pll gains
    // This calculation is currently identical to the PLL in SensorlessEstimator
    float pll_bandwidth = 1000.0f;  // [rad/s]
    pll_kp = 2.0f * pll_bandwidth;

    // Critically damped
    pll_ki = 0.25f * (pll_kp * pll_kp);
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(hw_config.timer, TIM_CHANNEL_ALL);
    GPIO_subscribe(hw_config.index_port, hw_config.index_pin, GPIO_NOPULL,
            enc_index_cb_wrapper, this);
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// TODO: disarm interrupt once we found the index
void Encoder::enc_index_cb() {
    if (!index_found) {
        set_count(0);
        index_found = true;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_count(uint32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    state = count;
    hw_config.timer->Instance->CNT = count;
    pll_pos = (float)count;
    __set_PRIMASK(prim);
}


// TODO: Do the scan with current, not voltage!
// TODO: add check_timing
bool Encoder::calib_enc_offset(float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const float scan_duration = 1.0f;
    static const float scan_range = 16.0f * M_PI;
    static const size_t num_steps = scan_duration * current_meas_hz;

    // go to motor zero phase for start_lock_duration to get ready to scan
    size_t i = 0;
    axis->run_control_loop([&](){
        axis->motor.enqueue_voltage_timings(voltage_magnitude, 0.0f);
        return ++i < start_lock_duration * current_meas_hz;
    });

    int32_t init_enc_val = (int16_t)hw_config.timer->Instance->CNT;
    int64_t encvaluesum = 0;

    // scan forward
    i = 0;
    axis->run_control_loop([&](){
        float phase = wrap_pm_pi(scan_range * (float)i / (float)num_steps - scan_range / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        axis->motor.enqueue_voltage_timings(v_alpha, v_beta);

        encvaluesum += (int64_t)hw_config.timer->Instance->CNT;
        
        return ++i < num_steps;
    });
    if (i < num_steps)
        return false;

    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis->motor.config.pole_pairs * 2 * M_PI * (1.0f / (float)(config.cpr));
    float expected_encoder_delta = scan_range / elec_rad_per_enc;
    float actual_encoder_delta_abs = fabsf((int16_t)hw_config.timer->Instance->CNT-init_enc_val);
    if(fabsf(actual_encoder_delta_abs - expected_encoder_delta)/expected_encoder_delta > config.calib_range)
    {
        axis->motor.error = ERROR_ENCODER_CPR_OUT_OF_RANGE;
        return false;
    }
    // check direction
    if ((int16_t)hw_config.timer->Instance->CNT > init_enc_val + 8) {
        // motor same dir as encoder
        axis->motor.config.direction = 1;
    } else if ((int16_t)hw_config.timer->Instance->CNT < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis->motor.config.direction = -1;
    } else {
        // Encoder response error
        axis->motor.error = ERROR_ENCODER_RESPONSE;
        return false;
    }

    // scan backwards
    i = 0;
    axis->run_control_loop([&](){
        float phase = wrap_pm_pi(-scan_range * (float)i / (float)num_steps + scan_range / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        axis->motor.enqueue_voltage_timings(v_alpha, v_beta);

        encvaluesum += (int64_t)hw_config.timer->Instance->CNT;
        
        return ++i < num_steps;
    });
    if (i < num_steps)
        return false;

    int offset = encvaluesum / (num_steps * 2);
    config.offset = offset;
    config.calibrated = true;
    return true;
}

bool Encoder::scan_for_enc_idx(float omega, float voltage_magnitude) {
    index_found = false;
    float phase = 0.0f;
    axis->run_control_loop([&](){
        phase = wrap_pm_pi(phase + omega * current_meas_period);

        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        axis->motor.enqueue_voltage_timings(v_alpha, v_beta);
        
        // continue until the index is found
        return !index_found;
    });
    return index_found;
}

bool Encoder::run_calibration() {
    float enc_calibration_voltage;
    if (axis->motor.config.motor_type == MOTOR_TYPE_HIGH_CURRENT)
        enc_calibration_voltage = axis->motor.config.calibration_current * axis->motor.config.phase_resistance;
    else if (axis->motor.config.motor_type == MOTOR_TYPE_GIMBAL)
        enc_calibration_voltage = axis->motor.config.calibration_current;
    else
        return false;

    if (config.use_index && !index_found)
        if (!scan_for_enc_idx(
                /*(float)(axis->motor.config.direction) * */ config.idx_search_speed,
                enc_calibration_voltage))
            return false;
    if (!config.calibrated)
        if (!calib_enc_offset(enc_calibration_voltage))
            return false;
    return true;
}

bool Encoder::update(float* pos_estimate, float* vel_estimate, float* phase_output) {
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp < 1.0f)) {
        axis->motor.error = ERROR_CALIBRATION_TIMING;
        return false;
    }

    // update internal encoder state
    int16_t delta_enc = (int16_t)hw_config.timer->Instance->CNT - (int16_t)state;
    state += (int32_t)delta_enc;

    // compute electrical phase
    int corrected_enc = state % config.cpr;
    corrected_enc -= config.offset;
    //corrected_enc *= axis->motor.config.direction; TODO: verify if this still works
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis->motor.config.pole_pairs * 2 * M_PI * (1.0f / (float)(config.cpr));
    float ph = elec_rad_per_enc * (float)corrected_enc;
    // ph = fmodf(ph, 2*M_PI);
    phase = wrap_pm_pi(ph);

    // run pll (for now pll is in units of encoder counts)
    // TODO pll_pos runs out of precision very quickly here! Perhaps decompose into integer and fractional part?
    // Predict current pos
    pll_pos += current_meas_period * pll_vel;
    // discrete phase detector
    float delta_pos = (float)(state - (int32_t)floorf(pll_pos));
    // pll feedback
    pll_pos += current_meas_period * pll_kp * delta_pos;
    pll_vel += current_meas_period * pll_ki * delta_pos;

    // Assign output arguments
    if (*pos_estimate) *pos_estimate = pll_pos;
    if (*vel_estimate) *vel_estimate = pll_vel;
    if (*phase_output) *phase_output = phase;
    return true;
}
