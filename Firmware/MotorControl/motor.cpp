
#include <algorithm>

#include "drv8301.h"
#include "odrive_main.h"


Motor::Motor(STM32_Timer_t* timer,
             STM32_GPIO_t* pwm_al_gpio, STM32_GPIO_t* pwm_bl_gpio, STM32_GPIO_t* pwm_cl_gpio,
             STM32_GPIO_t* pwm_ah_gpio, STM32_GPIO_t* pwm_bh_gpio, STM32_GPIO_t* pwm_ch_gpio,
             GateDriver_t* gate_driver_a,
             GateDriver_t* gate_driver_b,
             GateDriver_t* gate_driver_c,
             CurrentSensor_t* current_sensor_a,
             CurrentSensor_t* current_sensor_b,
             CurrentSensor_t* current_sensor_c,
             Thermistor_t* inverter_thermistor,
             uint16_t period, uint16_t repetition_counter, uint16_t dead_time,
             Config_t& config) :
        timer_(timer),
        pwm_al_gpio_(pwm_al_gpio), pwm_bl_gpio_(pwm_bl_gpio), pwm_cl_gpio_(pwm_cl_gpio),
        pwm_ah_gpio_(pwm_ah_gpio), pwm_bh_gpio_(pwm_bh_gpio), pwm_ch_gpio_(pwm_ch_gpio),
        gate_driver_a_(gate_driver_a),
        gate_driver_b_(gate_driver_b),
        gate_driver_c_(gate_driver_c),
        current_sensor_a_(current_sensor_a),
        current_sensor_b_(current_sensor_b),
        current_sensor_c_(current_sensor_c),
        inverter_thermistor_(inverter_thermistor),
        period_(period), repetition_counter_(repetition_counter), dead_time_(dead_time),
        config_(config) {
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

    // Reset controller states, integrators, setpoints, etc.
    axis_->controller_.reset();
    reset_current_control();

    // Wait until the interrupt handler triggers twice. This gives
    // the control loop the correct time quota to set up modulation timings.
    if (!axis_->wait_for_current_meas())
        return axis_->error_ |= Axis::ERROR_CURRENT_MEASUREMENT_TIMEOUT, false;
    safety_critical_arm_motor_pwm(*this);
    return true;
}

void Motor::reset_current_control() {
    current_control_.v_current_control_integral_d = 0.0f;
    current_control_.v_current_control_integral_q = 0.0f;
}

// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
void Motor::update_current_controller_gains() {
    // Calculate current control gains
    current_control_.p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.i_gain = plant_pole * current_control_.p_gain;
}

void Motor::handle_timer_update() {
    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    bool counting_down = timer_->htim.Instance->CR1 & TIM_CR1_DIR;

    // TODO: implement double rate updates, then we need to run this check also
    // while counting down
    if (!counting_down) {
        if (is_updating_pwm_timings_ || last_pwm_update_timestamp_ > pwm_control_deadline_) {
            // PWM values were not updated in time - shut down PWM
            safety_critical_disarm_motor_pwm(*this);
        } else {
            // Reset PWM values to 50%. If they are overwritten before the next
            // timer update occurs, this has no effect. Otherwise, 50% duty cycle
            // comes into effect until this interrupt handler is executed again and
            // disarms the output alltogether.
            uint16_t half_timings[] = {
                (uint16_t)(period_ / 2),
                (uint16_t)(period_ / 2),
                (uint16_t)(period_ / 2)
            };
            safety_critical_apply_motor_pwm_timings(*this, period_, half_timings);
        }
        last_pwm_update_timestamp_ = 0xffffffff;
    }

    if (!counting_down) {
        axis_->encoder_.sample_now();

        for (int i = 0; i < n_GPIO_samples; ++i) {
            GPIO_port_samples[i] = GPIOs_to_samp[i]->IDR;
        }
    }
}

// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
// TODO: Document how the phasing is done, link to timing diagram
void Motor::handle_current_sensor_update() {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    static const float calib_filter_k = current_meas_period / calib_tau;

    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    bool counting_down = timer_->htim.Instance->CR1 & TIM_CR1_DIR;
    bool current_meas_not_DC_CAL = !counting_down;

    new_current_readings_ |= current_sensor_a_->has_value() ? 0x1 : 0x0;
    new_current_readings_ |= current_sensor_b_->has_value() ? 0x2 : 0x0;
    new_current_readings_ |= current_sensor_c_->has_value() ? 0x4 : 0x0;

    if (new_current_readings_ == 0x7) {
        // all current sensors have a reading
        new_current_readings_ = 0;

        Iph_ABC_t current = { 0 };
        if (!(current_sensor_a_->consume_value(&current.phA)
            && current_sensor_b_->consume_value(&current.phB)
            && current_sensor_c_->consume_value(&current.phC))) {
            set_error(ERROR_CURRENT_SENSOR);
            return;
        }

        // TODO: make DC cal optional
        if (current_meas_not_DC_CAL) {
            current_meas_.phA = current.phA - DC_calib_.phA;
            current_meas_.phB = current.phB - DC_calib_.phB;
            current_meas_.phC = current.phC - DC_calib_.phC;

            // Prepare hall readings
            // TODO move this to inside encoder update function
            axis_->encoder_.decode_hall_samples(GPIO_port_samples);
            // Trigger axis thread
            axis_->signal_current_meas();
        } else {
            // DC_CAL measurement
            DC_calib_.phA += (current.phA - DC_calib_.phA) * calib_filter_k;
            DC_calib_.phB += (current.phB - DC_calib_.phB) * calib_filter_k;
            DC_calib_.phC += (current.phC - DC_calib_.phC) * calib_filter_k;
        }
    }
}

// @brief Set up the gate drivers
bool Motor::init() {
    update_current_controller_gains();

    uint16_t half_timings[] = {
        (uint16_t)(period_ / 2),
        (uint16_t)(period_ / 2),
        (uint16_t)(period_ / 2)
    };
    safety_critical_apply_motor_pwm_timings(*this, period_, half_timings);

    // Init PWM
    if (!timer_->init(
            period_ /* period */,
            STM32_Timer_t::UP_DOWN /* mode */,
            0 /* prescaler */,
            repetition_counter_ /* repetition counter */
        )) {
        return false;
    }
    // Ensure that debug halting of the core doesn't leave the motor PWM running
    if (!timer_->set_freeze_on_dbg(true)
        || !timer_->setup_pwm(1, pwm_ah_gpio_, pwm_al_gpio_, true, true, period_ / 2)
        || !timer_->setup_pwm(2, pwm_bh_gpio_, pwm_bl_gpio_, true, true, period_ / 2)
        || !timer_->setup_pwm(3, pwm_ch_gpio_, pwm_cl_gpio_, true, true, period_ / 2)
        /*|| !timer_->setup_pwm(4, nullptr, nullptr, true, true, period_ / 2)*/ // required to trigger ADC
        || !timer_->set_dead_time(dead_time_)
        || !timer_->on_update_.set<Motor, &Motor::handle_timer_update>(*this)) {
        return false;
    }
    vbus_voltage = 124;
    //timer_.enable_interrupt(TRIGGER_AND_COMMUTATION, 0, 0); // TODO: M1 used to enable this one too, why?

    static const float kMargin = 0.90f;
    static const float kTripMargin = 1.0f; // Trip level is at edge of linear range of amplifer

    if (!gate_driver_a_ || !gate_driver_b_ || !gate_driver_c_)
        return false;

    // Init ADC
    if (!gate_driver_a_->init())
        return false;
    if (!gate_driver_b_->init())
        return false;
    if (!gate_driver_c_->init())
        return false;

    if (!current_sensor_a_ || !current_sensor_b_ || !current_sensor_c_)
        return false;

    if (!current_sensor_a_->init(config_.requested_current_range / kMargin))
        return false;
    if (!current_sensor_b_->init(config_.requested_current_range / kMargin))
        return false;
    if (!current_sensor_c_->init(config_.requested_current_range / kMargin))
        return false;

    system_stats_.boot_progress++;

    float range_a, range_b, range_c;
    if (!current_sensor_a_->get_range(&range_a))
        return false;
    if (!current_sensor_b_->get_range(&range_b))
        return false;
    if (!current_sensor_c_->get_range(&range_c))
        return false;

    if (range_a != range_b || range_b != range_c)
        return false;

    system_stats_.boot_progress++;

    // Clip all current control to actual usable range
    current_control_.max_allowed_current = range_a;
    // Set trip level
    current_control_.overcurrent_trip_level = (kTripMargin / kMargin) * current_control_.max_allowed_current;

    if (!current_sensor_a_->on_update_.set<Motor, &Motor::handle_current_sensor_update>(*this))
        return false;
    if (!current_sensor_b_->on_update_.set<Motor, &Motor::handle_current_sensor_update>(*this))
        return false;
    if (!current_sensor_c_->on_update_.set<Motor, &Motor::handle_current_sensor_update>(*this))
        return false;

    return true;
}

bool Motor::start_updates() {
    if (!current_sensor_a_ || !current_sensor_b_ || !current_sensor_c_)
        return false;

    // The inferred current sensor doesn't trigger update events so we ignore
    // the return values here. If none of them triggers, the PWM timings will not
    // be updated and the motor will go to error state.
    (void) current_sensor_a_->enable_updates();
    (void) current_sensor_b_->enable_updates();
    (void) current_sensor_c_->enable_updates();


    pwm_control_deadline_ = period_ - 1;
    uint16_t half_load = period_ / 2;
    timer_->htim.Instance->CCR1 = half_load;
    timer_->htim.Instance->CCR2 = half_load;
    timer_->htim.Instance->CCR3 = half_load;

    // Enable the update interrupt (used to coherently sample GPIO)
    timer_->enable_update_interrupt();

    // TODO: this can probably be removed
    safety_critical_disarm_motor_pwm(*this);

    // TODO: CH4 used to be started too - why?
    timer_->enable_pwm(true, true, true, true, true, true, false, false);

    timer_->start();

    return true;
}

// @brief Checks if the gate driver is in operational state.
// @returns: true if the gate driver is OK (no fault), false otherwise
bool Motor::check_DRV_fault() {
    if (!gate_driver_a_->check_fault())
        return false;
    if (!gate_driver_b_->check_fault())
        return false;
    if (!gate_driver_c_->check_fault())
        return false;
    return true;
}

void Motor::set_error(Motor::Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
    safety_critical_disarm_motor_pwm(*this);
    update_brake_current();
}

float Motor::get_inverter_temp() {
    // TODO: support more than one temp sensor
    float temp;
    if (inverter_thermistor_ && inverter_thermistor_->read_temp(&temp)) {
        return temp;
    } else {
        return 0.0f;
    }
}

bool Motor::update_thermal_limits() {
    float fet_temp = get_inverter_temp();
    float temp_margin = config_.inverter_temp_limit_upper - fet_temp;
    float derating_range = config_.inverter_temp_limit_upper - config_.inverter_temp_limit_lower;
    thermal_current_lim_ = config_.current_lim * (temp_margin / derating_range);
    if (!(thermal_current_lim_ >= 0.0f)) { //Funny polarity to also catch NaN
        thermal_current_lim_ = 0.0f;
    }
    if (fet_temp > config_.inverter_temp_limit_upper + 5) {
        set_error(ERROR_INVERTER_OVER_TEMP);
        return false;
    }
    return true;
}

bool Motor::do_checks() {
    if (!check_DRV_fault()) {
        set_error(ERROR_DRV_FAULT);
        return false;
    }
    if (!update_thermal_limits()) {
        //error already set in function
        return false;
    }
    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage);
    } else {
        current_lim = std::min(current_lim, axis_->motor_.current_control_.max_allowed_current);
    }
    // Thermal limit
    current_lim = std::min(current_lim, thermal_current_lim_);

    return current_lim;
}

//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    static const float kI = 10.0f;                                 // [(V/s)/A]
    static const size_t num_test_cycles = static_cast<size_t>(3.0f / current_meas_period); // Test runs for 3s
    float test_voltage = 0.0f;
    
    size_t i = 0;
    axis_->run_control_loop([&](){
        float Ialpha = current_meas_.phA;
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);
        if (test_voltage > max_voltage || test_voltage < -max_voltage)
            return set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE), false;

        // Test voltage along phase A
        if (!enqueue_voltage_timings(test_voltage, 0.0f))
            return false; // error set inside enqueue_voltage_timings
        //log_timing(TIMING_LOG_MEAS_R);

        return ++i < num_test_cycles;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

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
        if (!enqueue_voltage_timings(test_voltages[i], 0.0f))
            return false; // error set inside enqueue_voltage_timings
        //log_timing(TIMING_LOG_MEAS_L);

        return ++t < (num_cycles << 1);
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    config_.phase_inductance = L;
    // TODO arbitrary values set for now
    if (L < 2e-6f || L > 4000e-6f)
        return set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE), false;
    return true;
}


bool Motor::run_calibration() {
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

bool Motor::enqueue_modulation_timings(float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0)
        return set_error(ERROR_MODULATION_MAGNITUDE), false;
    uint16_t next_timings[] = {
        (uint16_t)(tA * (float)period_),
        (uint16_t)(tB * (float)period_),
        (uint16_t)(tC * (float)period_)
    };
    safety_critical_apply_motor_pwm_timings(*this, period_, next_timings);
    update_brake_current();
    return true;
}

bool Motor::enqueue_voltage_timings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false;
    //log_timing(TIMING_LOG_FOC_VOLTAGE);
    return true;
}

// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool Motor::FOC_voltage(float v_d, float v_q, float pwm_phase) {
    float c = our_arm_cos_f32(pwm_phase);
    float s = our_arm_sin_f32(pwm_phase);
    float v_alpha = c*v_d - s*v_q;
    float v_beta  = c*v_q + s*v_d;
    // TODO: Ibus is not updated here, so the brake resistor will not work
    return enqueue_voltage_timings(v_alpha, v_beta);
}

bool Motor::FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase) {
    // Syntactic sugar
    CurrentControl_t& ictrl = current_control_;

    // For Reporting
    ictrl.Iq_setpoint = Iq_des;

    // Check for current sense saturation
    if (fabsf(current_meas_.phB) > ictrl.overcurrent_trip_level
     || fabsf(current_meas_.phC) > ictrl.overcurrent_trip_level) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
    }

    // Clarke transform
    float Ialpha = -current_meas_.phB - current_meas_.phC;
    float Ibeta = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);

    // Park transform
    float c_I = our_arm_cos_f32(I_phase);
    float s_I = our_arm_sin_f32(I_phase);
    float Id = c_I * Ialpha + s_I * Ibeta;
    float Iq = c_I * Ibeta - s_I * Ialpha;
    ictrl.Iq_measured += ictrl.I_measured_report_filter_k * (Iq - ictrl.Iq_measured);
    ictrl.Id_measured += ictrl.I_measured_report_filter_k * (Id - ictrl.Id_measured);

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl.v_current_control_integral_d + Ierr_d * ictrl.p_gain;
    float Vq = ictrl.v_current_control_integral_q + Ierr_q * ictrl.p_gain;

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
        ictrl.v_current_control_integral_d *= 0.99f;
        ictrl.v_current_control_integral_q *= 0.99f;
    } else {
        ictrl.v_current_control_integral_d += Ierr_d * (ictrl.i_gain * current_meas_period);
        ictrl.v_current_control_integral_q += Ierr_q * (ictrl.i_gain * current_meas_period);
    }

    // Compute estimated bus current
    ictrl.Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c_p * mod_d - s_p * mod_q;
    float mod_beta  = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl.final_v_alpha = mod_to_V * mod_alpha;
    ictrl.final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false; // error set inside enqueue_modulation_timings
    //log_timing(TIMING_LOG_FOC_CURRENT);

    return true;
}


bool Motor::update(float current_setpoint, float phase, float phase_vel) {
    current_setpoint *= config_.direction;
    phase *= config_.direction;
    phase_vel *= config_.direction;

    float pwm_phase = phase + 1.5f * current_meas_period * phase_vel;

    // Execute current command
    // TODO: move this into the mot
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {
        if(!FOC_current(0.0f, current_setpoint, phase, pwm_phase)){
            return false;
        }
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        //In gimbal motor mode, current is reinterptreted as voltage.
        if(!FOC_voltage(0.0f, current_setpoint, pwm_phase))
            return false;
    } else {
        set_error(ERROR_NOT_IMPLEMENTED_MOTOR_TYPE);
        return false;
    }
    return true;
}
