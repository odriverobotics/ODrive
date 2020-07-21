
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "odrive_main.h"
#include "utils.hpp"
#include "gpio_utils.hpp"
#include "communication/interface_can.hpp"

Axis::Axis(int axis_num,
           const AxisHardwareConfig_t& hw_config,
           Config_t& config,
           Encoder& encoder,
           SensorlessEstimator& sensorless_estimator,
           Controller& controller,
           Motor& motor,
           TrapezoidalTrajectory& trap,
           Endstop& min_endstop,
           Endstop& max_endstop): 
      axis_num_(axis_num),
      hw_config_(hw_config),
      config_(config),
      encoder_(encoder),
      sensorless_estimator_(sensorless_estimator),
      controller_(controller),
      motor_(motor),
      trap_(trap),
      min_endstop_(min_endstop),
      max_endstop_(max_endstop)
{
    encoder_.axis_ = this;
    sensorless_estimator_.axis_ = this;
    controller_.axis_ = this;
    motor_.axis_ = this;
    trap_.axis_ = this;
    min_endstop_.axis_ = this;
    max_endstop_.axis_ = this;
    decode_step_dir_pins();
    watchdog_feed();
}

Axis::LockinConfig_t Axis::default_calibration() {
    Axis::LockinConfig_t config;
    config.current = 10.0f;           // [A]
    config.ramp_time = 0.4f;          // [s]
    config.ramp_distance = 1 * M_PI;  // [rad]
    config.accel = 20.0f;     // [rad/s^2]
    config.vel = 40.0f; // [rad/s]
    config.finish_distance = 100.0f * 2.0f * M_PI;  // [rad]
    config.finish_on_vel = false;
    config.finish_on_distance = true;
    config.finish_on_enc_idx = true;
    return config;
}

Axis::LockinConfig_t Axis::default_sensorless() {
    Axis::LockinConfig_t config;
    config.current = 10.0f;           // [A]
    config.ramp_time = 0.4f;          // [s]
    config.ramp_distance = 1 * M_PI;  // [rad]
    config.accel = 200.0f;     // [rad/s^2]
    config.vel = 400.0f; // [rad/s]
    config.finish_distance = 100.0f;  // [rad]
    config.finish_on_vel = true;
    config.finish_on_distance = false;
    config.finish_on_enc_idx = false;
    return config;
}

static void step_cb_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->step_cb();
}

void Axis::use_enable_pin_update() {
    if (config_.use_enable_pin) {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = en_pin_;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        if (config_.enable_pin_active_low) {
            GPIO_InitStruct.Pull = GPIO_PULLUP;
        } else {
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        }
        HAL_GPIO_Init(en_port_, &GPIO_InitStruct);
    }
}
// @brief Sets up all components of the axis,
// such as gate driver and encoder hardware.
void Axis::setup() {
    use_enable_pin_update();

    motor_.setup();
}

static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}

// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, hw_config_.thread_priority, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}

// @brief Unblocks the control loop thread.
// This is called from the current sense interrupt handler.
void Axis::signal_current_meas() {
    if (thread_id_valid_)
        osSignalSet(thread_id_, M_SIGNAL_PH_CURRENT_MEAS);
}

// @brief Blocks until a current measurement is completed
// @returns True on success, false otherwise
bool Axis::wait_for_current_meas() {
    return osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status == osEventSignal;
}

// step/direction interface
void Axis::step_cb() {
    if (step_dir_active_) {
        GPIO_PinState dir_pin = HAL_GPIO_ReadPin(dir_port_, dir_pin_);
        float dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
        controller_.input_pos_ += dir * config_.counts_per_step;
        controller_.input_pos_updated();
    }
};

void Axis::load_default_step_dir_pin_config(const AxisHardwareConfig_t& hw_config, Config_t* config) {
        config->step_gpio_pin = hw_config.step_gpio_pin;
        config->dir_gpio_pin = hw_config.dir_gpio_pin;
        config->en_gpio_pin = hw_config.en_gpio_pin;
}

void Axis::load_default_can_id(const int& id, Config_t& config){
    config.can_node_id = id;
}

void Axis::decode_step_dir_pins() {
    step_port_ = get_gpio_port_by_pin(config_.step_gpio_pin);
    step_pin_ = get_gpio_pin_by_pin(config_.step_gpio_pin);
    dir_port_ = get_gpio_port_by_pin(config_.dir_gpio_pin);
    dir_pin_ = get_gpio_pin_by_pin(config_.dir_gpio_pin);
    en_port_ = get_gpio_port_by_pin(config_.en_gpio_pin);
    en_pin_ = get_gpio_pin_by_pin(config_.en_gpio_pin);
}

// @brief (de)activates step/dir input
void Axis::set_step_dir_active(bool active) {
    if (active) {
        // Set up the direction GPIO as input
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = dir_pin_;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(dir_port_, &GPIO_InitStruct);
        // Subscribe to rising edges of the step GPIO
        GPIO_subscribe(step_port_, step_pin_, GPIO_PULLDOWN, step_cb_wrapper, this);

        step_dir_active_ = true;
    } else {
        step_dir_active_ = false;

        // Unsubscribe from step GPIO
        GPIO_unsubscribe(step_port_, step_pin_);
    }
}

// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks() {
    if (!brake_resistor_armed)
        error_ |= ERROR_BRAKE_RESISTOR_DISARMED;
    if ((current_state_ != AXIS_STATE_IDLE) && (motor_.armed_state_ == Motor::ARMED_STATE_DISARMED))
        // motor got disarmed in something other than the idle loop
        error_ |= ERROR_MOTOR_DISARMED;
    if (!(vbus_voltage >= board_config.dc_bus_undervoltage_trip_level))
        error_ |= ERROR_DC_BUS_UNDER_VOLTAGE;
    if (!(vbus_voltage <= board_config.dc_bus_overvoltage_trip_level))
        error_ |= ERROR_DC_BUS_OVER_VOLTAGE;

    // Sub-components should use set_error which will propegate to this error_
    motor_.do_checks();
    // encoder_.do_checks();
    // sensorless_estimator_.do_checks();
    // controller_.do_checks();

    // Check for endstop presses
    if (min_endstop_.config_.enabled && min_endstop_.get_state() && !(current_state_ == AXIS_STATE_HOMING)) {
        error_ |= ERROR_MIN_ENDSTOP_PRESSED;
    } else if (max_endstop_.config_.enabled && max_endstop_.get_state() && !(current_state_ == AXIS_STATE_HOMING)) {
        error_ |= ERROR_MAX_ENDSTOP_PRESSED;
    }

    return check_for_errors();
}

// @brief Update all esitmators
bool Axis::do_updates() {
    // Sub-components should use set_error which will propegate to this error_
    encoder_.update();
    sensorless_estimator_.update();
    min_endstop_.update();
    max_endstop_.update();
    bool ret = check_for_errors();
    odCAN->send_heartbeat(this);
    return ret;
}

// @brief Feed the watchdog to prevent watchdog timeouts.
void Axis::watchdog_feed() {
    watchdog_current_value_ = get_watchdog_reset();
}

// @brief Check the watchdog timer for expiration. Also sets the watchdog error bit if expired.
bool Axis::watchdog_check() {
    if (!config_.enable_watchdog) return true;

    // explicit check here to ensure that we don't underflow back to UINT32_MAX
    if (watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } else {
        error_ |= ERROR_WATCHDOG_TIMER_EXPIRED;
        return false;
    }
}

bool Axis::run_lockin_spin(const LockinConfig_t &lockin_config) {
    // Spiral up current for softer rotor lock-in
    lockin_state_ = LOCKIN_STATE_RAMP;
    float x = 0.0f;
    run_control_loop([&]() {
        float phase = wrap_pm_pi(lockin_config.ramp_distance * x);
        float I_mag = lockin_config.current * x;
        x += current_meas_period / lockin_config.ramp_time;
        if (!motor_.update(I_mag, phase, 0.0f))
            return false;
        return x < 1.0f;
    });
    
    // Spin states
    float distance = lockin_config.ramp_distance;
    float phase = wrap_pm_pi(distance);
    float vel = distance / lockin_config.ramp_time;

    // Function of states to check if we are done
    auto spin_done = [&](bool vel_override = false) -> bool {
        bool done = false;
        if (lockin_config.finish_on_vel || vel_override)
            done = done || std::abs(vel) >= std::abs(lockin_config.vel);
        if (lockin_config.finish_on_distance)
            done = done || std::abs(distance) >= std::abs(lockin_config.finish_distance);
        if (lockin_config.finish_on_enc_idx)
            done = done || encoder_.index_found_;
        return done;
    };

    // Accelerate
    lockin_state_ = LOCKIN_STATE_ACCELERATE;
    run_control_loop([&]() {
        vel += lockin_config.accel * current_meas_period;
        distance += vel * current_meas_period;
        phase = wrap_pm_pi(phase + vel * current_meas_period);

        if (!motor_.update(lockin_config.current, phase, vel))
            return false;
        return !spin_done(true); //vel_override to go to next phase
    });

    if (!encoder_.index_found_)
        encoder_.set_idx_subscribe(true);

    // Constant speed
    if (!spin_done()) {
        lockin_state_ = LOCKIN_STATE_CONST_VEL;
        vel = lockin_config.vel; // reset to actual specified vel to avoid small integration error
        run_control_loop([&]() {
            distance += vel * current_meas_period;
            phase = wrap_pm_pi(phase + vel * current_meas_period);

            if (!motor_.update(lockin_config.current, phase, vel))
                return false;
            return !spin_done();
        });
    }

    lockin_state_ = LOCKIN_STATE_INACTIVE;
    return check_for_errors();
}

// Note run_sensorless_control_loop and run_closed_loop_control_loop are very similar and differ only in where we get the estimate from.
bool Axis::run_sensorless_control_loop() {
    controller_.pos_estimate_src_ = nullptr;
    controller_.pos_estimate_valid_src_ = nullptr;
    controller_.vel_estimate_src_ = &sensorless_estimator_.vel_estimate_;
    controller_.vel_estimate_valid_src_ = &sensorless_estimator_.vel_estimate_valid_;

    run_control_loop([this](){
        // Note that all estimators are updated in the loop prefix in run_control_loop
        float current_setpoint;
        if (!controller_.update(&current_setpoint))
            return error_ |= ERROR_CONTROLLER_FAILED, false;
        if (!motor_.update(current_setpoint, sensorless_estimator_.phase_, sensorless_estimator_.vel_estimate_))
            return false; // set_error should update axis.error_
        return true;
    });
    return check_for_errors();
}
void Axis::enable_pin_check() {
    if (!encoder_.config_.pre_calibrated){return;}     // No work until encoder pre_calibrated set True
    if (error_ != ERROR_NONE ) return;
    if (config_.use_enable_pin) {
        bool enable = HAL_GPIO_ReadPin(en_port_, en_pin_) ^ config_.enable_pin_active_low;
        switch(current_state_){
            case AXIS_STATE_IDLE: 
                if (enable) requested_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL ;
                break;
            case AXIS_STATE_CLOSED_LOOP_CONTROL:
                if(!enable) requested_state_ = AXIS_STATE_IDLE ;
                break;
            default :
                break;
        }
       
       /* if (enable && (current_state_ == AXIS_STATE_IDLE)) {
            requested_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;   
        } else if (!enable && (current_state_ != AXIS_STATE_IDLE)) {
            requested_state_ = AXIS_STATE_IDLE;
        }*/
    }
}
bool Axis::run_closed_loop_control_loop() {
    if (!controller_.select_encoder(controller_.config_.load_encoder_axis)) {
        return error_ |= ERROR_CONTROLLER_FAILED, false;
    }
    

    // To avoid any transient on startup, we intialize the setpoint to be the current position
    controller_.pos_setpoint_ = *controller_.pos_estimate_src_;

    controller_.input_pos_ = *controller_.pos_estimate_src_;

    // Avoid integrator windup issues
    controller_.vel_integrator_current_ = 0.0f;

    set_step_dir_active(config_.enable_step_dir);
    run_control_loop([this](){
        //check enable pin
        // Note that all estimators are updated in the loop prefix in run_control_loop
        float current_setpoint;
        if (!controller_.update(&current_setpoint))
            return error_ |= ERROR_CONTROLLER_FAILED, false;

        float phase_vel = 2 * M_PI * encoder_.vel_estimate_ / (float)encoder_.config_.cpr * motor_.config_.pole_pairs;
        if (!motor_.update(current_setpoint, encoder_.phase_, phase_vel))
            return false; // set_error should update axis.error_

        return true;
    });
    set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    return check_for_errors();
}


// Slowly drive in the negative direction at homing_speed until the min endstop is pressed
// When pressed, set the linear count to the offset (default 0), and then go to position 0
bool Axis::run_homing() {
    Controller::ControlMode_t stored_control_mode = controller_.config_.control_mode;
    Controller::InputMode_t stored_input_mode = controller_.config_.input_mode;

    // TODO: theoretically this check should be inside the update loop,
    // otherwise someone could disable the endstop while homing is in progress.
    if (!min_endstop_.config_.enabled) {
        return error_ |= ERROR_HOMING_WITHOUT_ENDSTOP, false;
    }

    controller_.config_.control_mode = Controller::CTRL_MODE_VELOCITY_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;

    controller_.input_pos_ = 0.0f;
    controller_.input_pos_updated();
    controller_.input_vel_ = -controller_.config_.homing_speed;
    controller_.input_current_ = 0.0f;

    homing_.is_homed = false;

    if (!controller_.select_encoder(controller_.config_.load_encoder_axis)) {
        return error_ |= ERROR_CONTROLLER_FAILED, false;
    }
    
    // To avoid any transient on startup, we intialize the setpoint to be the current position
    controller_.pos_setpoint_ = *controller_.pos_estimate_src_;

    // Avoid integrator windup issues
    controller_.vel_integrator_current_ = 0.0f;

    run_control_loop([this](){
        // Note that all estimators are updated in the loop prefix in run_control_loop
        float current_setpoint;
        if (!controller_.update(&current_setpoint))
            return error_ |= ERROR_CONTROLLER_FAILED, false;

        float phase_vel = 2 * M_PI * encoder_.vel_estimate_ / (float)encoder_.config_.cpr * motor_.config_.pole_pairs;
        if (!motor_.update(current_setpoint, encoder_.phase_, phase_vel))
            return false; // set_error should update axis.error_

        return !min_endstop_.get_state();
    });
    error_ &= ~ERROR_MIN_ENDSTOP_PRESSED; // clear this error since we deliberately drove into the endstop

    // pos_setpoint is the starting position for the trap_traj so we need to set it.
    controller_.pos_setpoint_ = min_endstop_.config_.offset;
    controller_.vel_setpoint_ = 0.0f;  // Change directions without decelerating

    // Set our current position in encoder counts to make control more logical
    encoder_.set_linear_count((int32_t)controller_.pos_setpoint_);

    controller_.config_.control_mode = Controller::CTRL_MODE_POSITION_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;

    controller_.input_pos_ = 0.0f;
    controller_.input_pos_updated();
    controller_.input_vel_ = 0.0f;
    controller_.input_current_ = 0.0f;

    run_control_loop([this](){
        // Note that all estimators are updated in the loop prefix in run_control_loop
        float current_setpoint;
        if (!controller_.update(&current_setpoint))
            return error_ |= ERROR_CONTROLLER_FAILED, false;

        float phase_vel = 2 * M_PI * encoder_.vel_estimate_ / (float)encoder_.config_.cpr * motor_.config_.pole_pairs;
        if (!motor_.update(current_setpoint, encoder_.phase_, phase_vel))
            return false; // set_error should update axis.error_

        return !controller_.trajectory_done_;
    });

    controller_.config_.control_mode = stored_control_mode;
    controller_.config_.input_mode = stored_input_mode;
    homing_.is_homed = true;

    return check_for_errors();
}

bool Axis::run_idle_loop() {
    // run_control_loop ignores missed modulation timing updates
    // if and only if we're in AXIS_STATE_IDLE
    safety_critical_disarm_motor_pwm(motor_);
    set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    run_control_loop([this]() {
        return true;
    });
    return check_for_errors();
}

// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {

    // arm!
    motor_.arm();

    for (;;) {
        // Load the task chain if a specific request is pending
        if (requested_state_ != AXIS_STATE_UNDEFINED) {
            size_t pos = 0;
            if (requested_state_ == AXIS_STATE_STARTUP_SEQUENCE) {
                if (config_.startup_motor_calibration)
                    task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                if (config_.startup_encoder_index_search && encoder_.config_.use_index)
                    task_chain_[pos++] = AXIS_STATE_ENCODER_INDEX_SEARCH;
                if (config_.startup_encoder_offset_calibration)
                    task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                if (config_.startup_homing)
                    task_chain_[pos++] = AXIS_STATE_HOMING;
                if (config_.startup_closed_loop_control)
                    task_chain_[pos++] = AXIS_STATE_CLOSED_LOOP_CONTROL;
                else if (config_.startup_sensorless_control)
                    task_chain_[pos++] = AXIS_STATE_SENSORLESS_CONTROL;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } else if (requested_state_ == AXIS_STATE_FULL_CALIBRATION_SEQUENCE) {
                task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                if (encoder_.config_.use_index)
                    task_chain_[pos++] = AXIS_STATE_ENCODER_INDEX_SEARCH;
                task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } else if (requested_state_ != AXIS_STATE_UNDEFINED) {
                task_chain_[pos++] = requested_state_;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            }
            task_chain_[pos++] = AXIS_STATE_UNDEFINED;  // TODO: bounds checking
            requested_state_ = AXIS_STATE_UNDEFINED;
            // Auto-clear any invalid state error
            error_ &= ~ERROR_INVALID_STATE;
        }

        // Note that current_state is a reference to task_chain_[0]

        // Run the specified state
        // Handlers should exit if requested_state != AXIS_STATE_UNDEFINED
        bool status;
        switch (current_state_) {
            case AXIS_STATE_MOTOR_CALIBRATION: {
                status = motor_.run_calibration();
            } break;

            case AXIS_STATE_ENCODER_INDEX_SEARCH: {
                if (!motor_.is_calibrated_)
                    goto invalid_state_label;
                if (encoder_.config_.idx_search_unidirectional && motor_.config_.direction==0)
                    goto invalid_state_label;

                status = encoder_.run_index_search();
            } break;

            case AXIS_STATE_ENCODER_DIR_FIND: {
                if (!motor_.is_calibrated_)
                    goto invalid_state_label;

                status = encoder_.run_direction_find();
            } break;

            case AXIS_STATE_HOMING: {
                status = run_homing();
            } break;

            case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
                if (!motor_.is_calibrated_)
                    goto invalid_state_label;
                status = encoder_.run_offset_calibration();
            } break;

            case AXIS_STATE_LOCKIN_SPIN: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0)
                    goto invalid_state_label;
                status = run_lockin_spin(config_.lockin);
            } break;

            case AXIS_STATE_SENSORLESS_CONTROL: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0)
                        goto invalid_state_label;
                status = run_lockin_spin(config_.sensorless_ramp); // TODO: restart if desired
                if (status) {
                    // call to controller.reset() that happend when arming means that vel_setpoint
                    // is zeroed. So we make the setpoint the spinup target for smooth transition.
                    controller_.vel_setpoint_ = config_.sensorless_ramp.vel;
                    status = run_sensorless_control_loop();
                }
            } break;

            case AXIS_STATE_CLOSED_LOOP_CONTROL: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0)
                    goto invalid_state_label;
                if (!encoder_.is_ready_)
                    goto invalid_state_label;
                watchdog_feed();
                status = run_closed_loop_control_loop();  
            } break;

            case AXIS_STATE_IDLE: {
                run_idle_loop();
                status = motor_.arm(); // done with idling - try to arm the motor
            } break;

            default:
            invalid_state_label:
                error_ |= ERROR_INVALID_STATE;
                status = false;  // this will set the state to idle
                break;
        }

        // If the state failed, go to idle, else advance task chain
        if (!status) {
            std::fill(task_chain_.begin(), task_chain_.end(), AXIS_STATE_UNDEFINED);
            current_state_ = AXIS_STATE_IDLE;
        } else {
            std::rotate(task_chain_.begin(), task_chain_.begin() + 1, task_chain_.end());
            task_chain_.back() = AXIS_STATE_UNDEFINED;
        }
    }
}
