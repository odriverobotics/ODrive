
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "odrive_main.h"
#include "utils.hpp"
#include "communication/interface_can.hpp"

Axis::Axis(int axis_num,
           uint16_t default_step_gpio_pin,
           uint16_t default_dir_gpio_pin,
           osPriority thread_priority,
           Encoder& encoder,
           SensorlessEstimator& sensorless_estimator,
           Controller& controller,
           Motor& motor,
           TrapezoidalTrajectory& trap,
           Endstop& min_endstop,
           Endstop& max_endstop,
           MechanicalBrake& mechanical_brake)
    : axis_num_(axis_num),
      default_step_gpio_pin_(default_step_gpio_pin),
      default_dir_gpio_pin_(default_dir_gpio_pin),
      thread_priority_(thread_priority),
      encoder_(encoder),
      sensorless_estimator_(sensorless_estimator),
      controller_(controller),
      motor_(motor),
      trap_traj_(trap),
      min_endstop_(min_endstop),
      max_endstop_(max_endstop),
      mechanical_brake_(mechanical_brake)
{
    encoder_.axis_ = this;
    sensorless_estimator_.axis_ = this;
    controller_.axis_ = this;
    motor_.axis_ = this;
    trap_traj_.axis_ = this;
    min_endstop_.axis_ = this;
    max_endstop_.axis_ = this;
    mechanical_brake_.axis_ = this;
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

bool Axis::apply_config() {
    config_.parent = this;
    decode_step_dir_pins();
    watchdog_feed();
    return true;
}

void Axis::clear_config() {
    config_ = {};
    config_.step_gpio_pin = default_step_gpio_pin_;
    config_.dir_gpio_pin = default_dir_gpio_pin_;
    config_.can.node_id = axis_num_;
}

static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}

// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, thread_priority_, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}

/**
 * @brief Blocks until at least one complete control loop has been executed.
 */
bool Axis::wait_for_control_iteration() {
    uint16_t control_iteration_num = odrv.n_evt_control_loop_;
    while (odrv.n_evt_control_loop_ == control_iteration_num) {
        osDelay(1);
    }
    return true;
}

// step/direction interface
void Axis::step_cb() {
    if (step_dir_active_) {
        const bool dir_pin = dir_gpio_.read();
        const float dir = dir_pin ? 1.0f : -1.0f;
        controller_.input_pos_ += dir * config_.turns_per_step;
        controller_.input_pos_updated();
    }
}

void Axis::decode_step_dir_pins() {
    step_gpio_ = get_gpio(config_.step_gpio_pin);
    dir_gpio_ = get_gpio(config_.dir_gpio_pin);
}

// @brief (de)activates step/dir input
void Axis::set_step_dir_active(bool active) {
    if (active) {
        // Subscribe to rising edges of the step GPIO
        if (!step_gpio_.subscribe(true, false, step_cb_wrapper, this)) {
            odrv.misconfigured_ = true;
        }

        step_dir_active_ = true;
    } else {
        step_dir_active_ = false;

        // Unsubscribe from step GPIO
        // TODO: if we change the GPIO while the subscription is active and then
        // unsubscribe then the unsubscribe is for the wrong pin.
        step_gpio_.unsubscribe();
    }
}

// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks(uint32_t timestamp) {
    // Sub-components should use set_error which will propegate to this error_
    motor_.effective_current_lim();
    motor_.do_checks(timestamp);

    // Check for endstop presses
    if (min_endstop_.config_.enabled && min_endstop_.rose() && !(current_state_ == AXIS_STATE_HOMING)) {
        error_ |= ERROR_MIN_ENDSTOP_PRESSED;
    } else if (max_endstop_.config_.enabled && max_endstop_.rose() && !(current_state_ == AXIS_STATE_HOMING)) {
        error_ |= ERROR_MAX_ENDSTOP_PRESSED;
    }

    return check_for_errors();
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

bool Axis::run_lockin_spin(const LockinConfig_t &lockin_config, bool remain_armed) {
    CRITICAL_SECTION() {
        // Reset state variables
        open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.phase_ = 0.0f;
        open_loop_controller_.phase_vel_ = 0.0f;

        open_loop_controller_.max_current_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_voltage_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_phase_vel_ramp_ = lockin_config.accel;
        open_loop_controller_.target_current_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? lockin_config.current : 0.0f;
        open_loop_controller_.target_voltage_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : lockin_config.current;
        open_loop_controller_.target_vel_ = lockin_config.vel;
        open_loop_controller_.total_distance_ = 0.0f;

        motor_.current_control_.enable_current_control_src_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&open_loop_controller_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&open_loop_controller_.Vdq_setpoint_);

        motor_.current_control_.phase_src_.connect_to(&open_loop_controller_.phase_);
        async_estimator_.rotor_phase_src_.connect_to(&open_loop_controller_.phase_);
        
        motor_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
        motor_.current_control_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
        async_estimator_.rotor_phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
    }
    wait_for_control_iteration();

    motor_.arm(&motor_.current_control_);

    bool did_subscribe_to_idx = false;
    bool success = false;
    float dir = lockin_config.vel >= 0.0f ? 1.0f : -1.0f;

    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
        bool reached_target_vel = std::abs(open_loop_controller_.phase_vel_.get_any().value_or(0.0f) - lockin_config.vel) <= std::numeric_limits<float>::epsilon();
        bool reached_target_dist = open_loop_controller_.total_distance_.get_any().value_or(0.0f) * dir >= lockin_config.finish_distance * dir;

        // Check if terminal condition is reached
        bool terminal_condition = (reached_target_vel && lockin_config.finish_on_vel)
                               || (reached_target_dist && lockin_config.finish_on_distance)
                               || (encoder_.index_found_ && lockin_config.finish_on_enc_idx);
        if (terminal_condition) {
            success = true;
            break;
        }

        // Activate index pin as soon as target velocity was reached. This is
        // to avoid hitting the index from the wrong direction.
        if (reached_target_vel && !encoder_.index_found_ && !did_subscribe_to_idx) {
            encoder_.set_idx_subscribe(true);
            did_subscribe_to_idx = true;
        }

        osDelay(1);
    }

    if (!success || !remain_armed) {
        motor_.disarm();
    }

    return success;
}


bool Axis::start_closed_loop_control() {
    bool sensorless_mode = config_.enable_sensorless_mode;

    if (sensorless_mode) {
        // TODO: restart if desired
        if (!run_lockin_spin(config_.sensorless_ramp, true)) {
            return false;
        }
    }

    // Hook up the data paths between the components
    CRITICAL_SECTION() {
        if (sensorless_mode) {
            controller_.pos_estimate_linear_src_.disconnect();
            controller_.pos_estimate_circular_src_.disconnect();
            controller_.pos_wrap_src_.disconnect();
            controller_.vel_estimate_src_.connect_to(&sensorless_estimator_.vel_estimate_);
        } else if (controller_.config_.load_encoder_axis < AXIS_COUNT) {
            Axis* ax = &axes[controller_.config_.load_encoder_axis];
            controller_.pos_estimate_circular_src_.connect_to(&ax->encoder_.pos_circular_);
            controller_.pos_wrap_src_.connect_to(&controller_.config_.circular_setpoint_range);
            controller_.pos_estimate_linear_src_.connect_to(&ax->encoder_.pos_estimate_);
            controller_.vel_estimate_src_.connect_to(&ax->encoder_.vel_estimate_);
        } else {
            controller_.pos_estimate_circular_src_.disconnect();
            controller_.pos_estimate_linear_src_.disconnect();
            controller_.pos_wrap_src_.disconnect();
            controller_.vel_estimate_src_.disconnect();
            controller_.set_error(Controller::ERROR_INVALID_LOAD_ENCODER);
            return false;
        }

        // To avoid any transient on startup, we intialize the setpoint to be the current position
        // note - input_pos_ is not set here. It is set to 0 earlier in this method and velocity control is used.
        if (controller_.config_.control_mode >= Controller::CONTROL_MODE_POSITION_CONTROL) {
            std::optional<float> pos_init = (controller_.config_.circular_setpoints ?
                                    controller_.pos_estimate_circular_src_ :
                                    controller_.pos_estimate_linear_src_).get_any();
            if (!pos_init.has_value()) {
                return false;
            } else {
                controller_.pos_setpoint_ = *pos_init;
                controller_.input_pos_ = *pos_init;
            }
        }
        controller_.input_pos_updated();

        // Avoid integrator windup issues
        controller_.vel_integrator_torque_ = 0.0f;

        motor_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
        motor_.direction_ = sensorless_mode ? 1.0f : encoder_.config_.direction;

        motor_.current_control_.enable_current_control_src_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&motor_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&motor_.Vdq_setpoint_);

        OutputPort<float>* phase_src = sensorless_mode ? &sensorless_estimator_.phase_ : &encoder_.phase_;
        motor_.current_control_.phase_src_.connect_to(phase_src);
        async_estimator_.rotor_phase_src_.connect_to(phase_src);
        
        OutputPort<float>* phase_vel_src = sensorless_mode ? &sensorless_estimator_.phase_vel_ : &encoder_.phase_vel_;
        motor_.phase_vel_src_.connect_to(phase_vel_src);
        motor_.current_control_.phase_vel_src_.connect_to(phase_vel_src);
        async_estimator_.rotor_phase_vel_src_.connect_to(phase_vel_src);
        
        if (sensorless_mode) {
            // Make the final velocity of the loĉk-in spin the setpoint of the
            // closed loop controller to allow for smooth transition.
            float vel = config_.sensorless_ramp.vel / (2.0f * M_PI * motor_.config_.pole_pairs);
            controller_.input_vel_ = vel;
            controller_.vel_setpoint_ = vel;
        }
    }

    // In sensorless mode the motor is already armed.
    if (!motor_.is_armed_) {
        wait_for_control_iteration();
        motor_.arm(&motor_.current_control_);
    }

    return true;
}

bool Axis::stop_closed_loop_control() {
    motor_.disarm();
    return check_for_errors();
}

bool Axis::run_closed_loop_control_loop() {
    start_closed_loop_control();
    set_step_dir_active(config_.enable_step_dir);

    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
        osDelay(1);
    }

    set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    stop_closed_loop_control();

    return check_for_errors();
}


// Slowly drive in the negative direction at homing_speed until the min endstop is pressed
// When pressed, set the linear count to the offset (default 0), and then go to position 0
bool Axis::run_homing() {
    Controller::ControlMode stored_control_mode = controller_.config_.control_mode;
    Controller::InputMode stored_input_mode = controller_.config_.input_mode;

    // TODO: theoretically this check should be inside the update loop,
    // otherwise someone could disable the endstop while homing is in progress.
    if (!min_endstop_.config_.enabled) {
        return error_ |= ERROR_HOMING_WITHOUT_ENDSTOP, false;
    }

    controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;

    controller_.input_pos_ = 0.0f;
    controller_.input_pos_updated();
    controller_.input_vel_ = -controller_.config_.homing_speed;
    controller_.input_torque_ = 0.0f;

    homing_.is_homed = false;

    start_closed_loop_control();

    // Driving toward the endstop
    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_ && !min_endstop_.get_state()) {
        osDelay(1);
    }

    stop_closed_loop_control();

    error_ &= ~ERROR_MIN_ENDSTOP_PRESSED; // clear this error since we deliberately drove into the endstop

    // pos_setpoint is the starting position for the trap_traj so we need to set it.
    controller_.pos_setpoint_ = min_endstop_.config_.offset;
    controller_.vel_setpoint_ = 0.0f;  // Change directions without decelerating

    // Set our current position in encoder counts to make control more logical
    encoder_.set_linear_count((int32_t)(controller_.pos_setpoint_ * encoder_.config_.cpr));

    controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;

    controller_.input_pos_ = 0.0f;
    controller_.input_pos_updated();
    controller_.input_vel_ = 0.0f;
    controller_.input_torque_ = 0.0f;

    start_closed_loop_control();

    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_ && !controller_.trajectory_done_) {
        osDelay(1);
    }

    stop_closed_loop_control();

    controller_.config_.control_mode = stored_control_mode;
    controller_.config_.input_mode = stored_input_mode;
    homing_.is_homed = true;

    return check_for_errors();
}

bool Axis::run_idle_loop() {
    mechanical_brake_.engage();
    set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    while (requested_state_ == AXIS_STATE_UNDEFINED) {
        motor_.setup();
        osDelay(1);
    }
    return check_for_errors();
}

// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {

    // Wait for up to 2s for motor to become ready to allow for error-free
    // startup. This delay gives the current sensor calibration time to
    // converge. If the DRV chip is unpowered, the motor will not become ready
    // but we still enter idle state.
    for (size_t i = 0; i < 2000; ++i) {
        if (motor_.current_meas_.has_value()) {
            break;
        }
        osDelay(1);
    }

    sensorless_estimator_.error_ &= ~SensorlessEstimator::ERROR_UNKNOWN_CURRENT_MEASUREMENT;

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
                if (!motor_.is_calibrated_ || encoder_.config_.direction==0)
                    goto invalid_state_label;
                status = run_lockin_spin(config_.general_lockin, false);
            } break;

            case AXIS_STATE_CLOSED_LOOP_CONTROL: {
                if (!motor_.is_calibrated_ || (encoder_.config_.direction==0 && !config_.enable_sensorless_mode))
                    goto invalid_state_label;
                watchdog_feed();
                status = run_closed_loop_control_loop();
            } break;

            case AXIS_STATE_IDLE: {
                run_idle_loop();
                status = true;
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
