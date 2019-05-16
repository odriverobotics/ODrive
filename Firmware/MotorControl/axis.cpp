
#include "axis.hpp"
#include "low_level.h"
#include <stm32_gpio.hpp>
#include <functional>
#include <stdlib.h>

Axis::Axis(Motor& motor,
           Encoder encoder,
           SensorlessEstimator sensorless_estimator,
           AsyncEstimator async_estimator,
           Controller controller,
           TrapezoidalTrajectory trap,
           osPriority thread_priority,
           Config_t& config) :
      motor_(motor),
      encoder_(encoder),
      sensorless_estimator_(sensorless_estimator),
      async_estimator_(async_estimator),
      controller_(controller),
      trap_(trap),
      thread_priority_(thread_priority),
      config_(config) {
    motor_.axis_ = this;
    encoder_.axis_ = this;
    sensorless_estimator_.axis_ = this;
    async_estimator_.axis_ = this;
    controller_.axis_ = this;
    trap_.axis_ = this;
}

// @brief Sets up all components of the axis,
// such as gate driver and encoder hardware.
bool Axis::init() {
    if (!motor_.init())
        return false;
    if (!encoder_.init())
        return false;
    if (!sensorless_estimator_.init())
        return false;
    if (!async_estimator_.init())
        return false;
    if (!controller_.init())
        return false;
    if (!trap_.init())
        return false;

    decode_step_dir_pins();
    update_watchdog_settings();
    return true;
}

static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}

// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, thread_priority_, 0, 4 * 512);
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}

// @brief Unblocks the control loop thread.
// This is called from the current sense interrupt handler.
void Axis::signal_current_meas() {
    if (thread_id_valid_) {
        current_meas_events_++;
        osSignalSet(thread_id_, M_SIGNAL_PH_CURRENT_MEAS);
    }
}

// @brief Blocks until a current measurement is completed
// @returns True on success, false otherwise
bool Axis::wait_for_current_meas() {
    uint64_t start = get_ticks_us();
    uint32_t meas_evt = current_meas_events_;
    uint32_t updt_evt = motor_.update_events_;
    bool result = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status == osEventSignal;
    uint64_t wait = get_ticks_us() - start;
    if (wait > max_wait_us_) {
        max_wait_us_ = wait;
        max_wait_meas_evt_ = current_meas_events_ - meas_evt;
        max_wait_updt_evt_ = motor_.update_events_ - updt_evt;
    }
    return result;
}

// step/direction interface
void Axis::step_cb() {
    if (step_dir_active_) {
        bool dir_pin = dir_gpio_->read();
        float dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
        controller_.pos_setpoint_ += dir * config_.counts_per_step;
    }
}

void Axis::decode_step_dir_pins() {
    if (step_gpio_)
        step_gpio_->deinit();
    if (dir_gpio_)
        dir_gpio_->deinit();
    if (config_.step_gpio_num < num_gpios)
        step_gpio_ = gpios[config_.step_gpio_num];
    else
        step_gpio_ = nullptr;
    if (config_.dir_gpio_num < num_gpios)
        dir_gpio_ = gpios[config_.dir_gpio_num];
    else
        dir_gpio_ = nullptr;
    // TODO: reinit GPIOs here
}

// @brief: Setup the watchdog reset value from the configuration watchdog timeout interval. 
void Axis::update_watchdog_settings() {

    if(config_.watchdog_timeout <= 0.0f) { // watchdog disabled 
        watchdog_reset_value_ = 0;
    } else if(config_.watchdog_timeout >= UINT32_MAX / (current_meas_hz+1)) { //overflow! 
        watchdog_reset_value_ = UINT32_MAX;
    } else {
        watchdog_reset_value_ = static_cast<uint32_t>(config_.watchdog_timeout * current_meas_hz);
    }

    // Do a feed to avoid instant timeout
    watchdog_feed();
}

// @brief (de)activates step/dir input
void Axis::set_step_dir_active(bool active) {
    if (active) {
        if (dir_gpio_) {
            dir_gpio_->init(GPIO_t::INPUT, GPIO_t::NO_PULL);
        }
        if (step_gpio_) {
            step_gpio_->init(GPIO_t::INPUT, GPIO_t::PULL_DOWN);
            step_gpio_->subscribe(true, false,
                [](void* ctx){ reinterpret_cast<Axis*>(ctx)->step_cb(); }, this);
        }

        step_dir_active_ = true;
    } else {
        step_dir_active_ = false;

        if (step_gpio_)
            step_gpio_->deinit();
        if (dir_gpio_)
            dir_gpio_->deinit();
    }
}

// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks() {
    if (brake_resistor_enabled && !brake_resistor_armed)
        error_ |= ERROR_BRAKE_RESISTOR_DISARMED;
    if ((current_state_ != AXIS_STATE_IDLE) && !motor_.is_armed_)
        // motor got disarmed in something other than the idle loop
        error_ |= ERROR_MOTOR_DISARMED;

    // Sub-components should use set_error
    return check_for_errors();
}

// @brief Update all esitmators
bool Axis::do_updates(float dt) {
    // Sub-components should use set_error which will propagate to this error_
    encoder_.update(dt);
    sensorless_estimator_.update(dt);
    async_estimator_.update(dt);
    did_update_.invoke(this);
    return check_for_errors();
}

// @brief Feed the watchdog to prevent watchdog timeouts.
void Axis::watchdog_feed() {
    watchdog_current_value_ = watchdog_reset_value_;
}

// @brief Check the watchdog timer for expiration. Also sets the watchdog error bit if expired. 
bool Axis::watchdog_check() {
    // reset value = 0 means watchdog disabled. 
    if(watchdog_reset_value_ == 0) return true;

    // explicit check here to ensure that we don't underflow back to UINT32_MAX
    if(watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } else {
        error_ |= ERROR_WATCHDOG_TIMER_EXPIRED;
        return false;
    }
}

bool Axis::run_lockin_spin() {
    // Spiral up current for softer rotor lock-in
    lockin_state_ = LOCKIN_STATE_RAMP;

    uint32_t start_ms = get_ticks_ms();
    float phase_vel = config_.lockin.ramp_distance / config_.lockin.ramp_time;
    if (!motor_.arm_foc())
        return error_ |= ERROR_MOTOR_FAILED, false;
    run_control_loop([&](float dt) {
        uint32_t delta_ms = get_ticks_ms() - start_ms;
        float x = delta_ms / (config_.lockin.ramp_time * 1000.0f);
        float phase = wrap_pm_pi(config_.lockin.ramp_distance * x);
        float I_mag = std::min(std::max(x, 0.0f), 1.0f) * config_.lockin.current;
        return motor_.FOC_update(0.0f, I_mag, phase, phase_vel);
    });
    
    // Spin states
    float distance = config_.lockin.ramp_distance;
    float phase = wrap_pm_pi(distance);
    float vel = distance / config_.lockin.ramp_time;

    // Function of states to check if we are done
    auto spin_done = [&](bool vel_override = false) -> bool {
        bool done = false;
        if (config_.lockin.finish_on_vel || vel_override)
            done = done || fabsf(vel) >= fabsf(config_.lockin.vel);
        if (config_.lockin.finish_on_distance)
            done = done || fabsf(distance) >= fabsf(config_.lockin.finish_distance);
        if (config_.lockin.finish_on_enc_idx)
            done = done || encoder_.index_found_;
        return done;
    };

    // Accelerate
    lockin_state_ = LOCKIN_STATE_ACCELERATE;
    run_control_loop([&](float dt) {
        vel += config_.lockin.accel * dt;
        distance += vel * dt;
        phase = wrap_pm_pi(phase + vel * dt);

        if (encoder_.error_ != Encoder::ERROR_NONE)
            return error_ |= ERROR_ENCODER_FAILED, false;
        if (!motor_.FOC_update(0.0f, config_.lockin.current, phase, vel))
            return false;
        return !spin_done(true); //vel_override to go to next phase
    });

    if (!encoder_.index_found_)
        encoder_.set_idx_subscribe(true);

    // Constant speed
    if (!spin_done()) {
        lockin_state_ = LOCKIN_STATE_CONST_VEL;
        vel = config_.lockin.vel; // reset to actual specified vel to avoid small integration error
        run_control_loop([&](float dt) {
            distance += vel * dt;
            phase = wrap_pm_pi(phase + vel * dt);

            if (encoder_.error_ != Encoder::ERROR_NONE)
                return error_ |= ERROR_ENCODER_FAILED, false;
            if (!motor_.FOC_update(0.0f, config_.lockin.current, phase, vel))
                return false;
            return !spin_done();
        });
    }

    lockin_state_ = LOCKIN_STATE_INACTIVE;
    return check_for_errors();
}

// Note run_sensorless_control_loop and run_closed_loop_control_loop are very similar and differ only in where we get the estimate from.
bool Axis::run_sensorless_control_loop() {
    if (!motor_.arm_foc())
        return error_ |= ERROR_MOTOR_FAILED, false;
    run_control_loop([this](float dt){
        if (controller_.config_.control_mode >= Controller::CTRL_MODE_POSITION_CONTROL)
            return error_ |= ERROR_POS_CTRL_DURING_SENSORLESS, false;

        // Note that all estimators are updated in the loop prefix in run_control_loop
        float current_setpoint;
        if (sensorless_estimator_.error_ != SensorlessEstimator::ERROR_NONE)
            return error_ |= ERROR_SENSORLESS_ESTIMATOR_FAILED, false;
        if (!controller_.update(dt, sensorless_estimator_.pll_pos_, sensorless_estimator_.vel_estimate_, &current_setpoint))
            return error_ |= ERROR_CONTROLLER_FAILED, false;
        if (!motor_.FOC_update(0.0f, current_setpoint, sensorless_estimator_.phase_, sensorless_estimator_.vel_estimate_))
            return false; // set_error should update axis.error_
        return true;
    });
    return check_for_errors();
}

bool Axis::run_closed_loop_control_loop() {
    // To avoid any transient on startup, we intialize the setpoint to be the current position
    controller_.pos_setpoint_ = encoder_.pos_estimate_;
    set_step_dir_active(config_.enable_step_dir);
    if (!motor_.arm_foc())
        return error_ |= ERROR_MOTOR_FAILED, false;
    run_control_loop([this](float dt){
        // Note that all estimators are updated in the loop prefix in run_control_loop
        float current_setpoint;
        if (encoder_.error_ != Encoder::ERROR_NONE)
            return error_ |= ERROR_ENCODER_FAILED, false;
        if (!controller_.update(dt, encoder_.pos_estimate_, encoder_.vel_estimate_, &current_setpoint))
            return error_ |= ERROR_CONTROLLER_FAILED, false; //TODO: Make controller.set_error
        float phase_vel = 2*M_PI * encoder_.vel_estimate_ / (float)encoder_.config_.cpr * motor_.config_.pole_pairs;
        if (!motor_.FOC_update(0.0f, current_setpoint, encoder_.phase_, phase_vel))
            return false; // set_error should update axis.error_
        return true;
    });
    set_step_dir_active(false);
    return check_for_errors();
}




/**
 * @brief Spins the magnetic field at a fixed velocity (defined by the velocity
 * setpoint) and current/voltage setpoint. The current controller still runs in
 * closed loop mode.
 */
bool Axis::run_open_loop_control_loop() {
    if (!motor_.arm_foc())
        return error_ |= ERROR_MOTOR_FAILED, false;

    set_step_dir_active(config_.enable_step_dir);

    float phase = 0.0f;
    run_control_loop([&](float dt){
        float phase_vel = 2 * M_PI * controller_.vel_setpoint_ * motor_.config_.pole_pairs;
        phase = wrap_pm_pi(phase + phase_vel * dt);
        if (!motor_.FOC_update(0.0f, controller_.current_setpoint_, phase, phase_vel))
            return false; // set_error should update axis.error_
        return true;
    });
    set_step_dir_active(false);
    return check_for_errors();
}


/**
 * @brief Spins the magnetic field at a fixed velocity (defined by the velocity
 * setpoint) and current/voltage setpoint. The current controller still runs in
 * closed loop mode.
 */
bool Axis::run_async_control_loop() {
    if (!motor_.is_calibrated_ || !motor_.config_.async_calibrated)
        return error_ |= ERROR_MOTOR_FAILED, false;

    float sigma = 1 - (motor_.config_.mutual_inductance * motor_.config_.mutual_inductance) / (motor_.config_.rotor_inductance * motor_.config_.phase_inductance);
    float l_s_sqr = motor_.config_.phase_inductance * motor_.config_.phase_inductance;
    float sigma_sqr = sigma * sigma;
    float sigma_l_s_sqr = sigma_sqr * l_s_sqr;
    float fw2_factor = sqrtf((1 + sigma_sqr) / (2 * sigma_l_s_sqr));

    if (!motor_.arm_foc())
        return error_ |= ERROR_MOTOR_FAILED, false;

    set_step_dir_active(config_.enable_step_dir);

    //float phase = 0.0f;
    run_control_loop([&](float dt){
        if (encoder_.error_ != Encoder::ERROR_NONE)
            return error_ |= ERROR_ENCODER_FAILED, false;
        if (async_estimator_.error_ != AsyncEstimator::ERROR_NONE)
            return error_ |= ERROR_ENCODER_FAILED, false;
#if 0
        controller_.vel_setpoint_ = encoder_.vel_estimate_ / encoder_.config_.cpr + controller_.config_.optimal_slip;
        float phase_vel = 2 * M_PI * controller_.vel_setpoint_ * motor_.config_.pole_pairs;
        phase = wrap_pm_pi(phase + phase_vel * dt);
        if (!motor_.FOC_update(0.0f, controller_.current_setpoint_, phase, phase_vel))
            return false; // set_error should update axis.error_
#endif
        float phase = async_estimator_.phase_;
        float phase_vel = async_estimator_.phase_vel_;

        float max_voltage = motor_.vbus_voltage_ * V_DC_TO_V_PHASE_MAX;
        float max_current = controller_.current_setpoint_;
        float max_current_sqr = max_current * max_current;

        // assumptions:
        //  - stator resistance is negligible
        //  - change of current amplitude is slow
        // source:
        //  http://shodhganga.inflibnet.ac.in/bitstream/10603/93562/12/12_chapter6.pdf
        //  (note: paper has some typos, also in some equations)

        float Id_setpoint;
        float Iq_setpoint;

        // Eq 6.18 in [1]
        if (!(phase_vel > max_voltage / abs(max_current) * fw2_factor)) {
            // Field Weakening region I (aka constant power region)
            // Eq 6.16 and 6.17 in [1]
            float v_max_over_omega = max_voltage / phase_vel;
            float Id_sqr = (v_max_over_omega * v_max_over_omega - sigma_l_s_sqr * max_current_sqr) / (l_s_sqr - sigma_l_s_sqr);
            float Iq_sqr = max_current_sqr - Id_sqr;

            if (!(Id_sqr < Iq_sqr)) {
                // Constant torque region (no field weakening)
                // To achieve maximum torque-per-amps the rotor flux angle
                // should be 45° to the rotor flux angle
                Id_sqr = Iq_sqr = max_current_sqr / 2;
                motor_.field_weakening_status_ = 0;
            } else {
                motor_.field_weakening_status_ = 1;
            }
            Id_setpoint = sqrtf(Id_sqr);
            Iq_setpoint = sqrtf(Iq_sqr);

        } else {
            // Field Weakening region II (aka constant slip region)
            // Eq 6.19 and 6.20 in [1]
            Id_setpoint = max_voltage * ((float)M_SQRT1_2) / (phase_vel * motor_.config_.phase_inductance);
            Iq_setpoint = Id_setpoint / sigma;
            motor_.field_weakening_status_ = 2;
        }

        if (max_current < 0) {
            Iq_setpoint = -Iq_setpoint;
        }

        if (!motor_.FOC_update(Id_setpoint, Iq_setpoint, phase, phase_vel))
            return false; // set_error should update axis.error_
        return true;
    });
    set_step_dir_active(false);
    return check_for_errors();
}

bool Axis::run_phase_locked_control() {
    Axis other_axis = (this == &axes[0]) ? axes[1] : axes[0];
    if (!motor_.arm_foc())
        return error_ |= ERROR_MOTOR_FAILED, false;

    run_control_loop([&](float dt){
        float phase = 0.0f;
        float phase_vel = 0.0f;
        if (other_axis.current_state_ != AXIS_STATE_IDLE) {
            phase = other_axis.motor_.current_control_.phase; // TODO: add an offset here to account for delayed PWM
            phase_vel = other_axis.motor_.current_control_.phase_vel;
        }
        if (!motor_.FOC_update(0.0f, controller_.current_setpoint_, phase, phase_vel))
            return false; // set_error should update axis.error_
        return true;
    });
    
    return check_for_errors();
}

bool Axis::run_idle_loop() {
    // run_control_loop ignores missed modulation timing updates
    // if and only if we're in AXIS_STATE_IDLE
    motor_.disarm();
    // the only valid reason to leave idle is an external request
    while (requested_state_ == AXIS_STATE_UNDEFINED) {
        //motor_.disarm();
        run_control_loop([this](float dt) {
            (void) dt;
            return true;
        });
    }
    return check_for_errors();
}

// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {
    while (!thread_id_valid_) {
        // Wait until the main task has signalled the readiness of this task,
        // otherwise the current measurement updates won't signal this thread.
        osDelay(1);
    }

    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    // TODO: Move this somewhere else
    // TODO: respect changes of CPR
    int encoder_cpr = encoder_.config_.cpr;
    controller_.anticogging_.cogging_map = (float*)malloc(encoder_cpr * sizeof(float));
    if (controller_.anticogging_.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            controller_.anticogging_.cogging_map[i] = 0.0f;
        }
    }

    //// arm!
    //motor_.arm();
    
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
            case AXIS_STATE_PWM_TEST: {
                status = motor_.pwm_test(1.0f);
            } break;

            case AXIS_STATE_MOTOR_CALIBRATION: {
                status = motor_.run_calibration();
            } break;

            case AXIS_STATE_ENCODER_INDEX_SEARCH: {
                if (encoder_.config_.idx_search_unidirectional && motor_.config_.direction==0)
                    goto invalid_state_label;

                status = encoder_.run_index_search();
            } break;

            case AXIS_STATE_ENCODER_DIR_FIND: {
                status = encoder_.run_direction_find();
            } break;

            case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
                status = encoder_.run_offset_calibration();
            } break;

            case AXIS_STATE_LOCKIN_SPIN: {
                if (motor_.config_.direction==0)
                    goto invalid_state_label;
                status = run_lockin_spin();
            } break;

            case AXIS_STATE_SENSORLESS_CONTROL: {
                if (motor_.config_.direction==0)
                        goto invalid_state_label;
                status = run_lockin_spin(); // TODO: restart if desired
                if (status) {
                    // call to controller.reset() that happend when arming means that vel_setpoint
                    // is zeroed. So we make the setpoint the spinup target for smooth transition.
                    controller_.vel_setpoint_ = config_.lockin.vel;
                    status = run_sensorless_control_loop();
                }
            } break;

            case AXIS_STATE_CLOSED_LOOP_CONTROL: {
                if (motor_.config_.direction==0)
                    goto invalid_state_label;
                if (!encoder_.is_ready_)
                    goto invalid_state_label;
                status = run_closed_loop_control_loop();
            } break;

            case AXIS_STATE_OPEN_LOOP_CONTROL: {
                if (motor_.config_.direction==0)
                    goto invalid_state_label;
                status = run_open_loop_control_loop();
            } break;

            case AXIS_STATE_ASYNC_CONTROL: {
                if (motor_.config_.direction==0)
                    goto invalid_state_label;
                status = run_open_loop_control_loop();
            } break;

            case AXIS_STATE_PHASE_LOCKED_CONTROL: {
                if (motor_.config_.direction==0)
                    goto invalid_state_label;
                status = run_phase_locked_control();
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
        if (!status)
            current_state_ = AXIS_STATE_IDLE;
        else
            memmove(task_chain_, task_chain_ + 1, sizeof(task_chain_) - sizeof(task_chain_[0]));
    }
}
