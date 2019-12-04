
#include "odrive_main.h"

#include <algorithm>

Controller::Controller(Config_t& config) :
    config_(config)
{
    update_filter_gains();
}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

void Controller::set_error(Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::input_pos_updated() {
    input_pos_updated_ = true;
}

bool Controller::select_encoder(size_t encoder_num) {
    if (encoder_num < AXIS_COUNT) {
        Axis* ax = axes[encoder_num];
        if (config_.setpoints_in_cpr) {
            pos_estimate_src_ = &ax->encoder_.pos_cpr_;
            pos_wrap_src_ = &ax->encoder_.config_.cpr;
        } else {
            pos_estimate_src_ = &ax->encoder_.pos_estimate_;
            pos_wrap_src_ = nullptr;
        }
        pos_estimate_valid_src_ = &ax->encoder_.pos_estimate_valid_;
        vel_estimate_src_ = &ax->encoder_.vel_estimate_;
        vel_estimate_valid_src_ = &ax->encoder_.vel_estimate_valid_;
        return true;
    } else {
        return set_error(Controller::ERROR_INVALID_LOAD_ENCODER), false;
    }
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_.config_.vel_limit,
                                 axis_->trap_.config_.accel_limit,
                                 axis_->trap_.config_.decel_limit);
    traj_start_loop_count_ = axis_->loop_counter_;
    trajectory_done_ = false;
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.calib_anticogging = true;
    }
}


/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold) {
        config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 3600)] = vel_integrator_current_;
    }
    if (config_.anticogging.index < 3600) {
        config_.control_mode = CTRL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * axis_->encoder_.getCoggingRatio();
        input_vel_ = 0.0f;
        input_current_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        config_.anticogging.index = 0;
        config_.control_mode = CTRL_MODE_POSITION_CONTROL;
        input_pos_ = 0.0f;  // Send the motor home
        input_vel_ = 0.0f;
        input_current_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

void Controller::update_filter_gains() {
    input_filter_ki_ = 2.0f * config_.input_filter_bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float Iq) {
    float Imax = (vel_limit - vel_estimate) * vel_gain;
    float Imin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(Iq, Imin, Imax);
}

bool Controller::update(float* current_setpoint_output) {
    float* pos_estimate_src = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_src_ : nullptr;
    float* vel_estimate_src = (vel_estimate_valid_src_ && *vel_estimate_valid_src_)
            ? vel_estimate_src_ : nullptr;

    // Calib_anticogging is only true when calibration is occurring, so we can't block anticogging_pos
    float anticogging_pos = axis_->encoder_.pos_estimate_ / axis_->encoder_.getCoggingRatio();
    if (config_.anticogging.calib_anticogging) {
        if (!axis_->encoder_.pos_estimate_valid_ || !axis_->encoder_.vel_estimate_valid_) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(axis_->encoder_.pos_estimate_, axis_->encoder_.vel_estimate_);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (pos_wrap_src_) {
        float cpr = *pos_wrap_src_;
        // Keep pos setpoint from drifting
        input_pos_ = fmodf_pos(input_pos_, cpr);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            current_setpoint_ = input_current_;
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            current_setpoint_ = step / current_meas_period * config_.inertia;
        } break;
        case INPUT_MODE_CURRENT_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.current_ramp_rate);
            float full_step     = input_current_ - current_setpoint_;
            float step          = std::clamp(full_step, -max_step_size, max_step_size);

            current_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            current_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += std::clamp(current_meas_period * accel, 2.0f * std::abs(delta_vel), -2.0f * std::abs(delta_vel)); // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror >= 0 && config_.axis_to_mirror < AXIS_COUNT) {
                vel_setpoint_ = axes[config_.axis_to_mirror]->encoder_.vel_estimate_ * config_.mirror_ratio;
                if (config_.setpoints_in_cpr) {
                    pos_setpoint_ += vel_setpoint_ * current_meas_period;
                } else {
                    pos_setpoint_ = axes[config_.axis_to_mirror]->encoder_.pos_estimate_ * config_.mirror_ratio;
                }
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            // Note: uint32_t loop count delta is OK across overflow
            // Beware of negative deltas, as they will not be well behaved due to uint!
            float t = (axis_->loop_counter_ - traj_start_loop_count_) * current_meas_period;
            if (t > axis_->trap_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CTRL_MODE_POSITION_CONTROL;
                pos_setpoint_ = input_pos_;
                vel_setpoint_ = 0.0f;
                current_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_.eval(t);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                current_setpoint_ = traj_step.Ydd * config_.inertia;
            }
            anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CTRL_MODE_POSITION_CONTROL) {
        float pos_err;
        if (!pos_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        if (pos_wrap_src_) {
            float cpr = *pos_wrap_src_;
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, cpr);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_src;
            pos_err = wrap_pm(pos_err, 0.5f * cpr);
        } else {
            pos_err = pos_setpoint_ - *pos_estimate_src;
        }

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        if (vel_des > vel_lim) vel_des = vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate_src) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // Velocity control
    float Iq = current_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.enable) {
        Iq += config_.anticogging.cogging_map[std::clamp(mod(static_cast<int>(anticogging_pos), 3600), 0, 3600)];
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate_src;
        Iq += (config_.vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        Iq += vel_integrator_current_;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CTRL_MODE_VELOCITY_CONTROL && config_.enable_current_vel_limit) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        Iq = limitVel(config_.vel_limit, *vel_estimate_src, config_.vel_gain, Iq);
    }

    // Current limiting
    bool limited = false;
    float Ilim = axis_->motor_.effective_current_lim();
    if (Iq > Ilim) {
        limited = true;
        Iq = Ilim;
    }
    if (Iq < -Ilim) {
        limited = true;
        Iq = -Ilim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CTRL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_current_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_current_ *= 0.99f;
        } else {
            vel_integrator_current_ += ((config_.vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
    }

    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
