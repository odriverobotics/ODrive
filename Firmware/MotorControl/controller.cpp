
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

// Slowly drive in the negative direction at homing_speed until the min endstop is pressed
// When pressed, set the linear count to the offset (default 0), and then

//TODO: This needs to be upgraded to use its own run_control_loop!
bool Controller::home_axis() {
    if (axis_->min_endstop_.config_.enabled) {
        config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
        input_pos_ = 0.0f;
        input_pos_updated();
        input_vel_ = -config_.homing_speed;
        input_current_ = 0.0f;
        axis_->homing_state_ = HOMING_STATE_HOMING;
    } else {
        return false;
    }
    return true;
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (config_.anticogging.calib_anticogging) {
        float pos_err = input_pos_ - pos_estimate;
        if (fabsf(pos_err) <= config_.anticogging.calib_pos_threshold &&
            fabsf(vel_estimate) < config_.anticogging.calib_vel_threshold) {
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
    return false;
}

void Controller::update_filter_gains() {
    input_filter_ki_ = 2.0f * config_.input_filter_bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

namespace {
float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float Iq) {
    float Imax = (vel_limit - vel_estimate) * vel_gain;
    float Imin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(Iq, Imin, Imax);
}
}  // namespace

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if config_.anticogging.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);
    float anticogging_pos = pos_estimate / axis_->encoder_.getCoggingRatio();

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
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            current_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
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
        if (config_.setpoints_in_cpr) {
            // TODO this breaks the semantics that estimates come in on the arguments.
            // It's probably better to call a get_estimate that will arbitrate (enc vs sensorless) instead.
            float cpr = (float)(axis_->encoder_.config_.cpr);
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, cpr);
            // Circular delta
            pos_err = pos_setpoint_ - axis_->encoder_.pos_cpr_;
            pos_err = wrap_pm(pos_err, 0.5f * cpr);
        } else {
            pos_err = pos_setpoint_ - pos_estimate;
        }
        vel_des += config_.pos_gain * pos_err;
                // V-shaped gain shedule based on position error
        float abs_pos_err = fabsf(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (vel_des > vel_lim) vel_des = vel_lim;
    if (vel_des < -vel_lim) vel_des = -vel_lim;

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.vel_limit_tolerance > 0.0f) {  // 0.0f to disable
        if (fabsf(vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // Velocity control
    float Iq = current_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_) {
        Iq += config_.anticogging.cogging_map[std::clamp(mod(static_cast<int>(anticogging_pos), axis_->encoder_.config_.cpr), 0, 3600)];
    }

    float v_err = vel_des - vel_estimate;
    if (config_.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
        Iq += (config_.vel_gain * gain_scheduling_multiplier) * v_err;
    }

    // Velocity integral action before limiting
    Iq += vel_integrator_current_;

    // Velocity limiting in current mode
    if (config_.control_mode < CTRL_MODE_VELOCITY_CONTROL && config_.vel_limit > 0.0f) {
        Iq = limitVel(config_.vel_limit, vel_estimate, config_.vel_gain, Iq);
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
