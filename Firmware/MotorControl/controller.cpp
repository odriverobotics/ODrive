
#include "odrive_main.h"


Controller::Controller(ControllerConfig_t& config) :
    config_(config)
{}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", pos_setpoint, vel_setpoint_, current_setpoint_);
#endif
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", vel_setpoint_, motor->current_setpoint_);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    config_.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", current_setpoint_);
#endif
}

void Controller::move_to_pos(float goal_point) {
    planned_move_end_time_ = axis_->trap_.planTrapezoidal(goal_point, pos_setpoint_,
                                                          vel_setpoint_, axis_->trap_.config_.vel_limit,
                                                          axis_->trap_.config_.accel_limit, axis_->trap_.config_.decel_limit);
    
    goal_point_ = goal_point;
    config_.control_mode = CTRL_MODE_PLANNED_MOVE_CONTROL;
    TrapTrajStep_t myTraj = axis_->trap_.evalTrapTraj(0.0f);
    pos_setpoint_ = myTraj.Y;
    vel_setpoint_ = myTraj.Yd;
    current_setpoint_ = myTraj.Ydd * axis_->trap_.config_.cpss_to_A;

    planned_move_timer_ = axis_->loop_counter_ * current_meas_period;
}

void Controller::move_incremental(float displacement, bool from_goal_point = true){
    if(from_goal_point){
        move_to_pos(goal_point_ + displacement);
    } else{
        move_to_pos(axis_->encoder_.pos_estimate_ + displacement);
    }
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (anticogging_.cogging_map != NULL && axis_->error_ == Axis::ERROR_NONE) {
        anticogging_.calib_anticogging = true;
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
    if (anticogging_.calib_anticogging && anticogging_.cogging_map != NULL) {
        float pos_err = anticogging_.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging_.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging_.calib_vel_threshold) {
            anticogging_.cogging_map[anticogging_.index++] = vel_integrator_current_;
        }
        if (anticogging_.index < axis_->encoder_.config_.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging_.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging_.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            anticogging_.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging_.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if anticogging_.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);
    float anticogging_pos = pos_estimate;

    // Controlled Move
    if (config_.control_mode >= CTRL_MODE_PLANNED_MOVE_CONTROL) {
        float time_now = axis_->loop_counter_ * current_meas_period;
        if ((time_now - planned_move_timer_) > planned_move_end_time_) {
            config_.control_mode = CTRL_MODE_POSITION_CONTROL;
            vel_setpoint_ = 0.0f;
            current_setpoint_ = 0.0f;
        } else {
            TrapTrajStep_t myTraj = axis_->trap_.evalTrapTraj(time_now - planned_move_timer_);
            pos_setpoint_ = myTraj.Y;
            vel_setpoint_ = myTraj.Yd;
            current_setpoint_ = myTraj.Ydd * axis_->trap_.config_.cpss_to_A;
        }
        anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CTRL_MODE_POSITION_CONTROL) {
        float pos_err = pos_setpoint_ - pos_estimate;
        vel_des += config_.pos_gain * pos_err;
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (vel_des > vel_lim) vel_des = vel_lim;
    if (vel_des < -vel_lim) vel_des = -vel_lim;

    // Velocity control
    float Iq = current_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_.use_anticogging) {
        Iq += anticogging_.cogging_map[mod(static_cast<int>(anticogging_pos), axis_->encoder_.config_.cpr)];
    }

    float v_err = vel_des - vel_estimate;
    if (config_.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
        Iq += config_.vel_gain * v_err;
    }

    // Velocity integral action before limiting
    Iq += vel_integrator_current_;

    // Current limiting
    float Ilim = std::min(axis_->motor_.config_.current_lim, axis_->motor_.current_control_.max_allowed_current);
    bool limited = false;
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
            vel_integrator_current_ += (config_.vel_integrator_gain * current_meas_period) * v_err;
        }
    }

    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
