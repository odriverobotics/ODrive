
#include "odrive_main.hpp"


Controller::Controller(ControllerConfig_t& config) :
    config(config)
{}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint = pos_setpoint;
    vel_setpoint = vel_feed_forward;
    current_setpoint = current_feed_forward;
    config.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", motor->pos_setpoint, motor->vel_setpoint, motor->current_setpoint);
#endif
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint = vel_setpoint;
    current_setpoint = current_feed_forward;
    config.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", motor->vel_setpoint, motor->current_setpoint);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint = current_setpoint;
    config.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", motor->current_setpoint);
#endif
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anti_cogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging.calib_anticogging && anticogging.cogging_map != NULL) {
        float pos_err = anticogging.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging.calib_vel_threshold) {
            anticogging.cogging_map[anticogging.index++] = vel_integrator_current;
        }
        if (anticogging.index < axis->encoder.config.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            anticogging.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if anticogging.calib_anticogging is true; non-blocking
    anti_cogging_calibration(pos_estimate, vel_estimate);
    
    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float vel_des = vel_setpoint;
    if (config.control_mode >= CTRL_MODE_POSITION_CONTROL) {
        float pos_err = pos_setpoint - pos_estimate;
        vel_des += config.pos_gain * pos_err;
    }

    // Velocity limiting
    float vel_lim = config.vel_limit;
    if (vel_des > vel_lim) vel_des = vel_lim;
    if (vel_des < -vel_lim) vel_des = -vel_lim;

    // Velocity control
    float Iq = current_setpoint;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging.use_anticogging) {
        Iq += anticogging.cogging_map[mod(pos_estimate, axis->encoder.config.cpr)];
    }

    float v_err = vel_des - vel_estimate;
    if (config.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
        Iq += config.vel_gain * v_err;
    }

    // Velocity integral action before limiting
    Iq += vel_integrator_current;

    // Current limiting
    float Ilim = std::min(axis->motor.config.current_lim, axis->motor.current_control.max_allowed_current);
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
    if (config.control_mode < CTRL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_current = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_current *= 0.99f;
        } else {
            vel_integrator_current += (config.vel_integrator_gain * current_meas_period) * v_err;
        }
    }

    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
