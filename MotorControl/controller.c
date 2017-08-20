#include <controller.h>

// Set a desired position, with Velocity and Current (Torque, Acceleration) Feed Forward terms
// For use with Position control mode.
void set_pos_setpoint(Motor_t *motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward)
{
    motor->pos_setpoint = pos_setpoint;
    motor->vel_setpoint = vel_feed_forward;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", motor->pos_setpoint, motor->vel_setpoint, motor->current_setpoint);
#endif
}

// Set a desired velocity, with Current (Torque, Acceleration) Feed Forward term
// For use with Velocity control mode
void set_vel_setpoint(Motor_t *motor, float vel_setpoint, float current_feed_forward)
{
    motor->vel_setpoint = vel_setpoint;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", motor->vel_setpoint, motor->current_setpoint);
#endif
}

// Sets a desired Current (Torque, Acceleration)
// For use with Current control mode
void set_current_setpoint(Motor_t *motor, float current_setpoint)
{
    motor->current_setpoint = current_setpoint;
    motor->control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", motor->current_setpoint);
#endif
}