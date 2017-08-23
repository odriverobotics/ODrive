#pragma once

#include "motor.h"
#include "encoder.h"

//Note: to control without feed forward, set feed forward terms to 0.0f.
void set_pos_setpoint(Motor_t *motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward);
void set_vel_setpoint(Motor_t *motor, float vel_setpoint, float current_feed_forward);
void set_current_setpoint(Motor_t *motor, float current_setpoint);
