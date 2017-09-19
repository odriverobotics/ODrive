#pragma once

#include "encoder.h"
#include "motor.h"

//Note: to control without feed forward, set feed forward terms to 0.0f.
class Controller {
   public:
    static void setPositionSetpoint(Motor &motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    static void setVelocitySetpoint(Motor &motor, float vel_setpoint, float current_feed_forward);
    static void setCurrentSetpoint(Motor &motor, float current_setpoint);
};