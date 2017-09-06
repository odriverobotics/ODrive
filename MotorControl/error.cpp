#include "error.h"
#include "motor.h"

void Error::global_fault(int error) {
    // Disable motors NOW!
    for (int i = 0; i < Motor::getNumMotors(); ++i) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(Motor::getMotorByID(i)->motor_timer);
    }
    // Set fault codes, etc.
    for (int i = 0; i < Motor::getNumMotors(); ++i) {
        Motor::getMotorByID(i)->error = error;
        Motor::getMotorByID(i)->enable_control = false;
        Motor::getMotorByID(i)->calibration_ok = false;
    }
    // disable brake resistor
    Motor::update_brake_current(0.0f);
}