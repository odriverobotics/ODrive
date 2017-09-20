
#include "axis.hpp"

//TODO: goal of refactor is to kick this out completely
extern "C" {
#include "low_level.h"
}

// C interface
extern "C" {
void axis_thread_entry(void const* temp_motor_ptr) {
    Motor_t* motor = (Motor_t*)temp_motor_ptr;
    Axis axis(motor);
    axis.StateMachineLoop();
}
}  // extern "C"

Axis::Axis(Motor_t* legacy_motor_ref)
    : legacy_motor_ref_(legacy_motor_ref) {}

void Axis::StateMachineLoop() {
    motor_thread(legacy_motor_ref_);
}