
#include "axis.hpp"

//TODO: goal of refactor is to kick this out completely
extern "C" {
#include "low_level.h"
}

// C interface
extern "C" {
void axis_thread_entry(void const* temp_motor_ptr) {
    Motor_t* motor = (Motor_t*)temp_motor_ptr;

    //TODO: explicit axis number assignment
    //for now we search for it
    uint8_t ax_number = 0;
    while(&motors[ax_number] != motor)
        ++ax_number;

    static const AxisConfig default_config;

    Axis axis(default_config, ax_number, motor);
    axis.StateMachineLoop();
}
}  // extern "C"

Axis::Axis(const AxisConfig& config, uint8_t axis_number, Motor_t* legacy_motor_ref)
    : axis_number_(axis_number),
      enable_control_(config.enable_control_at_start),
      do_calibration_(config.do_calibration_at_start),
      legacy_motor_ref_(legacy_motor_ref) {}

void Axis::StateMachineLoop() {
    // override for compatibility with legacy comms paradigm
    // TODO next gen comms
    exposed_bools[4*axis_number_ + 1] = &enable_control_;
    exposed_bools[4*axis_number_ + 2] = &do_calibration_;

    // for (;;) {
        
    // }

    motor_thread(legacy_motor_ref_);
}