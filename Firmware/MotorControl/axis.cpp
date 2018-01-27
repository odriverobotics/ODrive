
#include "axis.h"
#include <stdlib.h>
#include "legacy_commands.h"

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
    while (&motors[ax_number] != motor)
        ++ax_number;

    static const AxisConfig default_config;

    Axis axis(default_config, ax_number, motor);
    axis.StateMachineLoop();
}
}  // extern "C"

void Axis::SetupLegacyMappings() {
    // Legacy reachability from C
    legacy_motor_ref_->axis_legacy.enable_control = &enable_control_;

    // override for compatibility with legacy comms paradigm
    // TODO next gen comms
    exposed_bools[4 * axis_number_ + 1] = &enable_control_;
    exposed_bools[4 * axis_number_ + 2] = &do_calibration_;
}

Axis::Axis(const AxisConfig& config, uint8_t axis_number, Motor_t* legacy_motor_ref)
    : axis_number_(axis_number),
      enable_control_(config.enable_control_at_start),
      do_calibration_(config.do_calibration_at_start),
      legacy_motor_ref_(legacy_motor_ref) {
    SetupLegacyMappings();
}

void Axis::StateMachineLoop() {

    //TODO: Move this somewhere else
    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    int encoder_cpr = legacy_motor_ref_->encoder.encoder_cpr;
    legacy_motor_ref_->anticogging.cogging_map = (float*)malloc(encoder_cpr * sizeof(float));
    if (legacy_motor_ref_->anticogging.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            legacy_motor_ref_->anticogging.cogging_map[i] = 0.0f;
        }
    }

    legacy_motor_ref_->motor_thread = osThreadGetId();
    legacy_motor_ref_->thread_ready = true;
    bool calibration_ok = false;
    for (;;) {
        // Keep rotor estimation up to date while idling
        osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
        update_rotor(legacy_motor_ref_);
        
        if (do_calibration_) {
            do_calibration_ = false;

            __HAL_TIM_MOE_ENABLE(legacy_motor_ref_->motor_timer);  // enable pwm outputs
            calibration_ok = motor_calibration(legacy_motor_ref_);
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(legacy_motor_ref_->motor_timer);  // disables pwm outputs
        }

        if (calibration_ok && enable_control_) {
            legacy_motor_ref_->enable_step_dir = true;
            __HAL_TIM_MOE_ENABLE(legacy_motor_ref_->motor_timer);

            bool spin_up_ok = true;
            if (legacy_motor_ref_->rotor_mode == ROTOR_MODE_SENSORLESS)
                spin_up_ok = spin_up_sensorless(legacy_motor_ref_);
            if (spin_up_ok)
                control_motor_loop(legacy_motor_ref_);

            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(legacy_motor_ref_->motor_timer);
            legacy_motor_ref_->enable_step_dir = false;

            if (enable_control_) {  // if control is still enabled, we exited because of error
                calibration_ok = false;
                enable_control_ = false;
            }
        }
    }
    legacy_motor_ref_->thread_ready = false;
}