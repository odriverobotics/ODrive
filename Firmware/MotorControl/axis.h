#ifndef __AXIS_HPP
#define __AXIS_HPP

//TODO: goal of refactor is to kick this out completely
extern "C" {
#include "low_level.h"
}

//Outside axis:
    //command handler
    //callback dispatch

// TODO: decide if we want to consolidate all default configs in one file for ease of use?
struct AxisConfig {
    bool enable_control_at_start = true;
    bool do_calibration_at_start = true;
};
extern AxisConfig axis_configs[];

class Axis {
public:
    //thread/os/system management
        //timing log
        //thread id
        //etc.
    //state machine
        //control mode
        //control_en/calib_ok
        //error state
    //motor
        //current controller
            //contains rotor phase logic
        //motor level calibration routines
        //low_level (implementation specifics)
            //DRV driver
            //adc callback handling
            //pwm queueing
    //rotor estimator
        //kick out rotor phase logic
    //pos/vel controller
    //step/dir handler

    // Object operation requires ptr to legacy object for now, TODO: get rid of this dep
    Axis(AxisConfig& config, uint8_t axis_number, Motor_t* legacy_motor_ref);

    // Infinite loop that does calibration and enters main control loop as appropriate
    void StateMachineLoop();

    uint8_t axis_number_;

    bool enable_control_;
    bool do_calibration_;

    AxisConfig& config_;

    Motor_t* legacy_motor_ref_;

private:
    void SetupLegacyMappings();

};

#endif /* __AXIS_HPP */
