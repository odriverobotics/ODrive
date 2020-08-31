#ifndef __OPEN_LOOP_CONTROLLER_HPP
#define __OPEN_LOOP_CONTROLLER_HPP

#include "component.hpp"
#include <cmath>

class OpenLoopController : public ComponentBase {
public:
    void update(uint32_t timestamp) final;

    // Config
    float max_current_ramp_ = INFINITY; // [A/s]
    float max_voltage_ramp_ = INFINITY; // [V/s]
    float max_phase_vel_ramp_ = INFINITY; // [rad/s^2]

    // Inputs
    float target_vel_ = NAN;
    float target_current_ = NAN;
    float target_voltage_ = NAN;

    // State/Outputs
    uint32_t timestamp_ = 0;
    float Id_setpoint_ = NAN;
    float Iq_setpoint_ = NAN;
    float Vd_setpoint_ = NAN;
    float Vq_setpoint_ = NAN;
    float phase_ = NAN;
    float phase_vel_ = NAN;
    float total_distance_ = NAN;
};

#endif // __OPEN_LOOP_CONTROLLER_HPP