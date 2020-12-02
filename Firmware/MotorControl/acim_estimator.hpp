#ifndef __ACIM_ESTIMATOR_HPP
#define __ACIM_ESTIMATOR_HPP

#include <component.hpp>
#include <cmath>
#include <autogen/interfaces.hpp>

class AcimEstimator : public ComponentBase {
public:
    struct Config_t {
        float slip_velocity = 14.706f; // [rad/s electrical] = 1/rotor_tau
    };

    void update(uint32_t timestamp) final;

    // Config
    Config_t config_;

    // Inputs
    InputPort<float> rotor_phase_src_;
    InputPort<float> rotor_phase_vel_src_;
    InputPort<float2D> idq_src_;

    // State variables
    float active_ = false;
    uint32_t last_timestamp_ = 0;
    float rotor_flux_ = 0.0f; // [A]
    float phase_offset_ = 0.0f; // [A]

    // Outputs
    OutputPort<float> slip_vel_ = 0.0f; // [rad/s electrical]
    OutputPort<float> stator_phase_vel_ = 0.0f; // [rad/s] rotor flux angular velocity estimate
    OutputPort<float> stator_phase_ = 0.0f; // [rad] rotor flux phase angle estimate
};

#endif // __ACIM_ESTIMATOR_HPP