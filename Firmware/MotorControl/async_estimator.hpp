#ifndef __ASYNC_ESTIMATOR_HPP
#define __ASYNC_ESTIMATOR_HPP

#include <component.hpp>
#include <cmath>

class AsyncEstimator : public ComponentBase {
public:
    struct Config_t {
        float slip_velocity = 14.706f; // [rad/s electrical] = 1/rotor_tau
    };

    void update(uint32_t timestamp) final;

    // Config
    Config_t config_;

    // Inputs
    float* rotor_phase_src_ = nullptr;
    float* rotor_phase_vel_src_ = nullptr;
    float* id_src_ = nullptr;
    float* iq_src_ = nullptr;

    // State variables
    float active_ = false;
    uint32_t last_timestamp_ = 0;
    float rotor_flux_ = 0.0f; // [A]
    float slip_vel_ = 0.0f; // [rad/s electrical]
    float phase_offset_ = 0.0f; // [rad electrical]

    // Outputs
    float stator_phase_vel_ = NAN; // [rad/s] rotor flux angular velocity estimate
    float stator_phase_ = NAN; // [rad] rotor flux phase angle estimate
};

#endif // __ASYNC_ESTIMATOR_HPP