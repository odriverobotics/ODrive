
#include "async_estimator.hpp"
#include <board.h>

void AsyncEstimator::update(uint32_t timestamp)  {
    float rotor_phase = rotor_phase_src_ ? *rotor_phase_src_ : NAN;
    float rotor_phase_vel = rotor_phase_vel_src_ ? *rotor_phase_vel_src_ : NAN;
    float id = id_src_ ? *id_src_ : NAN;
    float iq = iq_src_ ? *iq_src_ : NAN;

    if (std::isnan(rotor_phase) || std::isnan(rotor_phase_vel)) {
        stator_phase_vel_ = NAN;
        stator_phase_ = NAN;
        active_ = false;
        return;
    }

    if (!active_) {
        last_timestamp_ = timestamp;
        stator_phase_vel_ = 0.0f;
        stator_phase_ = 0.0f;
        active_ = true;
        return;
    }

    last_timestamp_ = timestamp;

    float dt = (float)(timestamp - last_timestamp_) / (float)TIM_1_8_CLOCK_HZ;

    // Note that the effect of the current commands on the real currents is actually 1.5 PWM cycles later
    // However the rotor time constant is (usually) so slow that it doesn't matter
    // So we elect to write it as if the effect is immediate, to have cleaner code

    // acim_rotor_flux is normalized to units of [A] tracking Id; rotor inductance is unspecified
    float dflux_by_dt = config_.slip_velocity * (id - rotor_flux_);
    rotor_flux_ += dflux_by_dt * dt;
    float slip_velocity = config_.slip_velocity * (iq / rotor_flux_);
    // Check for issues with small denominator. Polarity of check to catch NaN too
    bool acceptable_vel = fabsf(slip_velocity) <= 0.1f / dt;
    if (!acceptable_vel)
        slip_velocity = 0.0f;
    slip_vel_ = slip_velocity; // reporting only
    stator_phase_vel_ = rotor_phase_vel + slip_velocity;

    phase_offset_ += slip_velocity * dt;
    phase_offset_ = wrap_pm_pi(phase_offset_);
    stator_phase_ = wrap_pm_pi(rotor_phase + phase_offset_);
}
