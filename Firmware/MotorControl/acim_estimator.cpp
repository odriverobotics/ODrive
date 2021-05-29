
#include "acim_estimator.hpp"
#include <board.h>

void AcimEstimator::update(uint32_t timestamp)  {
    std::optional<float> rotor_phase = rotor_phase_src_.present();
    std::optional<float> rotor_phase_vel = rotor_phase_vel_src_.present();
    std::optional<float2D> idq = idq_src_.present();

    if (!rotor_phase.has_value() || !rotor_phase_vel.has_value() || !idq.has_value()) {
        active_ = false;
        return;
    }

    auto [id, iq] = *idq;

    float dt = (float)(timestamp - last_timestamp_) / (float)TIM_1_8_CLOCK_HZ;
    last_timestamp_ = timestamp;

    if (!active_) {
        // Skip first iteration and use it to reset state
        rotor_flux_ = 0.0f;
        phase_offset_ = 0.0f;
        active_ = true;
        return;
    }

    // Note that the effect of the current commands on the real currents is actually 1.5 PWM cycles later
    // However the rotor time constant is (usually) so slow that it doesn't matter
    // So we elect to write it as if the effect is immediate, to have cleaner code

    // acim_rotor_flux is normalized to units of [A] tracking Id; rotor inductance is unspecified
    float dflux_by_dt = config_.slip_velocity * (id - rotor_flux_);
    rotor_flux_ += dflux_by_dt * dt;
    float slip_velocity = config_.slip_velocity * (iq / rotor_flux_);
    // Check for issues with small denominator.
    if (is_nan(slip_velocity) || (std::abs(slip_velocity) > 0.1f / dt)) {
        slip_velocity = 0.0f;
    }
    slip_vel_ = slip_velocity; // reporting only

    stator_phase_vel_ = *rotor_phase_vel + slip_velocity;
    phase_offset_ = wrap_pm_pi(phase_offset_ + slip_velocity * dt);
    stator_phase_ = wrap_pm_pi(*rotor_phase + phase_offset_);
}
