#pragma once

#include <tuple>

struct Autotuning_t {
    float frequency = 0.0f;
    float pos_amplitude = 0.0f;
    float vel_amplitude = 0.0f;
    float torque_amplitude = 0.0f;
    float pos_phase = 0.0f;
    float vel_phase = 0.0f;
    float torque_phase = 0.0f;

    // Update the autotuner according to the specified timestep
    // Returns [pos_setpoint, vel_setpoint, torque_setpoint] tuple
    std::tuple<float, float, float> update(float timestep) noexcept {
        phase = wrap_pm_pi(phase + (2.0f * M_PI * frequency * timestep));
        float pos_setpoint = pos_amplitude * our_arm_sin_f32(phase + pos_phase);
        float vel_setpoint = vel_amplitude * our_arm_sin_f32(phase + vel_phase);
        float torque_setpoint = torque_amplitude * our_arm_sin_f32(phase + torque_phase);

        return {pos_setpoint, vel_setpoint, torque_setpoint};
    }

    void reset() {
        phase = 0.0f;
    }

   private:
    float phase = 0.0f;
};
