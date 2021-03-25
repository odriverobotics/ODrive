#pragma once

#include <tuple>

struct Autotuning_t {
    float frequency = 0.0f;
    float pos_amplitude = 0.0f;
    float pos_phase = 0.0f;
    float vel_amplitude = 0.0f;
    float vel_phase = 0.0f;
    float torque_amplitude = 0.0f;
    float torque_phase = 0.0f;

    std::tuple<float, float, float> update() {
        phase = wrap_pm_pi(phase + (2.0f * M_PI * frequency * current_meas_period));
        float pos_setpoint = pos_amplitude * our_arm_sin_f32(phase + pos_phase);
        float vel_setpoint = vel_amplitude * our_arm_sin_f32(phase + vel_phase);
        float torque_setpoint = torque_amplitude * our_arm_sin_f32(phase + torque_phase);

        return {pos_setpoint, vel_setpoint, torque_setpoint};
    }

   private:
    float phase = 0.0f;
};
