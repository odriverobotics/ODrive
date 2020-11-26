/* MIT License

Copyright (c) 2016-2021 ODrive Robotics, Inc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */


#include "open_loop_controller.hpp"
#include <board.h>

void OpenLoopController::update(uint32_t timestamp) {
    auto [prev_Id, prev_Iq] = Idq_setpoint_.previous().value_or(float2D{0.0f, 0.0f});
    auto [prev_Vd, prev_Vq] = Vdq_setpoint_.previous().value_or(float2D{0.0f, 0.0f});
    float phase = phase_.previous().value_or(initial_phase_);
    float phase_vel = phase_vel_.previous().value_or(0.0f);

    (void)prev_Iq; // unused
    (void)prev_Vq; // unused

    float dt = (float)(timestamp - timestamp_) / (float)TIM_1_8_CLOCK_HZ;
    
    Idq_setpoint_ = {
        std::clamp(target_current_, prev_Id - max_current_ramp_ * dt, prev_Id + max_current_ramp_ * dt),
        0.0f
    };
    Vdq_setpoint_ = {
        std::clamp(target_voltage_, prev_Vd - max_voltage_ramp_ * dt, prev_Vd + max_voltage_ramp_ * dt),
        0.0f
    };
    
    phase_vel = std::clamp(target_vel_, phase_vel - max_phase_vel_ramp_ * dt, phase_vel + max_phase_vel_ramp_ * dt);
    phase_vel_ = phase_vel;
    phase_ = wrap_pm_pi(phase + phase_vel * dt);
    total_distance_ = total_distance_.previous().value_or(0.0f) + phase_vel * dt;
    timestamp_ = timestamp;
}
