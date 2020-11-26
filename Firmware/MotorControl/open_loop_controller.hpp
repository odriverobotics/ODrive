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

#ifndef __OPEN_LOOP_CONTROLLER_HPP
#define __OPEN_LOOP_CONTROLLER_HPP

#include "component.hpp"
#include <cmath>
#include <autogen/interfaces.hpp>

class OpenLoopController : public ComponentBase {
public:
    void update(uint32_t timestamp) final;

    // Config
    float max_current_ramp_ = INFINITY; // [A/s]
    float max_voltage_ramp_ = INFINITY; // [V/s]
    float max_phase_vel_ramp_ = INFINITY; // [rad/s^2]

    // Inputs
    float target_vel_ = 0.0f;
    float target_current_ = 0.0f;
    float target_voltage_ = 0.0f;
    float initial_phase_ = 0.0f;

    // State/Outputs
    uint32_t timestamp_ = 0;
    OutputPort<float2D> Idq_setpoint_ = {{0.0f, 0.0f}};
    OutputPort<float2D> Vdq_setpoint_ = {{0.0f, 0.0f}};
    OutputPort<float> phase_ = 0.0f;
    OutputPort<float> phase_vel_ = 0.0f;
    OutputPort<float> total_distance_ = 0.0f;
};

#endif // __OPEN_LOOP_CONTROLLER_HPP