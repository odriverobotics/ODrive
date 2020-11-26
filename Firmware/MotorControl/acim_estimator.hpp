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