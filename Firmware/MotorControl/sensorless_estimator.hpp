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

#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

#include "component.hpp"

class SensorlessEstimator : public ODriveIntf::SensorlessEstimatorIntf {
public:
    struct Config_t {
        float observer_gain = 1000.0f; // [rad/s]
        float pll_bandwidth = 1000.0f;  // [rad/s]
        float pm_flux_linkage = 1.58e-3f; // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
    };

    void reset();
    bool update();

    Axis* axis_ = nullptr; // set by Axis constructor
    Config_t config_;

    // TODO: expose on protocol
    Error error_ = ERROR_NONE;
    float pll_pos_ = 0.0f;                      // [rad]
    float flux_state_[2] = {0.0f, 0.0f};        // [Vs]
    float V_alpha_beta_memory_[2] = {0.0f, 0.0f}; // [V]

    OutputPort<float> phase_ = 0.0f;                   // [rad]
    OutputPort<float> phase_vel_ = 0.0f;               // [rad/s]
    OutputPort<float> vel_estimate_ = 0.0f;            // [turns/s]
};

#endif /* __SENSORLESS_ESTIMATOR_HPP */
