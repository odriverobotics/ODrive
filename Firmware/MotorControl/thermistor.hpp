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

#ifndef __THERMISTOR_HPP
#define __THERMISTOR_HPP

class Motor; // declared in motor.hpp

#include "current_limiter.hpp"
#include <autogen/interfaces.hpp>

class ThermistorCurrentLimiter : public CurrentLimiter, public ODriveIntf::ThermistorCurrentLimiterIntf {
public:
    virtual ~ThermistorCurrentLimiter() = default;

    ThermistorCurrentLimiter(uint16_t adc_channel,
                             const float* const coefficients,
                             size_t num_coeffs,
                             const float& temp_limit_lower,
                             const float& temp_limit_upper,
                             const bool& enabled);

    void update();
    bool do_checks();
    float get_current_limit(float base_current_lim) const override;

    uint16_t adc_channel_;
    const float* const coefficients_;
    const size_t num_coeffs_;
    float temperature_ = NAN; // [°C] NaN while the ODrive is initializing.
    const float& temp_limit_lower_;
    const float& temp_limit_upper_;
    const bool& enabled_;
    Motor* motor_ = nullptr; // set by Motor::apply_config()
};

class OnboardThermistorCurrentLimiter : public ThermistorCurrentLimiter, public ODriveIntf::OnboardThermistorCurrentLimiterIntf {
public:
    struct Config_t {
        float temp_limit_lower = 100;
        float temp_limit_upper = 120;
        bool enabled = true;
    };

    virtual ~OnboardThermistorCurrentLimiter() = default;
    OnboardThermistorCurrentLimiter(uint16_t adc_channel, const float* const coefficients, size_t num_coeffs);

    Config_t config_;
};

class OffboardThermistorCurrentLimiter : public ThermistorCurrentLimiter, public ODriveIntf::OffboardThermistorCurrentLimiterIntf {
public:
    static const size_t num_coeffs_ = 4;

    struct Config_t {
        float thermistor_poly_coeffs[num_coeffs_];

        uint16_t gpio_pin = 4;
        float temp_limit_lower = 100;
        float temp_limit_upper = 120;
        bool enabled = false;

        // custom setters
        OffboardThermistorCurrentLimiter* parent;
        void set_gpio_pin(uint16_t value) { gpio_pin = value; parent->decode_pin(); }
    };

    virtual ~OffboardThermistorCurrentLimiter() = default;
    OffboardThermistorCurrentLimiter();

    Config_t config_;

    bool apply_config();

private:
    void decode_pin();
};

#endif // __THERMISTOR_HPP
