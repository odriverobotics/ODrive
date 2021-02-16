#ifndef __THERMISTOR_HPP
#define __THERMISTOR_HPP

class Motor; // declared in motor.hpp

#include "current_limiter.hpp"
#include <autogen/interfaces.hpp>

class ThermistorCurrentLimiter : public CurrentLimiter, public ODriveIntf::ThermistorCurrentLimiterIntf {
public:
    virtual ~ThermistorCurrentLimiter() = default;

    ThermistorCurrentLimiter(const float& temp_limit_lower,
                             const float& temp_limit_upper,
                             const bool& enabled);

    bool do_checks();
    float get_current_limit(float base_current_lim) const override;
    void update(); // fetch value, and run low pass filter
    float get_temp() const { return lpf_vals_.back(); };
    virtual float get_raw_temp() const = 0;

    const float& temp_limit_lower_;
    const float& temp_limit_upper_;
    const bool& enabled_;
    Motor* motor_ = nullptr; // set by Motor::apply_config()
    std::array<float, 2> lpf_vals_ = { 0.0f };
};

class OnboardThermistorCurrentLimiter : public ThermistorCurrentLimiter, public ODriveIntf::OnboardThermistorCurrentLimiterIntf {
public:
    struct Config_t {
        float temp_limit_lower = 100;
        float temp_limit_upper = 120;
        bool enabled = true;
    };

    virtual ~OnboardThermistorCurrentLimiter() = default;
    OnboardThermistorCurrentLimiter(float* value_ptr);
    float get_raw_temp() const final { return *value_ptr_; }

    Config_t config_;
    float* value_ptr_;
};

class OffboardThermistorCurrentLimiter : public ThermistorCurrentLimiter, public ODriveIntf::OffboardThermistorCurrentLimiterIntf {
public:
    static const size_t num_coeffs_ = 4;

    struct Config_t {
        float thermistor_poly_coeffs[num_coeffs_];

#if HW_VERSION_MAJOR == 3
        uint16_t gpio_pin = 4;
#elif HW_VERSION_MAJOR == 4
        uint16_t gpio_pin = 2;
#endif
        float temp_limit_lower = 100;
        float temp_limit_upper = 120;
        bool enabled = false;
    };

    virtual ~OffboardThermistorCurrentLimiter() = default;
    OffboardThermistorCurrentLimiter();

    float get_raw_temp() const final;
    bool apply_config();

    Config_t config_;
    const float* const coefficients_;
};

#endif // __THERMISTOR_HPP
