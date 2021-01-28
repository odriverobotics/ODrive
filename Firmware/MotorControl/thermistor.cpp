#include "odrive_main.h"

ThermistorCurrentLimiter::ThermistorCurrentLimiter(const float& temp_limit_lower,
                                                   const float& temp_limit_upper,
                                                   const bool& enabled) :
    temp_limit_lower_(temp_limit_lower),
    temp_limit_upper_(temp_limit_upper),
    enabled_(enabled)
{
}

bool ThermistorCurrentLimiter::do_checks() {
    if (enabled_ && get_temp() >= temp_limit_upper_ + 5) {
        return false;
    }
    return true;
}

float ThermistorCurrentLimiter::get_current_limit(float base_current_lim) const {
    if (!enabled_) {
        return base_current_lim;
    }

    const float temp_margin = temp_limit_upper_ - get_temp();
    const float derating_range = temp_limit_upper_ - temp_limit_lower_;
    float thermal_current_lim = base_current_lim * (temp_margin / derating_range);
    if (thermal_current_lim < 0.0f || is_nan(thermal_current_lim)) {
        thermal_current_lim = 0.0f;
    }

    return std::min(thermal_current_lim, base_current_lim);
}

OnboardThermistorCurrentLimiter::OnboardThermistorCurrentLimiter(float* value_ptr) :
    ThermistorCurrentLimiter(config_.temp_limit_lower,
                             config_.temp_limit_upper,
                             config_.enabled),
    value_ptr_(value_ptr)
{
}

OffboardThermistorCurrentLimiter::OffboardThermistorCurrentLimiter() :
    ThermistorCurrentLimiter(config_.temp_limit_lower,
                             config_.temp_limit_upper,
                             config_.enabled),
    coefficients_(&config_.thermistor_poly_coeffs[0])
{
}

void OffboardThermistorCurrentLimiter::update() {
    const uint16_t gpio = config_.gpio_pin;
    const float ratio =  (gpio < board.gpio_adc_values.size()) ? board.gpio_adc_values[gpio] : -INFINITY;
    temperature_ = horner_poly_eval(ratio, coefficients_, num_coeffs_);
}
