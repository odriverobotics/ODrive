#include "odrive_main.h"

ThermistorCurrentLimiter::ThermistorCurrentLimiter(const float& temp_limit_lower,
                                                   const float& temp_limit_upper,
                                                   const bool& enabled) :
    temp_limit_lower_(temp_limit_lower),
    temp_limit_upper_(temp_limit_upper),
    enabled_(enabled)
{
}

void ThermistorCurrentLimiter::update() {
    constexpr float tau = 0.1f; // [sec]
    float k = current_meas_period / tau;
    float val = get_raw_temp();
    for (float& lpf_val : lpf_vals_) {
        lpf_val += k * (val - lpf_val);
        val = lpf_val;
    }
    if (is_nan(val)) {
        lpf_vals_.fill(0.0f);
    }
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

float OffboardThermistorCurrentLimiter::get_raw_temp() const {
    const uint16_t gpio = config_.gpio_pin;
    const float ratio =  (gpio < board.gpio_adc_values.size()) ? board.gpio_adc_values[gpio] : NAN;
    return is_nan(ratio) ? NAN : horner_poly_eval(ratio, coefficients_, num_coeffs_);
}
