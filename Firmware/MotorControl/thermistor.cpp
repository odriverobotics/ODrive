#include "odrive_main.h"

#include "low_level.h"

ThermistorCurrentLimiter::ThermistorCurrentLimiter(uint16_t adc_channel,
                                                   const float* const coefficients,
                                                   size_t num_coeffs,
                                                   const float& temp_limit_lower,
                                                   const float& temp_limit_upper,
                                                   const bool& enabled) :
    adc_channel_(adc_channel),
    coefficients_(coefficients),
    num_coeffs_(num_coeffs),
    temperature_(NAN),
    temp_limit_lower_(temp_limit_lower),
    temp_limit_upper_(temp_limit_upper),
    enabled_(enabled),
    error_(ERROR_NONE)
{
}

void ThermistorCurrentLimiter::update() {
    const float voltage = get_adc_voltage_channel(adc_channel_);
    const float normalized_voltage = voltage / adc_ref_voltage;
    temperature_ = horner_fma(normalized_voltage, coefficients_, num_coeffs_);
}

bool ThermistorCurrentLimiter::do_checks() {
    if (enabled_ && temperature_ >= temp_limit_upper_ + 5) {
        error_ = ERROR_OVER_TEMP;
        axis_->error_ |= Axis::ERROR_OVER_TEMP;
        return false;
    }
    return true;
}

float ThermistorCurrentLimiter::get_current_limit(float base_current_lim) const {
    if (!enabled_) {
        return base_current_lim;
    }

    const float temp_margin = temp_limit_upper_ - temperature_;
    const float derating_range = temp_limit_upper_ - temp_limit_lower_;
    float thermal_current_lim = base_current_lim * (temp_margin / derating_range);
    if (!(thermal_current_lim >= 0.0f)) { // Funny polarity to also catch NaN
        thermal_current_lim = 0.0f;
    }

    return std::min(thermal_current_lim, base_current_lim);
}

OnboardThermistorCurrentLimiter::OnboardThermistorCurrentLimiter(const ThermistorHardwareConfig_t& hw_config, Config_t& config) :
    ThermistorCurrentLimiter(hw_config.adc_ch,
                             hw_config.coeffs,
                             hw_config.num_coeffs,
                             config.temp_limit_lower,
                             config.temp_limit_upper,
                             config.enabled),
    config_(config)
{
}

OffboardThermistorCurrentLimiter::OffboardThermistorCurrentLimiter(Config_t& config) :
    ThermistorCurrentLimiter(UINT16_MAX,
                             &config.thermistor_poly_coeffs[0],
                             num_coeffs_,
                             config.temp_limit_lower,
                             config.temp_limit_upper,
                             config.enabled),
    config_(config)
{
    decode_pin();
}

void OffboardThermistorCurrentLimiter::decode_pin() {
    const GPIO_TypeDef* const port = get_gpio_port_by_pin(config_.gpio_pin);
    const uint16_t pin = get_gpio_pin_by_pin(config_.gpio_pin);

    adc_channel_ = channel_from_gpio(port, pin);
}
