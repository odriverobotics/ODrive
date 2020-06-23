#include <odrive_main.h>

Endstop::Endstop(Endstop::Config_t& config)
    : config_(config) {
    update_config();
    debounceTimer_.setIncrement(current_meas_period);
}


void Endstop::update() {
    debounceTimer_.update();
    if (config_.enabled) {
        bool last_pin_state = pin_state_;

        pin_state_ = get_gpio(config_.gpio_num).read();

        // If the pin state has changed, reset the timer
        if (pin_state_ != last_pin_state)
            debounceTimer_.reset();

        if (debounceTimer_.expired())
            endstop_state_ = config_.is_active_high ? pin_state_ : !pin_state_;  // endstop_state is the logical state
    } else {
        endstop_state_ = false;
    }
}

bool Endstop::get_state() {
    return endstop_state_;
}

void Endstop::update_config() {
    set_enabled(config_.enabled);
    debounceTimer_.setIncrement(config_.debounce_ms * 0.001f);
}

void Endstop::set_enabled(bool enable) {
    debounceTimer_.reset();
    if (config_.gpio_num != 0) {
        if (enable) {
            Stm32Gpio gpio = get_gpio(config_.gpio_num);
            gpio.config(GPIO_MODE_INPUT, config_.pullup ? GPIO_PULLUP : GPIO_PULLDOWN);
            debounceTimer_.start();
        } else
            debounceTimer_.stop();
    }
}