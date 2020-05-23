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

        uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
        GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
        pin_state_ = HAL_GPIO_ReadPin(gpio_port, gpio_pin);

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
        uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
        GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
        if (enable) {
            HAL_GPIO_DeInit(gpio_port, gpio_pin);
            GPIO_InitTypeDef GPIO_InitStruct;
            GPIO_InitStruct.Pin = gpio_pin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = config_.pullup ? GPIO_PULLUP : GPIO_PULLDOWN;
            HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);
            debounceTimer_.start();
        } else
            debounceTimer_.stop();
    }
}