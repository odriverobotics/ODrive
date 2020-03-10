#include <odrive_main.h>

Endstop::Endstop(Endstop::Config_t& config)
    : config_(config) {
    update_config();
}

void Endstop::update() {
    uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
    GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
    bool last_pin_state = pin_state_;
    pin_state_ = HAL_GPIO_ReadPin(gpio_port, gpio_pin);
    uint32_t now = static_cast<uint32_t>(axis_->loop_counter_ * current_meas_period * 1000);
    if (pin_state_ != last_pin_state) {
        debounce_timer_ = now;
    }
    if (config_.enabled) {
        if ((now - debounce_timer_) >= config_.debounce_ms) {         // Debounce timer expired, take the new pin state
            endstop_state_ = config_.is_active_high ? pin_state_ : !pin_state_;  // endstop_state is the logical state
            debounce_timer_ = now - config_.debounce_ms;              // Ensure timer doesn't have overflow issues
        } else {
            endstop_state_ = endstop_state_;  // Do nothing
        }
    } else {
        endstop_state_ = false;
    }
}

bool Endstop::get_state() {
    return endstop_state_;
}

void Endstop::update_config(){
    set_enabled(config_.enabled);
}

void Endstop::set_enabled(bool enable) {
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
        }
    }
}