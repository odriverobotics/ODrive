#include <odrive_main.h>
#include <algorithm>

Endstop::Endstop(EndstopConfig_t &config)
    : config_(config) {
}

static void endstop_cb_wrapper(void* ctx){
    reinterpret_cast<Endstop*>(ctx)->endstop_cb();
}

void Endstop::update() {
    if (config_.enabled) {
        float now = axis_->loop_counter_ * current_meas_period;
        if ((now - debounce_timer_) >= (config_.debounce_ms * 0.001)) {          // Debounce timer expired, take the new pin state
            endstop_state_ = config_.is_active_high ? pin_state_ : !pin_state_;  // endstop_state is the logical state
            debounce_timer_ = now - (config_.debounce_ms * 0.001);               // Ensure timer doesn't have overflow issues
        } else {
            endstop_state_ = endstop_state_;  // Do nothing
        }
    } else {
        endstop_state_ = false;
    }
}

bool Endstop::getEndstopState() {
    return endstop_state_;
}

void Endstop::endstop_cb() {
    uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
    GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);

    debounce_timer_ = axis_->loop_counter_ * current_meas_period;
    pin_state_ = HAL_GPIO_ReadPin(gpio_port, gpio_pin);
}

void Endstop::set_endstop_enabled(bool enable){
    uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
    GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
    if(enable){
        HAL_GPIO_DeInit(gpio_port, gpio_pin);
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

        uint32_t pull_up_down = config_.is_active_high ? GPIO_PULLDOWN : GPIO_PULLUP;
        uint32_t interrupt_mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_subscribe(gpio_port, gpio_pin, pull_up_down, interrupt_mode,
                        endstop_cb_wrapper, this);
    }
    else {
        GPIO_unsubscribe(gpio_port, gpio_pin);
    }
}