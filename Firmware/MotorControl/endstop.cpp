#include <odrive_main.h>

Endstop::Endstop(EndstopConfig_t &config)
    : config_(config) {
}

static void endstop_cb_wrapper(void* ctx){
    reinterpret_cast<Endstop*>(ctx)->endstop_cb();
}

void Endstop::endstop_cb(){
    uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
    GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);

    if(config_.enabled){
        endstop_state_ = HAL_GPIO_ReadPin(gpio_port, gpio_pin);
        if(config_.is_active_high == false)
            endstop_state_ = !endstop_state_;
    } else {
        endstop_state_ = false;
    }
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