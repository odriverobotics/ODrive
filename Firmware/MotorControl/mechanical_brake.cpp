#include <odrive_main.h>

MechanicalBrake::MechanicalBrake(MechanicalBrake::Config_t& config)
    : config_(config) {
    update_config();
}

bool MechanicalBrake::get_state() {
    return mechanical_brake_state_;
}

void MechanicalBrake::update_config() {
    set_enabled(config_.enabled);
}

void MechanicalBrake::engage() {
    uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
    GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
    HAL_GPIO_WritePin(gpio_port, gpio_pin, config_.is_active_low ? GPIO_PIN_RESET : GPIO_PIN_SET );
}

void MechanicalBrake::release() {
    uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
    GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
    HAL_GPIO_WritePin(gpio_port, gpio_pin, config_.is_active_low ? GPIO_PIN_SET : GPIO_PIN_RESET );
}

void MechanicalBrake::set_enabled(bool enable) {
    if (config_.gpio_num != 0) {
        uint16_t gpio_pin = get_gpio_pin_by_pin(config_.gpio_num);
        GPIO_TypeDef* gpio_port = get_gpio_port_by_pin(config_.gpio_num);
        if (enable) {
            HAL_GPIO_DeInit(gpio_port, gpio_pin);
            GPIO_InitTypeDef GPIO_InitStruct;
            GPIO_InitStruct.Pin = gpio_pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = config_.pulldown ? GPIO_PULLDOWN : GPIO_PULLUP;
            HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);
        }
    }
}