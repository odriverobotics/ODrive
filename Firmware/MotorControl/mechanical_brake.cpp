#include <odrive_main.h>

bool MechanicalBrake::apply_config() {
    set_enabled(config_.enabled);
    return true;
}

void MechanicalBrake::engage() {
    get_gpio(config_.gpio_num).write(config_.is_active_low ? 0 : 1);
}

void MechanicalBrake::release() {
    get_gpio(config_.gpio_num).write(config_.is_active_low ? 1 : 0);
}

void MechanicalBrake::set_enabled(bool enable) {
    if (config_.gpio_num != 0) {
        Stm32Gpio gpio = get_gpio(config_.gpio_num);
        if (enable) {
            gpio.config(GPIO_MODE_OUTPUT_PP, config_.pulldown ? GPIO_PULLDOWN : GPIO_PULLUP);
        }
    }
}