#ifndef STM32_GPIO_HPP__
#define STM32_GPIO_HPP__

#include <gpio.h>

class Stm32Gpio {
public:
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {}

    operator bool() const { return port_; }

    void write(bool state) {
        HAL_GPIO_WritePin(port_, pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    bool read() {
        return HAL_GPIO_ReadPin(port_, pin_) != GPIO_PIN_RESET;
    }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

#endif // STM32_GPIO_HPP__
