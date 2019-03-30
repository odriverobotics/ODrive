#ifndef __STM32_CAN_HPP
#define __STM32_CAN_HPP

#include <stm32_gpio.hpp>

class STM32_CAN_t {
public:
    CAN_HandleTypeDef hcan;

    const STM32_GPIO_t** rx_gpios;
    const STM32_GPIO_t** tx_gpios;
    uint8_t gpio_af;

    STM32_CAN_t(CAN_TypeDef* instance, const STM32_GPIO_t** rx_gpios, const STM32_GPIO_t** tx_gpios, uint8_t gpio_af) :
        hcan{ .Instance = instance },
        rx_gpios(rx_gpios),
        tx_gpios(tx_gpios),
        gpio_af(gpio_af) {}

    bool setup(STM32_GPIO_t* rx_gpio, STM32_GPIO_t* tx_gpio);
    bool enable_interrupts(uint8_t priority);
};

extern STM32_CAN_t can1, can2;

#endif // __STM32_CAN_HPP