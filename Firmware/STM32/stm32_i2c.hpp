#ifndef __STM32_I2C_HPP
#define __STM32_I2C_HPP

#include <stm32_gpio.hpp>
#include <stm32_dma.hpp>

class STM32_I2C_t {
public:
    I2C_HandleTypeDef hi2c;

    const STM32_GPIO_t** scl_gpios;
    const STM32_GPIO_t** sda_gpios;
    uint8_t gpio_af;
    const STM32_DMAChannel_t* tx_dmas;
    const STM32_DMAChannel_t* rx_dmas;

    STM32_DMAStream_t* tx_dma_ = nullptr;
    STM32_DMAStream_t* rx_dma_ = nullptr;

    STM32_I2C_t(I2C_TypeDef* instance, const STM32_GPIO_t** scl_gpios, const STM32_GPIO_t** sda_gpios, uint8_t gpio_af,
            const STM32_DMAChannel_t* tx_dmas, const STM32_DMAChannel_t* rx_dmas) :
        hi2c{ .Instance = instance },
        scl_gpios(scl_gpios),
        sda_gpios(sda_gpios),
        gpio_af(gpio_af),
        tx_dmas(tx_dmas),
        rx_dmas(rx_dmas) {}

    bool setup_as_slave(uint32_t freq, uint8_t addr, STM32_GPIO_t* scl_gpio, STM32_GPIO_t* sda_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma);
    bool enable_interrupts(uint8_t priority);
};

extern STM32_I2C_t i2c1, i2c2, i2c3;

#endif // __STM32_I2C_HPP