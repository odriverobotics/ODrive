#ifndef __STM32_SPI_HPP
#define __STM32_SPI_HPP

#include <stm32_gpio.hpp>
#include <stm32_dma.hpp>

class STM32_SPI_t {
public:
    SPI_HandleTypeDef hspi;

    const STM32_GPIO_t** sck_gpios;
    const STM32_GPIO_t** miso_gpios;
    const STM32_GPIO_t** mosi_gpios;
    uint8_t gpio_af;
    const STM32_DMAChannel_t* tx_dmas;
    const STM32_DMAChannel_t* rx_dmas;
    
    STM32_DMAStream_t* tx_dma_ = nullptr;
    STM32_DMAStream_t* rx_dma_ = nullptr;

    STM32_SPI_t(SPI_TypeDef* instance, const STM32_GPIO_t** sck_gpios, const STM32_GPIO_t** miso_gpios, const STM32_GPIO_t** mosi_gpios, uint8_t gpio_af,
            const STM32_DMAChannel_t* tx_dmas, const STM32_DMAChannel_t* rx_dmas) :
        hspi{ .Instance = instance },
        sck_gpios(sck_gpios),
        miso_gpios(miso_gpios),
        mosi_gpios(mosi_gpios),
        gpio_af(gpio_af),
        tx_dmas(tx_dmas),
        rx_dmas(rx_dmas) {}

    IRQn_Type get_irq_number() {
        if (hspi.Instance == SPI1)
            return SPI1_IRQn;
        else if (hspi.Instance == SPI2)
            return SPI2_IRQn;
        else if (hspi.Instance == SPI3)
            return SPI3_IRQn;
        else
            return (IRQn_Type)-1;
    }

    bool init(STM32_GPIO_t* sck_gpio, STM32_GPIO_t* miso_gpio, STM32_GPIO_t* mosi_gpio, STM32_DMAStream_t* tx_dma, STM32_DMAStream_t* rx_dma);
    bool enable_interrupts(uint8_t priority);
};

extern STM32_SPI_t spi1, spi2, spi3;

#endif // __STM32_SPI_HPP