#ifndef __STM32_SPI_HPP
#define __STM32_SPI_HPP

#include "stm32_dma.hpp"

struct Stm32Spi {
    struct Config {
        uint32_t max_baud_rate; // SPI can choose lower baud rate
        uint32_t clk_polarity;
        uint32_t clk_phase;

        bool operator==(const Config& rhs) {
            return (max_baud_rate == rhs.max_baud_rate)
                && (clk_polarity == rhs.clk_polarity)
                && (clk_phase == rhs.clk_phase);
        }
        bool operator!=(const Config& rhs) {
            return !(*this == rhs);
        }
    };

    Stm32Spi(SPI_TypeDef* instance, Stm32DmaStreamRef rx_dma, Stm32DmaStreamRef tx_dma);
    bool init();
    bool config(Config config);

    IRQn_Type get_irqn() {
        switch ((uint32_t)hspi_.Instance) {
            case SPI1_BASE: return SPI1_IRQn;
            case SPI2_BASE: return SPI2_IRQn;
            case SPI3_BASE: return SPI3_IRQn;
            default: return UsageFault_IRQn;
        }
    }

    operator SPI_HandleTypeDef*() {
        return &hspi_;
    }

    SPI_HandleTypeDef hspi_;
    DMA_HandleTypeDef hdma_rx_;
    DMA_HandleTypeDef hdma_tx_;
    uint32_t freq_; // peripheral core clock frequency
};

#endif // __STM32_SPI_HPP
