#ifndef __STM32_USART_HPP
#define __STM32_USART_HPP

#include "stm32_dma.hpp"

struct Stm32Usart {
    Stm32Usart(USART_TypeDef* instance, Stm32DmaStreamRef rx_dma, Stm32DmaStreamRef tx_dma);
    bool init(uint32_t baudrate);

    IRQn_Type get_irqn() {
        switch ((uint32_t)huart_.Instance) {
            case USART1_BASE: return USART1_IRQn;
            case USART2_BASE: return USART2_IRQn;
            case USART3_BASE: return USART3_IRQn;
            case UART4_BASE: return UART4_IRQn;
            case UART5_BASE: return UART5_IRQn;
            case USART6_BASE: return USART6_IRQn;
            default: return UsageFault_IRQn;
        }
    }

    operator UART_HandleTypeDef*() {
        return &huart_;
    }

    UART_HandleTypeDef huart_;
    DMA_HandleTypeDef hdma_rx_;
    DMA_HandleTypeDef hdma_tx_;
};

#endif // __STM32_USART_HPP
