
#include "stm32_dma.hpp"
#include "stm32_system.h"

static uint32_t channel_ids[] = {
    DMA_CHANNEL_0,
    DMA_CHANNEL_1,
    DMA_CHANNEL_2,
    DMA_CHANNEL_3,
    DMA_CHANNEL_4,
    DMA_CHANNEL_5,
    DMA_CHANNEL_6,
    DMA_CHANNEL_7,
#if defined(DMA_SxCR_CHSEL_3)
    DMA_CHANNEL_8,
    DMA_CHANNEL_9,
    DMA_CHANNEL_10,
    DMA_CHANNEL_11,
    DMA_CHANNEL_12,
    DMA_CHANNEL_13,
    DMA_CHANNEL_14,
    DMA_CHANNEL_15,
#endif /* DMA_SxCR_CHSEL_3 */
};

uint8_t STM32_DMAStream_t::find_in_list(const STM32_DMAChannel_t* channel_list) {
    // It is unlikely that we check against more than 256 channels but the upper
    // bound prevents a potential endless loop.
    for (size_t i = 0; i < 256; ++i) {
        if (channel_list[i].stream == this)
            return channel_list[i].channel_num;
        if (!channel_list[i].stream)
            return UINT8_MAX;
    }
    return UINT8_MAX;
}

bool STM32_DMAStream_t::init(const STM32_DMAChannel_t* channels, DOMAIN src, DOMAIN dst, ALIGNMENT alignment, MODE mode, PRIORITY priority) {
    uint8_t channel_number = find_in_list(channels);
    if (channel_number >= sizeof(channel_ids) / sizeof(channel_ids[0]))
        return false;
    uint32_t channel_id = channel_ids[channel_number];

    hdma.Init.Channel = channel_id;

    if (src == MEMORY && dst == MEMORY) {
        hdma.Init.Direction = DMA_MEMORY_TO_MEMORY;
    } else if (src == PERIPHERAL && dst == MEMORY) {
        hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    } else if (src == MEMORY && dst == PERIPHERAL) {
        hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    } else {
        return false;
    }

    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;

    switch (alignment) {
        case ALIGN_8_BIT:
            hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            hdma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            break;
        case ALIGN_16_BIT:
            hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
            hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
            break;
        case ALIGN_32_BIT:
            hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
            hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
            break;
        default:
            return false;
    }

    switch (mode) {
        case LINEAR: hdma.Init.Mode = DMA_NORMAL; break;
        case CIRCULAR: hdma.Init.Mode = DMA_CIRCULAR; break;
        default: return false;
    }

    switch (priority) {
        case LOW: hdma.Init.Priority = DMA_PRIORITY_LOW; break;
        case MEDIUM: hdma.Init.Priority = DMA_PRIORITY_MEDIUM; break;
        case HIGH: hdma.Init.Priority = DMA_PRIORITY_HIGH; break;
        case VERY_HIGH: hdma.Init.Priority = DMA_PRIORITY_VERY_HIGH; break;
        default: return false;
    }
    
    hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;

    if (hdma_parent == DMA1) {
        __HAL_RCC_DMA1_CLK_ENABLE();
    } else if (hdma_parent == DMA2) {
        __HAL_RCC_DMA2_CLK_ENABLE();
    } else {
        return false;
    }

    if (HAL_DMA_Init(&hdma) != HAL_OK)
        return false;

    return true;
}

bool STM32_DMAStream_t::enable_interrupts(uint8_t priority) {
    enable_interrupt(irqn, priority);
    return true;
}


/* Interrupt entrypoints -----------------------------------------------------*/

volatile uint32_t dma2_stream1_irq_ticks = 0;
volatile uint32_t dma2_stream2_irq_ticks = 0;

extern "C" {
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
}

/** @brief This function handles DMA1 stream0 global interrupt. */
void DMA1_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream0.hdma);
}

/** @brief This function handles DMA1 stream1 global interrupt. */
void DMA1_Stream1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream1.hdma);
}

/** @brief This function handles DMA1 stream2 global interrupt. */
void DMA1_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream2.hdma);
}

/** @brief This function handles DMA1 stream3 global interrupt. */
void DMA1_Stream3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream3.hdma);
}

/** @brief This function handles DMA1 stream4 global interrupt. */
void DMA1_Stream4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream4.hdma);
}

/** @brief This function handles DMA1 stream5 global interrupt. */
void DMA1_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream5.hdma);
}

/** @brief This function handles DMA1 stream6 global interrupt. */
void DMA1_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream6.hdma);
}

/** @brief This function handles DMA1 stream7 global interrupt. */
void DMA1_Stream7_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma1_stream7.hdma);
}

/** @brief This function handles DMA2 stream0 global interrupt. */
void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma2_stream0.hdma);
}

/** @brief This function handles DMA2 stream1 global interrupt. */
void DMA2_Stream1_IRQHandler(void) {
    PerformanceCounter_t cnt(dma2_stream1_irq_ticks);
    HAL_DMA_IRQHandler(&dma2_stream1.hdma);
}

/** @brief This function handles DMA2 stream2 global interrupt. */
void DMA2_Stream2_IRQHandler(void) {
    PerformanceCounter_t cnt(dma2_stream2_irq_ticks);
    HAL_DMA_IRQHandler(&dma2_stream2.hdma);
}

/** @brief This function handles DMA2 stream3 global interrupt. */
void DMA2_Stream3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma2_stream3.hdma);
}

/** @brief This function handles DMA2 stream4 global interrupt. */
void DMA2_Stream4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma2_stream4.hdma);
}

/** @brief This function handles DMA2 stream5 global interrupt. */
void DMA2_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma2_stream5.hdma);
}

/** @brief This function handles DMA2 stream6 global interrupt. */
void DMA2_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma2_stream6.hdma);
}

/** @brief This function handles DMA2 stream7 global interrupt. */
void DMA2_Stream7_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma2_stream7.hdma);
}


/* Peripheral definitions ----------------------------------------------------*/

STM32_DMAStream_t dma1_stream0{DMA1, DMA1_Stream0, DMA1_Stream0_IRQn};
STM32_DMAStream_t dma1_stream1{DMA1, DMA1_Stream1, DMA1_Stream1_IRQn};
STM32_DMAStream_t dma1_stream2{DMA1, DMA1_Stream2, DMA1_Stream2_IRQn};
STM32_DMAStream_t dma1_stream3{DMA1, DMA1_Stream3, DMA1_Stream3_IRQn};
STM32_DMAStream_t dma1_stream4{DMA1, DMA1_Stream4, DMA1_Stream4_IRQn};
STM32_DMAStream_t dma1_stream5{DMA1, DMA1_Stream5, DMA1_Stream5_IRQn};
STM32_DMAStream_t dma1_stream6{DMA1, DMA1_Stream6, DMA1_Stream6_IRQn};
STM32_DMAStream_t dma1_stream7{DMA1, DMA1_Stream7, DMA1_Stream7_IRQn};

STM32_DMAStream_t dma2_stream0{DMA2, DMA2_Stream0, DMA2_Stream0_IRQn};
STM32_DMAStream_t dma2_stream1{DMA2, DMA2_Stream1, DMA2_Stream1_IRQn};
STM32_DMAStream_t dma2_stream2{DMA2, DMA2_Stream2, DMA2_Stream2_IRQn};
STM32_DMAStream_t dma2_stream3{DMA2, DMA2_Stream3, DMA2_Stream3_IRQn};
STM32_DMAStream_t dma2_stream4{DMA2, DMA2_Stream4, DMA2_Stream4_IRQn};
STM32_DMAStream_t dma2_stream5{DMA2, DMA2_Stream5, DMA2_Stream5_IRQn};
STM32_DMAStream_t dma2_stream6{DMA2, DMA2_Stream6, DMA2_Stream6_IRQn};
STM32_DMAStream_t dma2_stream7{DMA2, DMA2_Stream7, DMA2_Stream7_IRQn};
