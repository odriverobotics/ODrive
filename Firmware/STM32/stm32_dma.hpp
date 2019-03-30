#ifndef __STM32_DMA_HPP
#define __STM32_DMA_HPP

#include "stm32f4xx.h"

class DMA_t {
public:
    enum DOMAIN {
        MEMORY,
        PERIPHERAL
    };
    enum ALIGNMENT {
        ALIGN_8_BIT,
        ALIGN_16_BIT,
        ALIGN_32_BIT
    };
    enum MODE {
        LINEAR,
        CIRCULAR
    };
    enum PRIORITY {
        LOW,
        MEDIUM,
        HIGH,
        VERY_HIGH
    };
};

class STM32_DMAStream_t;

struct STM32_DMAChannel_t {
    STM32_DMAStream_t* stream;
    uint8_t channel_num;
};

class STM32_DMAStream_t : DMA_t {
public:
    DMA_TypeDef* hdma_parent;
    DMA_HandleTypeDef hdma;
    IRQn_Type irqn;

    STM32_DMAStream_t(DMA_TypeDef* hdma_parent, DMA_Stream_TypeDef* instance, IRQn_Type irqn) :
            hdma_parent(hdma_parent),
            hdma{ .Instance = instance },
            irqn(irqn) {}

    /*
    * @brief Tries to find this stream in the given channel list and returns the
    * associated channel number. The list must be terminated with a { nullptr, 0 }
    * item.
    * Returns a channel number (below 0xFF) if the stream is found and 0xFF otherwise.
    * The list must not contain more than 256 elements.
    */
    uint8_t find_in_list(const STM32_DMAChannel_t* channel_list);

    bool init(const STM32_DMAChannel_t* channels, DOMAIN src, DOMAIN dst, ALIGNMENT alignment, MODE mode, PRIORITY priority);

    template<typename T>
    void link(T& owner, DMA_HandleTypeDef* T::*field) {
        // does the same thing as __HAL_LINKDMA(&owner, field, hdma); in a C++ way
        owner.*field = &hdma;
        hdma.Parent = &owner;
    }

    bool enable_interrupts(uint8_t priority);
};

extern volatile uint32_t dma2_stream1_irq_ticks;
extern volatile uint32_t dma2_stream2_irq_ticks;

extern STM32_DMAStream_t dma1_stream0, dma1_stream1, dma1_stream2, dma1_stream3, dma1_stream4, dma1_stream5, dma1_stream6, dma1_stream7;
extern STM32_DMAStream_t dma2_stream0, dma2_stream1, dma2_stream2, dma2_stream3, dma2_stream4, dma2_stream5, dma2_stream6, dma2_stream7;


#endif // __STM32_DMA_HPP