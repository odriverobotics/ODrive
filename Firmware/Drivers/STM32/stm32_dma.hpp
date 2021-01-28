#ifndef __STM32_DMA_HPP
#define __STM32_DMA_HPP

#include <Drivers/STM32/stm32_system.h>

struct Stm32DmaStreamRef {
public:
    Stm32DmaStreamRef(DMA_Stream_TypeDef* ref) : ref_(ref) {}
    
    static Stm32DmaStreamRef none() {
        return Stm32DmaStreamRef{nullptr};
    };

    IRQn_Type get_irqn() {
        switch ((uint32_t)ref_) {
            case DMA1_Stream0_BASE: return DMA1_Stream0_IRQn;
            case DMA1_Stream1_BASE: return DMA1_Stream1_IRQn;
            case DMA1_Stream2_BASE: return DMA1_Stream2_IRQn;
            case DMA1_Stream3_BASE: return DMA1_Stream3_IRQn;
            case DMA1_Stream4_BASE: return DMA1_Stream4_IRQn;
            case DMA1_Stream5_BASE: return DMA1_Stream5_IRQn;
            case DMA1_Stream6_BASE: return DMA1_Stream6_IRQn;
            case DMA1_Stream7_BASE: return DMA1_Stream7_IRQn;
            case DMA2_Stream0_BASE: return DMA2_Stream0_IRQn;
            case DMA2_Stream1_BASE: return DMA2_Stream1_IRQn;
            case DMA2_Stream2_BASE: return DMA2_Stream2_IRQn;
            case DMA2_Stream3_BASE: return DMA2_Stream3_IRQn;
            case DMA2_Stream4_BASE: return DMA2_Stream4_IRQn;
            case DMA2_Stream5_BASE: return DMA2_Stream5_IRQn;
            case DMA2_Stream6_BASE: return DMA2_Stream6_IRQn;
            case DMA2_Stream7_BASE: return DMA2_Stream7_IRQn;
        }
        return UsageFault_IRQn; // maybe this is not the smartest return value
    }

    DMA_Stream_TypeDef* get_instance() {
        return ref_;
    }

    bool enable_clock() {
        switch ((uint32_t)ref_) {
            case DMA1_Stream0_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream1_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream2_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream3_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream4_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream5_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream6_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA1_Stream7_BASE: __HAL_RCC_DMA1_CLK_ENABLE(); break;
            case DMA2_Stream0_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream1_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream2_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream3_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream4_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream5_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream6_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            case DMA2_Stream7_BASE: __HAL_RCC_DMA2_CLK_ENABLE(); break;
            default: return false;
        }
        return true;
    }

private:
    DMA_Stream_TypeDef* ref_;
};

#endif // __STM32_DMA_HPP
