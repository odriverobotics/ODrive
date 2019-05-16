
#include "stm32_adc.hpp"
#include "stm32_system.h"

static bool convert_sampling_time(uint32_t sampling_time, uint32_t* result) {
    uint32_t dummy;
    if (!result) {
        result = &dummy;
    }
    switch (sampling_time) {
        case 3: return (*result = ADC_SAMPLETIME_3CYCLES), true;
        case 15: return (*result = ADC_SAMPLETIME_15CYCLES), true;
        case 28: return (*result = ADC_SAMPLETIME_28CYCLES), true;
        case 56: return (*result = ADC_SAMPLETIME_56CYCLES), true;
        case 84: return (*result = ADC_SAMPLETIME_84CYCLES), true;
        case 112: return (*result = ADC_SAMPLETIME_112CYCLES), true;
        case 144: return (*result = ADC_SAMPLETIME_144CYCLES), true;
        case 480: return (*result = ADC_SAMPLETIME_480CYCLES), true;
        default: return false;
    }
}

bool STM32_ADC_t::init() {
    if (is_setup_) {
        return true;
    }

    if (hadc.Instance == ADC1)
        __HAL_RCC_ADC1_CLK_ENABLE();
    else if (hadc.Instance == ADC2)
        __HAL_RCC_ADC2_CLK_ENABLE();
    else if (hadc.Instance == ADC3)
        __HAL_RCC_ADC3_CLK_ENABLE();
    else
        return false;

    // Some of this is overwritten when the regular sequence is applied.
    // However we do a minimal init here too so the injected sequence works even if the regular sequence is not used.
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.ScanConvMode = ENABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = 0;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
        return false;

    is_setup_ = true;
    return true;
}

bool STM32_ADCInjected_t::init(STM32_DMAStream_t* dma) {
    if (!adc_.init()) {
        return false;
    }
    return !dma; // DMA not supported on injected sequence
}

bool STM32_ADCRegular_t::init(STM32_DMAStream_t* dma) {
    if (!adc_.init()) {
        return false;
    }
    
    if (dma) {
        dma_ = dma;
        if (!dma->init(adc_.dmas, DMA_t::PERIPHERAL, DMA_t::MEMORY, DMA_t::ALIGN_16_BIT, DMA_t::CIRCULAR, DMA_t::VERY_HIGH)) {
            return false;
        }
        dma->link(adc_.hadc, &ADC_HandleTypeDef::DMA_Handle);
    } else {
        return false; // non-DMA not implemented yet
    }

    return true;
}

bool STM32_ADCRegular_t::set_trigger(STM32_Timer_t* timer) {
    if (!timer)
        return false;

    // note: more trigger sources are available but currently not implemented
    if (timer->htim.Instance == TIM2) {
        trigger_source = ADC_EXTERNALTRIGCONV_T2_TRGO;
    } else if (timer->htim.Instance == TIM3) {
        trigger_source = ADC_EXTERNALTRIGCONV_T3_TRGO;
    } else if (timer->htim.Instance == TIM8) {
        trigger_source = ADC_EXTERNALTRIGCONV_T8_TRGO;
    } else {
        return false;
    }

    return true;
}

bool STM32_ADCInjected_t::set_trigger(STM32_Timer_t* timer) {
    if (!timer)
        return false;

    // note: more trigger sources are available but currently not implemented
    if (timer->htim.Instance == TIM1) {
        trigger_source = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    } else if (timer->htim.Instance == TIM2) {
        trigger_source = ADC_EXTERNALTRIGINJECCONV_T2_TRGO;
    } else if (timer->htim.Instance == TIM4) {
        trigger_source = ADC_EXTERNALTRIGINJECCONV_T4_TRGO;
    } else if (timer->htim.Instance == TIM5) {
        trigger_source = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
    } else {
        return false;
    }

    return true;
}

bool STM32_ADCRegular_t::apply() {
    adc_.hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc_.hadc.Init.Resolution = ADC_RESOLUTION_12B;
    adc_.hadc.Init.ScanConvMode = ENABLE; // scan entire sequence
    adc_.hadc.Init.ContinuousConvMode = DISABLE;
    adc_.hadc.Init.DiscontinuousConvMode = DISABLE;
    adc_.hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc_.hadc.Init.ExternalTrigConv = trigger_source;
    adc_.hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_.hadc.Init.NbrOfConversion = channel_sequence_length;
    adc_.hadc.Init.DMAContinuousRequests = dma_ ? ENABLE : DISABLE;
    adc_.hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV; // generate an interrupt after end of each regular conversion
    if (HAL_ADC_Init(&adc_.hadc) != HAL_OK)
        return false;
    
    for (size_t i = 0; i < channel_sequence_length; ++i) {
        Channel_t* channel = channel_sequence[i];
        if (!channel || !channel->is_valid())
            return false;

        ADC_ChannelConfTypeDef sConfig;
        sConfig.Channel = channel->channel_num_ << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = i + 1;
        if (!convert_sampling_time(channel->sampling_time, &sConfig.SamplingTime))
            return false;
        if (HAL_ADC_ConfigChannel(&adc_.hadc, &sConfig) != HAL_OK)
            return false;
    }

    return true;
}

bool STM32_ADCRegular_t::enable_updates() {
    if (dma_ && (HAL_ADC_Start_DMA(&adc_.hadc, reinterpret_cast<uint32_t*>(raw_values), channel_sequence_length) != HAL_OK)) {
        return false;
    }
    __HAL_ADC_ENABLE(&adc_.hadc);
    return true;
}

bool STM32_ADCRegular_t::enable_interrupts(uint8_t priority) {
    enable_interrupt(ADC_IRQn, priority);

    if (dma_) {
        dma_->enable_interrupts(priority);
    } else {
        __HAL_ADC_ENABLE_IT(&adc_.hadc, ADC_IT_EOC);
    }
    return true;
}

bool STM32_ADCRegular_t::disable() {
    return HAL_ADC_Stop_DMA(&adc_.hadc) == HAL_OK;
}

bool STM32_ADCRegular_t::has_completed() {
    uint32_t OVR = __HAL_ADC_GET_FLAG(&adc_.hadc, ADC_FLAG_OVR);
    if (OVR) {
        disable();
        error_ = true;
    }
    if (error_) {
        return false;
    }
    if (dma_) {
        DMA_Base_Registers *regs = (DMA_Base_Registers *)dma_->hdma.StreamBaseAddress;
        return regs->ISR & (DMA_FLAG_TCIF0_4 << dma_->hdma.StreamIndex);
    } else {
        return __HAL_ADC_GET_FLAG(&adc_.hadc, ADC_FLAG_EOC);
    }
}

bool STM32_ADCRegular_t::reset_values() {
    if (dma_) {
        DMA_Base_Registers *regs = (DMA_Base_Registers *)dma_->hdma.StreamBaseAddress;
        regs->IFCR = DMA_FLAG_TCIF0_4 << dma_->hdma.StreamIndex;
    } else {
        __HAL_ADC_CLEAR_FLAG(&adc_.hadc, ADC_FLAG_EOC);
    }
    return true;
}

bool STM32_ADCRegular_t::get_raw_value(size_t seq_pos, uint16_t *raw_value) {
    if (seq_pos < channel_sequence_length) {
        if (raw_value) {
            *raw_value = raw_values[seq_pos];
        }
        return true;
    } else {
        return false;
    }
}

bool STM32_ADCInjected_t::apply() {
    for (size_t i = 0; i < channel_sequence_length; ++i) {
        Channel_t* channel = channel_sequence[i];
        if (!channel || !channel->is_valid())
            return false;

        ADC_InjectionConfTypeDef sConfigInjected;
        sConfigInjected.InjectedChannel = channel->channel_num_ << ADC_CR1_AWDCH_Pos;
        sConfigInjected.InjectedRank = i + 1; // TODO not sure if the numbering should depend on sequence length, see note on ADC_JSQR (datasheet page 424)
        sConfigInjected.InjectedNbrOfConversion = channel_sequence_length;
        if (!convert_sampling_time(channel->sampling_time, &sConfigInjected.InjectedSamplingTime))
            return false;
        sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
        sConfigInjected.ExternalTrigInjecConv = trigger_source;
        sConfigInjected.AutoInjectedConv = DISABLE;
        sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
        sConfigInjected.InjectedOffset = 0;
        if (HAL_ADCEx_InjectedConfigChannel(&adc_.hadc, &sConfigInjected) != HAL_OK)
            return false;
    }

    return true;
}

bool STM32_ADCInjected_t::enable_updates() {
    __HAL_ADC_ENABLE(&adc_.hadc);
    return true;
}

bool STM32_ADCInjected_t::enable_interrupts(uint8_t priority) {
    enable_interrupt(ADC_IRQn, priority);
    __HAL_ADC_ENABLE_IT(&adc_.hadc, ADC_IT_JEOC);
    return true;
}

bool STM32_ADCInjected_t::disable() {
    __HAL_ADC_DISABLE_IT(&adc_.hadc, ADC_IT_JEOC);
    return true;
}

bool STM32_ADCInjected_t::has_completed() {
    return __HAL_ADC_GET_FLAG(&adc_.hadc, ADC_FLAG_JEOC) != 0;
}

bool STM32_ADCInjected_t::reset_values() {
    return __HAL_ADC_CLEAR_FLAG(&adc_.hadc, ADC_FLAG_JEOC), true;
}

bool STM32_ADCInjected_t::get_raw_value(size_t seq_pos, uint16_t *raw_value) {
    uint16_t dummy;
    if (!raw_value)
        raw_value = &dummy;

    switch (seq_pos) {
        case 0: return (*raw_value = adc_.hadc.Instance->JDR1), true;
        case 1: return (*raw_value = adc_.hadc.Instance->JDR2), true;
        case 2: return (*raw_value = adc_.hadc.Instance->JDR3), true;
        case 3: return (*raw_value = adc_.hadc.Instance->JDR4), true;
        default: return false;
    }
}

void STM32_ADCRegular_t::handle_irq() {
    uint32_t OVR = __HAL_ADC_GET_FLAG(&adc_.hadc, ADC_FLAG_OVR);
    if (OVR) {
        disable();
        error_ = true;
    }

/*      uint32_t EOC = __HAL_ADC_GET_FLAG(&adc_.hadc, ADC_FLAG_EOC);
    uint32_t EOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(&adc_.hadc, ADC_IT_EOC);
    if (EOC && EOC_IT_EN) {
        __HAL_ADC_CLEAR_FLAG(&adc_.hadc, (ADC_FLAG_STRT | ADC_FLAG_EOC));
        
        if (next_pos >= channel_sequence_length)
            next_pos = 0;
        raw_values[next_pos] = adc_.hadc.Instance->DR;

        Channel_t* channel = channel_sequence[next_pos];
        if (channel) {
            channel->handle_update();
        }
        
        next_pos++;
    }*/
}

void STM32_ADCRegular_t::handle_dma_complete() {
    for (size_t i = 0; i < channel_sequence_length; ++i) {
        Channel_t* channel = channel_sequence[i];
        if (channel) {
            channel->handle_update();
        }
    }
}

void STM32_ADCInjected_t::handle_irq() {
    uint32_t JEOC = __HAL_ADC_GET_FLAG(&adc_.hadc, ADC_FLAG_JEOC);
    uint32_t JEOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(&adc_.hadc, ADC_IT_JEOC);
    if (JEOC && JEOC_IT_EN) {
        __HAL_ADC_CLEAR_FLAG(&adc_.hadc, (ADC_FLAG_JSTRT | ADC_FLAG_JEOC));
        for (size_t i = 0; i < channel_sequence_length; ++i) {
            Channel_t* channel = channel_sequence[i];
            if (channel) {
                channel->handle_update();
            }
        }
    }
}


/* Interrupt entrypoints -----------------------------------------------------*/

volatile uint32_t adc_irq_ticks = 0;

/** @brief Entrypoint for the ADC1, ADC2 and ADC3 global interrupts. */
extern "C" void ADC_IRQHandler(void) {
    PerformanceCounter_t cnt(adc_irq_ticks);
    adc1_injected.handle_irq();
    adc2_injected.handle_irq();
    adc3_injected.handle_irq();
    adc1_regular.handle_irq();
    adc2_regular.handle_irq();
    adc3_regular.handle_irq();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &adc1.hadc) {
        adc1_regular.handle_dma_complete();
    } else if (hadc == &adc2.hadc) {
        adc2_regular.handle_dma_complete();
    } else if (hadc == &adc3.hadc) {
        adc3_regular.handle_dma_complete();
    }
}


/* Peripheral definitions ----------------------------------------------------*/

const std::array<STM32_GPIO_t*, 16> adc1_gpios = { &pa0, &pa1, &pa2, &pa3, &pa4, &pa5, &pa6, &pa7, &pb0, &pb1, &pc0, &pc1, &pc2, &pc3, &pc4, &pc5 };
const STM32_DMAChannel_t adc1_dmas[] = { {&dma2_stream0, 0}, {&dma2_stream4, 0}, {0} };

const std::array<STM32_GPIO_t*, 16> adc2_gpios = { &pa0, &pa1, &pa2, &pa3, &pa4, &pa5, &pa6, &pa7, &pb0, &pb1, &pc0, &pc1, &pc2, &pc3, &pc4, &pc5 };
const STM32_DMAChannel_t adc2_dmas[] = { {&dma2_stream2, 1}, {&dma2_stream3, 1}, {0} };

const std::array<STM32_GPIO_t*, 16> adc3_gpios = { &pa0, &pa1, &pa2, &pa3, &pf6, &pf7, &pf8, &pf8, &pf10, &pf3, &pc0, &pc1, &pc2, &pc3, &pf4, &pf5 };
const STM32_DMAChannel_t adc3_dmas[] = { {&dma2_stream0, 2}, {&dma2_stream1, 2}, {0} };

STM32_ADC_t adc1(ADC1, adc1_gpios, adc1_dmas);
STM32_ADC_t adc2(ADC2, adc2_gpios, adc2_dmas);
STM32_ADC_t adc3(ADC3, adc3_gpios, adc3_dmas);

STM32_ADCRegular_t adc1_regular(adc1);
STM32_ADCRegular_t adc2_regular(adc2);
STM32_ADCRegular_t adc3_regular(adc3);

STM32_ADCInjected_t adc1_injected(adc1);
STM32_ADCInjected_t adc2_injected(adc2);
STM32_ADCInjected_t adc3_injected(adc3);
