
#include "stm32_adc.hpp"

const float adc_full_scale = (float)(1 << 12);
const float adc_ref_voltage = 3.3f;

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
    if (!adc || !adc->init()) {
        return false;
    }
    return !dma; // DMA not supported on injected sequence
}

bool STM32_ADCRegular_t::init(STM32_DMAStream_t* dma) {
    if (!adc || !adc->init()) {
        return false;
    }
    
    if (dma) {
        dma_ = dma;
        if (!dma->init(adc->dmas, DMA_t::PERIPHERAL, DMA_t::MEMORY, DMA_t::ALIGN_16_BIT, DMA_t::CIRCULAR, DMA_t::LOW)) {
            return false;
        }
        dma->link(adc->hadc, &ADC_HandleTypeDef::DMA_Handle);
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
    if (!adc)
        return false;

    adc->hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc->hadc.Init.Resolution = ADC_RESOLUTION_12B;
    adc->hadc.Init.ScanConvMode = ENABLE; // scan entire sequence
    adc->hadc.Init.ContinuousConvMode = DISABLE;
    adc->hadc.Init.DiscontinuousConvMode = DISABLE;
    adc->hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc->hadc.Init.ExternalTrigConv = trigger_source;
    adc->hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc->hadc.Init.NbrOfConversion = channel_sequence_length;
    adc->hadc.Init.DMAContinuousRequests = dma_ ? ENABLE : DISABLE;
    adc->hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV; // generate an interrupt after end of each regular conversion
    if (HAL_ADC_Init(&adc->hadc) != HAL_OK)
        return false;
    
    for (size_t i = 0; i < channel_sequence_length; ++i) {
        STM32_ADCChannel_t* channel = channel_sequence[i];
        if (!channel || !channel->is_valid())
            return false;

        ADC_ChannelConfTypeDef sConfig;
        sConfig.Channel = channel->channel_num_ << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = i + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(&adc->hadc, &sConfig) != HAL_OK)
            return false;
    }

    return true;
}

bool STM32_ADCRegular_t::enable() {
    if (adc) {
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);

        if (dma_) {
            if (HAL_ADC_Start_DMA(&adc->hadc, reinterpret_cast<uint32_t*>(raw_values), channel_sequence_length) != HAL_OK) {
                return false;
            }
        } else {
            __HAL_ADC_ENABLE_IT(&adc->hadc, ADC_IT_EOC);
        }
        __HAL_ADC_ENABLE(&adc->hadc);
        return true;
    } else {
        return false;
    }
}

bool STM32_ADCInjected_t::apply() {
    if (!adc)
        return false;

    for (size_t i = 0; i < channel_sequence_length; ++i) {
        STM32_ADCChannel_t* channel = channel_sequence[i];
        if (!channel || !channel->is_valid())
            return false;

        ADC_InjectionConfTypeDef sConfigInjected;
        sConfigInjected.InjectedChannel = channel->channel_num_ << ADC_CR1_AWDCH_Pos;
        sConfigInjected.InjectedRank = i + 1; // TODO not sure if the numbering should depend on sequence length, see note on ADC_JSQR (datasheet page 424)
        sConfigInjected.InjectedNbrOfConversion = channel_sequence_length;
        sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
        sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
        sConfigInjected.ExternalTrigInjecConv = trigger_source;
        sConfigInjected.AutoInjectedConv = DISABLE;
        sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
        sConfigInjected.InjectedOffset = 0;
        if (HAL_ADCEx_InjectedConfigChannel(&adc->hadc, &sConfigInjected) != HAL_OK)
            return false;
    }

    return true;
}

bool STM32_ADCInjected_t::enable() {
    if (adc) {
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);

        __HAL_ADC_ENABLE_IT(&adc->hadc, ADC_IT_JEOC);
        __HAL_ADC_ENABLE(&adc->hadc);
        return true;
    } else {
        return false;
    }
}

void STM32_ADCRegular_t::handle_irq() {
    if (adc) {
        uint32_t EOC = __HAL_ADC_GET_FLAG(&adc->hadc, ADC_FLAG_EOC);
        uint32_t EOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(&adc->hadc, ADC_IT_EOC);
        if (EOC && EOC_IT_EN) {
            __HAL_ADC_CLEAR_FLAG(&adc->hadc, (ADC_FLAG_STRT | ADC_FLAG_EOC));
            
            if (next_pos >= channel_sequence_length)
                next_pos = 0;
            raw_values[next_pos] = adc->hadc.Instance->DR;

            STM32_ADCChannel_t* channel = channel_sequence[next_pos];
            if (channel) {
                channel->handle_update();
            }
            
            next_pos++;
        }
    }
}

void STM32_ADCRegular_t::handle_dma_complete() {
    if (adc) {
        for (size_t i = 0; i < channel_sequence_length; ++i) {
            STM32_ADCChannel_t* channel = channel_sequence[i];
            if (channel) {
                channel->handle_update();
            }
        }
    }
}

void STM32_ADCInjected_t::handle_irq() {
    if (adc) {
        uint32_t JEOC = __HAL_ADC_GET_FLAG(&adc->hadc, ADC_FLAG_JEOC);
        uint32_t JEOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(&adc->hadc, ADC_IT_JEOC);
        if (JEOC && JEOC_IT_EN) {
            __HAL_ADC_CLEAR_FLAG(&adc->hadc, (ADC_FLAG_JSTRT | ADC_FLAG_JEOC));
            raw_values[0] = adc->hadc.Instance->JDR1;
            raw_values[1] = adc->hadc.Instance->JDR2;
            raw_values[2] = adc->hadc.Instance->JDR3;
            raw_values[3] = adc->hadc.Instance->JDR4;
            for (size_t i = 0; i < channel_sequence_length; ++i) {
                STM32_ADCChannel_t* channel = channel_sequence[i];
                if (channel) {
                    channel->handle_update();
                }
            }
        }
    }
}

bool STM32_ADCChannel_t::get_voltage(float* value) {
    static const float voltage_scale = adc_ref_voltage / adc_full_scale;
    uint16_t raw_value;

    if (!adc_ || !adc_->get_raw_value(seq_pos_, &raw_value))
        return false;

    if (value)
        *value = (float)raw_value * voltage_scale;

    return true;
}

bool STM32_ADCChannel_t::get_normalized(float* value) {
    uint16_t raw_value;

    if (!adc_ || adc_->get_raw_value(seq_pos_, &raw_value))
        return false;

    if (value)
        *value = (float)raw_value / adc_full_scale;

    return true;
}

bool STM32_ADCChannel_t::enable_updates() {
    if (adc_) {
        adc_->enable();
        return true;
    } else {
        return false;
    }
}


/* Interrupt entrypoints -----------------------------------------------------*/

/** @brief Entrypoint for the ADC1, ADC2 and ADC3 global interrupts. */
extern "C" void ADC_IRQHandler(void) {
    adc1_injected.handle_irq();
    adc2_injected.handle_irq();
    adc3_injected.handle_irq();
    // adc1_regular.handle_irq(); // non-DMA regular conversions not used
    // adc2_regular.handle_irq(); // non-DMA regular conversions not used
    // adc3_regular.handle_irq(); // non-DMA regular conversions not used
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

STM32_ADCRegular_t adc1_regular(&adc1);
STM32_ADCRegular_t adc2_regular(&adc2);
STM32_ADCRegular_t adc3_regular(&adc3);

STM32_ADCInjected_t adc1_injected(&adc1);
STM32_ADCInjected_t adc2_injected(&adc2);
STM32_ADCInjected_t adc3_injected(&adc3);
