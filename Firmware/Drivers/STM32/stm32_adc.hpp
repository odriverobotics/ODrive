#ifndef __STM32_ADC_HPP
#define __STM32_ADC_HPP

#include "stm32_gpio.hpp"

class Stm32Adc {
public:
    static constexpr int channel_from_gpio(ADC_TypeDef* adc, Stm32Gpio gpio);

    Stm32Adc(ADC_HandleTypeDef* hadc) : hadc_(hadc) {}
    bool set_regular_sequence(const int* channels, size_t n_channels);
private:
    ADC_HandleTypeDef* hadc_;
};

uint32_t channel_id_from_int(int channel_num) {
    static const uint32_t ids[] = {
        ADC_CHANNEL_0,
        ADC_CHANNEL_1,
        ADC_CHANNEL_2,
        ADC_CHANNEL_3,
        ADC_CHANNEL_4,
        ADC_CHANNEL_5,
        ADC_CHANNEL_6,
        ADC_CHANNEL_7,
        ADC_CHANNEL_8,
        ADC_CHANNEL_9,
        ADC_CHANNEL_10,
        ADC_CHANNEL_11,
        ADC_CHANNEL_12,
        ADC_CHANNEL_13,
        ADC_CHANNEL_14,
        ADC_CHANNEL_15,
        ADC_CHANNEL_16,
        ADC_CHANNEL_17,
        ADC_CHANNEL_18,
    };
    if ((channel_num >= 0) && ((size_t)channel_num < sizeof(ids) / sizeof(ids[0]))) {
        return ids[channel_num];
    } else {
        return 0xffffffff; // will be caught by HAL
    }
}

#if defined(STM32F405xx) || defined(STM32F722xx)

bool Stm32Adc::set_regular_sequence(const int* channels, size_t n_channels) {
    if (!IS_ADC_REGULAR_LENGTH(n_channels)) {
        return false;
    }

    ADC_ChannelConfTypeDef sConfig;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    for (size_t rank = 1; rank <= n_channels; ++rank) {
        if (channels[rank - 1] < 0) {
            return false;
        }
        sConfig.Channel = channel_id_from_int(channels[rank - 1]);
        sConfig.Rank = rank;
        if (HAL_ADC_ConfigChannel(hadc_, &sConfig) != HAL_OK) {
            return false;
        }
    }

    hadc_->Instance->SQR1 &= ~(ADC_SQR1_L);
    hadc_->Instance->SQR1 = (hadc_->Instance->SQR1 & ~ADC_SQR1_L) |
                            ADC_SQR1(n_channels);
    return true;
}

constexpr int Stm32Adc::channel_from_gpio(ADC_TypeDef* adc, Stm32Gpio gpio) {
    if (adc == ADC1 || adc == ADC2 || adc == ADC3) {
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_0}) return 0;
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_1}) return 1;
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_2}) return 2;
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_3}) return 3;
        if (gpio == Stm32Gpio{GPIOC, GPIO_PIN_0}) return 10;
        if (gpio == Stm32Gpio{GPIOC, GPIO_PIN_1}) return 11;
        if (gpio == Stm32Gpio{GPIOC, GPIO_PIN_2}) return 12;
        if (gpio == Stm32Gpio{GPIOC, GPIO_PIN_3}) return 13;
    }
    if (adc == ADC1 || adc == ADC2) {
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_4}) return 4;
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_5}) return 5;
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_6}) return 6;
        if (gpio == Stm32Gpio{GPIOA, GPIO_PIN_7}) return 7;
        if (gpio == Stm32Gpio{GPIOB, GPIO_PIN_0}) return 8;
        if (gpio == Stm32Gpio{GPIOB, GPIO_PIN_1}) return 9;
        if (gpio == Stm32Gpio{GPIOC, GPIO_PIN_4}) return 14;
        if (gpio == Stm32Gpio{GPIOC, GPIO_PIN_5}) return 15;
    }
    if (adc == ADC3) {
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_6}) return 4;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_7}) return 5;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_8}) return 6;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_9}) return 7;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_10}) return 8;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_3}) return 9;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_4}) return 14;
        if (gpio == Stm32Gpio{GPIOF, GPIO_PIN_5}) return 15;
    }
    return -1;
}

#endif

#endif // __STM32_ADC_HPP
