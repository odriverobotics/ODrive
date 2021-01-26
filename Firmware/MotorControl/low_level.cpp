/* Includes ------------------------------------------------------------------*/

#include <board.h>

#include <cmsis_os.h>
#include <cmath>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <main.h>
#include <tim.h>
#include <utils.hpp>

#include "odrive_main.h"

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
constexpr float adc_full_scale = static_cast<float>(1UL << 12UL);
constexpr float adc_ref_voltage = 3.3f;
/* Global variables ----------------------------------------------------------*/

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;

/* Function implementations --------------------------------------------------*/

void start_adc_pwm() {
    // Disarm motors
    for (auto& axis: axes) {
        axis.motor_.disarm();
    }

    for (Motor& motor: motors) {
        // Init PWM
        int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
        motor.timer_->Instance->CCR1 = half_load;
        motor.timer_->Instance->CCR2 = half_load;
        motor.timer_->Instance->CCR3 = half_load;

        // Enable PWM outputs (they are still masked by MOE though)
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_3);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_3);
    }

    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);

    start_timers();

    if (odrv.config_.enable_brake_resistor) {
        odrv.brake_resistor_.arm();
    }
}

// @brief ADC1 measurements are written to this buffer by DMA
uint16_t adc_measurements_[ADC_CHANNEL_COUNT] = { 0 };

// @brief Starts the general purpose ADC on the ADC1 peripheral.
// The measured ADC voltages can be read with get_adc_voltage().
//
// ADC1 is set up to continuously sample all channels 0 to 15 in a
// round-robin fashion.
// DMA is used to copy the measured 12-bit values to adc_measurements_.
//
// The injected (high priority) channel of ADC1 is used to sample vbus_voltage.
// This conversion is triggered by TIM1 at the frequency of the motor control loop.
void start_general_purpose_adc() {
    ADC_ChannelConfTypeDef sConfig;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        odrv.misconfigured_ = true; // TODO: this is a bit of an abuse of this flag
        return;
    }

    // Set up sampling sequence (channel 0 ... channel 15)
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    for (uint32_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        sConfig.Channel = channel << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = channel + 1; // rank numbering starts at 1
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            odrv.misconfigured_ = true; // TODO: this is a bit of an abuse of this flag
            return;
        }
    }

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);
}

// @brief Returns the ADC voltage associated with the specified pin.
// This only works if the GPIO was not used for anything else since bootup, otherwise
// it must be put to analog mode first.
// Returns -1.0f if the pin has no associated ADC1 channel.
//
// On ODrive 3.3 and 3.4 the following pins can be used with this function:
//  GPIO_1, GPIO_2, GPIO_3, GPIO_4 and some pins that are connected to
//  on-board sensors (M0_TEMP, M1_TEMP, AUX_TEMP)
//
// The ADC values are sampled in background at ~30kHz without
// any CPU involvement.
//
// Details: each of the 16 conversion takes (15+26) ADC clock
// cycles and the ADC, so the update rate of the entire sequence is:
//  21000kHz / (15+26) / 16 = 32kHz
// The true frequency is slightly lower because of the injected vbus
// measurements
float get_adc_voltage(Stm32Gpio gpio) {
    return get_adc_relative_voltage(gpio) * adc_ref_voltage;
}

float get_adc_relative_voltage(Stm32Gpio gpio) {
    const uint16_t channel = channel_from_gpio(gpio);
    return get_adc_relative_voltage_ch(channel);
}

// @brief Given a GPIO_port and pin return the associated adc_channel.
// returns UINT16_MAX if there is no adc_channel;
uint16_t channel_from_gpio(Stm32Gpio gpio) {
    uint32_t channel = UINT32_MAX;
    if (gpio.port_ == GPIOA) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 0;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 1;
        else if (gpio.pin_mask_ == GPIO_PIN_2)
            channel = 2;
        else if (gpio.pin_mask_ == GPIO_PIN_3)
            channel = 3;
        else if (gpio.pin_mask_ == GPIO_PIN_4)
            channel = 4;
        else if (gpio.pin_mask_ == GPIO_PIN_5)
            channel = 5;
        else if (gpio.pin_mask_ == GPIO_PIN_6)
            channel = 6;
        else if (gpio.pin_mask_ == GPIO_PIN_7)
            channel = 7;
    } else if (gpio.port_ == GPIOB) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 8;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 9;
    } else if (gpio.port_ == GPIOC) {
        if (gpio.pin_mask_ == GPIO_PIN_0)
            channel = 10;
        else if (gpio.pin_mask_ == GPIO_PIN_1)
            channel = 11;
        else if (gpio.pin_mask_ == GPIO_PIN_2)
            channel = 12;
        else if (gpio.pin_mask_ == GPIO_PIN_3)
            channel = 13;
        else if (gpio.pin_mask_ == GPIO_PIN_4)
            channel = 14;
        else if (gpio.pin_mask_ == GPIO_PIN_5)
            channel = 15;
    }
    return channel;
}

// @brief Given an adc channel return the voltage as a ratio of adc_ref_voltage
// returns -1.0f if the channel is not valid.
float get_adc_relative_voltage_ch(uint16_t channel) {
    if (channel < ADC_CHANNEL_COUNT)
        return (float)adc_measurements_[channel] / adc_full_scale;
    else
        return -1.0f;
}

//--------------------------------
// IRQ Callbacks
//--------------------------------

void vbus_sense_adc_cb(uint32_t adc_value) {
    constexpr float voltage_scale = adc_ref_voltage * VBUS_S_DIVIDER_RATIO / adc_full_scale;
    vbus_voltage = adc_value * voltage_scale;
}

/* Analog speed control input */

static void update_analog_endpoint(const struct PWMMapping_t *map, int gpio)
{
    float fraction = get_adc_voltage(get_gpio(gpio)) / 3.3f;
    float value = map->min + (fraction * (map->max - map->min));
    fibre::set_endpoint_from_float(map->endpoint, value);
}

static void analog_polling_thread(void *)
{
    while (true) {
        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &odrv.config_.analog_mappings[i];

            if (fibre::is_endpoint_ref_valid(map->endpoint))
                update_analog_endpoint(map, i);
        }
        osDelay(10);
    }
}

void start_analog_thread() {
    osThreadDef(thread_def, analog_polling_thread, osPriorityLow, 0, 512 / sizeof(StackType_t));
    osThreadCreate(osThread(thread_def), NULL);
}
