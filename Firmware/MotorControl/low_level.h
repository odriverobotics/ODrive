/* MIT License

Copyright (c) 2016-2021 ODrive Robotics, Inc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <stdbool.h>
#include <adc.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ADC_CHANNEL_COUNT 16
extern const float adc_full_scale;
extern const float adc_ref_voltage;
/* Exported variables --------------------------------------------------------*/
extern float vbus_voltage;
extern float ibus_;
extern bool brake_resistor_armed;
extern bool brake_resistor_saturated;
extern uint16_t adc_measurements_[ADC_CHANNEL_COUNT];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void safety_critical_arm_brake_resistor();
void safety_critical_disarm_brake_resistor();
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on);

// called from STM platform code
extern "C" {
void vbus_sense_adc_cb(uint32_t adc_value);
void pwm_in_cb(TIM_HandleTypeDef *htim);
}

// Initalisation
void start_adc_pwm();
void start_pwm(TIM_HandleTypeDef* htim);
void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset,
                 TIM_HandleTypeDef* htim_refbase = nullptr);
void start_general_purpose_adc();
void pwm_in_init();
void start_analog_thread();

// ADC getters
uint16_t channel_from_gpio(Stm32Gpio gpio);
float get_adc_voltage(Stm32Gpio gpio);
float get_adc_relative_voltage(Stm32Gpio gpio);
float get_adc_relative_voltage_ch(uint16_t channel);

void update_brake_current();

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
