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
extern uint16_t adc_measurements_[ADC_CHANNEL_COUNT];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

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

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
