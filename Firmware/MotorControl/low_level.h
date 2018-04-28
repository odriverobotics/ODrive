/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <stdbool.h>
#include <adc.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern float vbus_voltage;
extern bool brake_resistor_armed;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void safety_critical_arm_motor_pwm(Motor& motor);
bool safety_critical_disarm_motor_pwm(Motor& motor);
void safety_critical_apply_motor_pwm_timings(Motor& motor, uint16_t timings[3]);
void safety_critical_arm_brake_resistor();
void safety_critical_disarm_brake_resistor();
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on);

// called from STM platform code
extern "C" {
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected);
void tim_update_cb(TIM_HandleTypeDef* htim);
}

// Initalisation
void start_adc_pwm();
void start_pwm(TIM_HandleTypeDef* htim);
void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);
void start_general_purpose_adc();

float get_adc_voltage(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);

void update_brake_current();

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
