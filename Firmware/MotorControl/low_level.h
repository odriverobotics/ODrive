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

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ADC_CHANNEL_COUNT 16
extern const float adc_full_scale;
extern const float adc_ref_voltage;
/* Exported variables --------------------------------------------------------*/
extern float vbus_voltage;
extern bool brake_resistor_armed;
extern uint16_t adc_measurements_[ADC_CHANNEL_COUNT];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void safety_critical_arm_motor_pwm(Motor& motor);
bool safety_critical_disarm_motor_pwm(Motor& motor);
void safety_critical_apply_motor_pwm_timings(Motor& motor, uint16_t period, uint16_t timings[3]);
void safety_critical_arm_brake_resistor();
void safety_critical_disarm_brake_resistor();
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on);


// Initalisation
//void start_adc_pwm();
//void start_pwm(TIM_HandleTypeDef* htim);
void start_general_purpose_adc();
void pwm_in_init();
void start_analog_thread();

void update_brake_current();

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
