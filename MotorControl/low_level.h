/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <motor.h>

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

/* Exported types ------------------------------------------------------------*/
typedef struct{
        int type;
        int index;
} monitoring_slot;

/* Exported constants --------------------------------------------------------*/
extern float vbus_voltage;

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

//Note: to control without feed forward, set feed forward terms to 0.0f.
void set_pos_setpoint(Motor_t* motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward);
void set_vel_setpoint(Motor_t* motor, float vel_setpoint, float current_feed_forward);
void set_current_setpoint(Motor_t* motor, float current_setpoint);

void safe_assert(int arg);
void init_motor_control();
void step_cb(uint16_t GPIO_Pin);
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected);

//@TODO move motor thread to high level file
void motor_thread(void const * argument);

//@TODO move cmd parsing to high level file
void motor_parse_cmd(uint8_t* buffer, int len);

#endif //__LOW_LEVEL_H
