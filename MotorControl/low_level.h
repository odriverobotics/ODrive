/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <motor.h>
#include <odrive_constants.h>
#include <odrive_configuration.h>
#include <odrive_commands.h>
#include <controller.h>
//default timeout waiting for phase measurement signals

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void init_motor_control();
void step_cb(uint16_t GPIO_Pin);
void pwm_trig_adc_cb(ADC_HandleTypeDef *hadc, bool injected);
void vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, bool injected);

void control_motor_loop(Motor_t *motor);

//@TODO move motor thread to high level file
void motor_thread(void const *argument);

#endif //__LOW_LEVEL_H