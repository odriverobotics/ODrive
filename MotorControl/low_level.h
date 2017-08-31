/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <usb_commands.h>
#include <configuration.h>
#include <constants.h>
#include <controller.h>
#include <motor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void init_motor_control();
void pwm_trig_adc_cb(ADC_HandleTypeDef *hadc, bool injected);
void control_motor_loop(Motor_t *motor);

//@TODO move motor thread to high level file
void motor_thread(int);

#ifdef __cplusplus
}
#endif

#endif  //__LOW_LEVEL_H