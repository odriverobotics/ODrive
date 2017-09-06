/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

/* Includes ------------------------------------------------------------------*/
#include "drv8301.h"
#include <configuration.h>
#include <constants.h>

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

//@TODO move motor thread to high level file
void motor_thread();

#ifdef __cplusplus
}
#endif

#endif  //__LOW_LEVEL_H