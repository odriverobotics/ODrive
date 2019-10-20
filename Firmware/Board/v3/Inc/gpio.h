/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

void SetGPIO12toUART();
bool GPIO_subscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin,
    uint32_t pull_up_down,
    void (*callback)(void*), void* ctx);
void GPIO_unsubscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);
void GPIO_set_to_analog(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);

uint16_t get_gpio_pin_by_pin(uint16_t GPIO_pin);
GPIO_TypeDef* get_gpio_port_by_pin(uint16_t GPIO_pin);

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR <= 4
#define GPIO_COUNT  5
#else
#define GPIO_COUNT  8
#endif

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
