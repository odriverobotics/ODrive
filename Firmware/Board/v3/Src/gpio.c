/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include <stdbool.h>

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 1 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 2
#include "prev_board_ver/gpio_V3_2.c"
#elif HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 3 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 4
#include "prev_board_ver/gpio_V3_4.c"
#else
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M0_nCS_Pin|M1_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = M0_nCS_Pin|M1_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = M1_ENC_Z_Pin|GPIO_5_Pin|M0_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = GPIO_6_Pin|GPIO_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
#endif // End GPIO Include

// @brief Returns the IRQ number associated with a certain pin.
// Note that all GPIOs with the same pin number map to the same IRQn,
// no matter which port they belong to.
IRQn_Type get_irq_number(uint16_t pin) {
  uint16_t pin_number = 0;
  pin >>= 1;
  while (pin) {
    pin >>= 1;
    pin_number++;
  }
  switch (pin_number) {
    case 0: return EXTI0_IRQn;
    case 1: return EXTI1_IRQn;
    case 2: return EXTI2_IRQn;
    case 3: return EXTI3_IRQn;
    case 4: return EXTI4_IRQn;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9: return EXTI9_5_IRQn;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: return EXTI15_10_IRQn;
    default: return 0; // impossible
  }
}

// @brief Puts the GPIO's 1 and 2 into UART mode.
// This will disable any interrupt subscribers of these GPIOs.
void SetGPIO12toUART() {
  GPIO_InitTypeDef GPIO_InitStruct;

  // make sure nothing is hogging the GPIO's
  GPIO_unsubscribe(GPIO_1_GPIO_Port, GPIO_1_Pin);
  GPIO_unsubscribe(GPIO_2_GPIO_Port, GPIO_2_Pin);

  GPIO_InitStruct.Pin = GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIO_2_GPIO_Port, &GPIO_InitStruct);
}

// Expected subscriptions: 2x step signal + 2x encoder index signal
#define MAX_SUBSCRIPTIONS 10
struct subscription_t {
  GPIO_TypeDef* GPIO_port;
  uint16_t GPIO_pin;
  void (*callback)(void*);
  void* ctx;
} subscriptions[MAX_SUBSCRIPTIONS] = { 0 };
size_t n_subscriptions = 0;

// Sets up the specified GPIO to trigger the specified callback
// on a rising edge of the GPIO.
// @param pull_up_down: one of GPIO_NOPULL, GPIO_PULLUP or GPIO_PULLDOWN
bool GPIO_subscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin,
    uint32_t pull_up_down,
    void (*callback)(void*), void* ctx) {
  
  // Register handler (or reuse existing registration)
  // TODO: make thread safe
  struct subscription_t* subscription = NULL;
  for (size_t i = 0; i < n_subscriptions; ++i) {
    if (subscriptions[i].GPIO_port == GPIO_port &&
        subscriptions[i].GPIO_pin == GPIO_pin)
      subscription = &subscriptions[i];
  }
  if (!subscription) {
    if (n_subscriptions >= MAX_SUBSCRIPTIONS)
      return false;
    subscription = &subscriptions[n_subscriptions++];
  }

  *subscription = (struct subscription_t){
    .GPIO_port = GPIO_port,
    .GPIO_pin = GPIO_pin,
    .callback = callback,
    .ctx = ctx
  };

  // Set up GPIO
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = pull_up_down;
  HAL_GPIO_Init(GPIO_port, &GPIO_InitStruct);

  // Enable interrupt
  HAL_NVIC_SetPriority(get_irq_number(GPIO_pin), 0, 0);
  HAL_NVIC_EnableIRQ(get_irq_number(GPIO_pin));
  return true;
}

void GPIO_unsubscribe(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin) {
  bool is_pin_in_use = false;
  for (size_t i = 0; i < n_subscriptions; ++i) {
    if (subscriptions[i].GPIO_port == GPIO_port &&
        subscriptions[i].GPIO_pin == GPIO_pin) {
      subscriptions[i].callback = NULL;
      subscriptions[i].ctx = NULL;
    } else if (subscriptions[i].GPIO_pin == GPIO_pin) {
      is_pin_in_use = true;
    }
  }
  if (!is_pin_in_use)
    HAL_NVIC_DisableIRQ(get_irq_number(GPIO_pin));
}

// @brief Configures the specified GPIO as an analog input.
// This disables any subscriptions that were active for this pin.
void GPIO_set_to_analog(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_unsubscribe(GPIO_port, GPIO_pin);
  GPIO_InitStruct.Pin = GPIO_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_port, &GPIO_InitStruct);
}

//Dispatch processing of external interrupts based on source
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
  for (size_t i = 0; i < n_subscriptions; ++i) {
    if (subscriptions[i].GPIO_pin == GPIO_pin) // TODO: check for port
      if (subscriptions[i].callback)
        subscriptions[i].callback(subscriptions[i].ctx);
  }
}

GPIO_TypeDef* get_gpio_port_by_pin(uint16_t GPIO_pin){
  switch(GPIO_pin){
    case 1: return GPIO_1_GPIO_Port; break;
    case 2: return GPIO_2_GPIO_Port; break;
    case 3: return GPIO_3_GPIO_Port; break;
    case 4: return GPIO_4_GPIO_Port; break;
#ifdef GPIO_5_GPIO_Port
    case 5: return GPIO_5_GPIO_Port; break;
#endif
#ifdef GPIO_6_GPIO_Port
    case 6: return GPIO_6_GPIO_Port; break;
#endif
#ifdef GPIO_7_GPIO_Port
    case 7: return GPIO_7_GPIO_Port; break;
#endif
#ifdef GPIO_8_GPIO_Port
    case 8: return GPIO_8_GPIO_Port; break;
#endif
    default: return GPIO_1_GPIO_Port;
  }
}

uint16_t get_gpio_pin_by_pin(uint16_t GPIO_pin){
  switch(GPIO_pin){
    case 1: return GPIO_1_Pin; break;
    case 2: return GPIO_2_Pin; break;
    case 3: return GPIO_3_Pin; break;
    case 4: return GPIO_4_Pin; break;
#ifdef GPIO_5_Pin
    case 5: return GPIO_5_Pin; break;
#endif
#ifdef GPIO_6_Pin
    case 6: return GPIO_6_Pin; break;
#endif
#ifdef GPIO_7_Pin
    case 7: return GPIO_7_Pin; break;
#endif
#ifdef GPIO_8_Pin
    case 8: return GPIO_8_Pin; break;
#endif
    default: return GPIO_1_Pin;
  }
}

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
