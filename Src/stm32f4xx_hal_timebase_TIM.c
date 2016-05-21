/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_TIM.c 
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup HAL_TimeBase
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim14; 
uint32_t                 uwIncrementState = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM14 as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
  * @param  TickPriority: Tick interrupt priorty.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIM14 IRQ priority */
  HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, TickPriority ,0); 
  
  /* Enable the TIM14 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn); 
  
  /* Enable TIM14 clock */
  __HAL_RCC_TIM14_CLK_ENABLE();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute TIM14 clock */
  uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
   
  /* Compute the prescaler value to have TIM14 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);
  
  /* Initialize TIM14 */
  htim14.Instance = TIM14;
  
  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM14CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim14.Init.Period = (1000000 / 1000) - 1;
  htim14.Init.Prescaler = uwPrescalerValue;
  htim14.Init.ClockDivision = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim14) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim14);
  }
  
  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM14 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM14 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim14, TIM_IT_UPDATE);                                                  
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM14 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM14 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_IncTick();
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
