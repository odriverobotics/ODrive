/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "low_level.h"

#define CAN_PRIVATE_M0p	0x320U
#define CAN_PRIVATE_M0v 0x321U
#define CAN_PRIVATE_M0c 0x322U
#define CAN_PRIVATE_M1p 0x323U
#define CAN_PRIVATE_M1v 0x324U
#define CAN_PRIVATE_M1c 0x325U

static CAN_PrivateConfigDef cancfg = {
  // This provides the sub-IDs: 0x320 - 0x327
  .filter_id = 0x320U,

  .filter_mask = 0x1FFFFFF8U,

  .tx_id = 0x328U

};

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_6TQ;
  hcan1.Init.BS2 = CAN_BS2_5TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = ENABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */
    CAN_FilterConfTypeDef  sFilterConfig;
    static CanTxMsgTypeDef TxMessage;
    static CanRxMsgTypeDef RxMessage;

    // Set CAN Rx/Tx message buffers
    canHandle->pTxMsg = &TxMessage;
    canHandle->pRxMsg = &RxMessage;

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

    // Rx message IDs filter common settings
    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.BankNumber = 14;

    // Rx message Standard IDs filter
    sFilterConfig.FilterIdHigh = (cancfg.filter_id << 5) & 0xFFFF; // STDID[10:0]
    sFilterConfig.FilterIdLow = 0x0;
    sFilterConfig.FilterMaskIdHigh = (cancfg.filter_mask << 5) & 0xFFFF;
    sFilterConfig.FilterMaskIdLow = 0x0;
    sFilterConfig.FilterActivation = ENABLE;

    if(HAL_CAN_ConfigFilter(canHandle, &sFilterConfig) != HAL_OK)
    {
      /* Filter configuration Error */
      Error_Handler();
    }

    // Rx message Extended IDs filter
    sFilterConfig.FilterNumber = 1;
    sFilterConfig.FilterIdHigh = (cancfg.filter_id >> 13) & 0xFFFF; // EXTID[28:13]
    sFilterConfig.FilterIdLow = ((cancfg.filter_id << 3) | CAN_ID_EXT) & 0xFFFF; // EXTID[12:0]
    sFilterConfig.FilterMaskIdHigh = (cancfg.filter_mask >> 13) & 0xFFFF;
    sFilterConfig.FilterMaskIdLow = ((cancfg.filter_mask << 3) | CAN_ID_EXT) & 0xFFFF;

    if(HAL_CAN_ConfigFilter(canHandle, &sFilterConfig) != HAL_OK)
    {
      /* Filter configuration Error */
      Error_Handler();
    }

    //Set up the CAN Receive buffer and enable it
    HAL_CAN_Receive_IT(canHandle, CAN_FIFO0);

    // Tx message IDs (unused at the moment)
    canHandle->pTxMsg->StdId = cancfg.tx_id;
    canHandle->pTxMsg->ExtId = cancfg.tx_id;
    canHandle->pTxMsg->RTR = CAN_RTR_DATA;
    canHandle->pTxMsg->IDE = CAN_ID_EXT;
    canHandle->pTxMsg->DLC = 1;

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  Retrieval complete callback in non blocking mode 
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* canHandle)
{
  float position, velocity, current;
  uint32_t CANID, motorID;
  char ctrlMode;

  if (canHandle->pRxMsg->IDE == CAN_ID_STD) {
    CANID = canHandle->pRxMsg->StdId;
  } else {
    CANID = canHandle->pRxMsg->ExtId;
  }

  // Retrieve the motor mode from the CAN ID
  switch (CANID)
  {
  case CAN_PRIVATE_M0p:
  case CAN_PRIVATE_M1p:
    ctrlMode = 'p';
    break;
  case CAN_PRIVATE_M0v:
  case CAN_PRIVATE_M1v:
    ctrlMode = 'v';
    break;
  case CAN_PRIVATE_M0c:
  case CAN_PRIVATE_M1c:
    ctrlMode = 'c';
    break;
  default:
    goto usid_recv_it;
    break;
  }

  // Retrieve the motor ID from the CAN ID
  switch (CANID)
  {
  case CAN_PRIVATE_M0p:
  case CAN_PRIVATE_M0v:
  case CAN_PRIVATE_M0c:
    motorID = 0;
    break;
  case CAN_PRIVATE_M1p:
  case CAN_PRIVATE_M1v:
  case CAN_PRIVATE_M1c:
    motorID = 1;
    break;
  default:
    goto usid_recv_it;
    break;
  }

  // Parse retrieved data in accordance with the control mode
  if (ctrlMode == 'p' && canHandle->pRxMsg->DLC == 8)
  {
    position = (float)(canHandle->pRxMsg->Data[2] | (uint32_t)(canHandle->pRxMsg->Data[1] << 8) | 
      (uint32_t)(canHandle->pRxMsg->Data[0] << 16)) / 100.0f;
    velocity = (float)(canHandle->pRxMsg->Data[5] | (uint32_t)(canHandle->pRxMsg->Data[4] << 8) | 
      (uint32_t)(canHandle->pRxMsg->Data[3] << 16)) / 100.0f;
    current = (float)(canHandle->pRxMsg->Data[7] | (uint32_t)(canHandle->pRxMsg->Data[6] << 8)) / 100.0f;

    //printf("CAN p %lu %f %f %f\n", motorID, position, velocity, current);
    set_pos_setpoint(&motors[motorID], position, velocity, current);
  }
  else if (ctrlMode == 'v' && canHandle->pRxMsg->DLC >= 5)
  {
    velocity = (float)(canHandle->pRxMsg->Data[2] | (uint32_t)(canHandle->pRxMsg->Data[1] << 8) | 
      (uint32_t)(canHandle->pRxMsg->Data[0] << 16)) / 100.0f;
    current = (float)(canHandle->pRxMsg->Data[4] | (uint32_t)(canHandle->pRxMsg->Data[3] << 8)) / 100.0f;

    //printf("CAN v %lu %f %f\n", motorID, velocity, current);
    set_vel_setpoint(&motors[motorID], velocity, current);
  }
  else if (ctrlMode == 'c' && canHandle->pRxMsg->DLC >= 2)
  {
    current = (float)(canHandle->pRxMsg->Data[1] | (uint32_t)(canHandle->pRxMsg->Data[0] << 8)) / 100.0f;

    //printf("CAN c %lu %f\n", motorID, current);
    set_current_setpoint(&motors[motorID], current);
  }

usid_recv_it:
  if(HAL_CAN_Receive_IT(canHandle, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
