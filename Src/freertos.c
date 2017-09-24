/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "freertos_vars.h"
#include "low_level.h"
#include "usart.h"
#include "version.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void usb_cmd_thread(void const * argument);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // Init usb irq binary semaphore, and start with no tolkens by removing the starting one.
  osSemaphoreDef(sem_usb_irq);
  sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
  osSemaphoreWait(sem_usb_irq, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */

  // Init motor control
  init_motor_control();

  // Start motor threads
  osThreadDef(task_motor_0, motor_thread,   osPriorityHigh+1, 0, 512);
  osThreadDef(task_motor_1, motor_thread,   osPriorityHigh,   0, 512);
  thread_motor_0 = osThreadCreate(osThread(task_motor_0), &motors[0]);
  thread_motor_1 = osThreadCreate(osThread(task_motor_1), &motors[1]);

  // Start USB command handling thread
  osThreadDef(task_usb_cmd, usb_cmd_thread, osPriorityNormal, 0, 512);
  thread_usb_cmd = osThreadCreate(osThread(task_usb_cmd), NULL);

  //If we get to here, then the default task is done.
  vTaskDelete(defaultTaskHandle);

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

//TODO move this to a different file
// Thread to handle deffered processing of USB interrupt
void usb_cmd_thread(void const * argument) {

  //DMA open loop continous circular buffer
  //1ms delay periodic, chase DMA ptr around, on new data:
    // Check for start char
    // copy into parse-buffer
    // check for end-char
    // checksum, etc.

  #define UART_BUFFER_SIZE 64
  static uint8_t dma_circ_buffer[UART_BUFFER_SIZE];
  static uint8_t parse_buffer[UART_BUFFER_SIZE];

  // DMA is set up to recieve in a circular buffer forever.
  // We dont use interrupts to fetch the data, instead we periodically read
  // data out of the circular buffer into a parse buffer, controlled by a state machine
  HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));

  uint32_t last_rcv_idx = UART_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
  // Re-run state-machine forever
  for (;;) {
    //Inialize recieve state machine
    bool reset_read_state = false;
    bool read_active = false;
    uint32_t parse_buffer_idx = 0;
    //Run state machine until reset
    do {
      // Fetch the circular buffer "write pointer", where it would write next
      uint32_t rcv_idx = UART_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
      // During sleeping, we may have fallen several characters behind, so we keep
      // going until we are caught up, before we sleep again
      while (rcv_idx != last_rcv_idx) {
        // Fetch the next char, rotate read ptr
        uint8_t c = dma_circ_buffer[last_rcv_idx];
        if (++last_rcv_idx == UART_BUFFER_SIZE)
          last_rcv_idx = 0;
        // Look for start character
        if (c == '$') {
          read_active = true;
          continue; // do not record start char
        }
        // Record into parse buffer when actively reading
        if (read_active) {
          parse_buffer[parse_buffer_idx++] = c;
          if (c == '\r' || c == '\n' || c == "!") {
            // End of command string: exchange end char with terminating null
            parse_buffer[parse_buffer_idx-1] = '\0';
            motor_parse_cmd(parse_buffer, parse_buffer_idx);
            // Reset receieve state machine
            reset_read_state = true;
            break;
          } else if (parse_buffer_idx == UART_BUFFER_SIZE - 1) {
            // We are not at end of command, and receiving another character after this
            // would go into the last slot, which is reserved for terminating null.
            // We have effectively overflowed parse buffer: abort.
            reset_read_state = true;
            break;
          }
        }
      }
      // When we reach here, we are out of immediate characters to fetch out of buffer
      // So we sleep for a bit.
      osDelay(1);
    } while (!reset_read_state);
  }

  for (;;) {
    // Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
    osSemaphoreWait(sem_usb_irq, osWaitForever);
    // Irq processing loop
    //while(HAL_NVIC_GetActive(OTG_FS_IRQn)) {
      HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    //}
    // Let the irq (OTG_FS_IRQHandler) fire again.
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }

  // If we get here, then this task is done
  vTaskDelete(osThreadGetId());
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
