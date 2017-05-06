/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_VARS_H
#define __FREERTOS_VARS_H

#include "FreeRTOS.h"
#include "cmsis_os.h"


// List of semaphore
osSemaphoreId sem_usb_irq;

// List of threads
osThreadId thread_motor_0;
osThreadId thread_motor_1;
osThreadId thread_usb_cmd;

#endif /* __FREERTOS_VARS_H */
