/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_VARS_H
#define __FREERTOS_VARS_H

// List of semaphore
extern osSemaphoreId sem_usb_irq;

// List of threads
extern osThreadId thread_motor_0;
extern osThreadId thread_motor_1;
extern osThreadId thread_usb_cmd;

#endif /* __FREERTOS_VARS_H */
