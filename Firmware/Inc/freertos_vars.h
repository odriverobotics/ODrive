/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphore
osSemaphoreId sem_usb_irq;

// List of threads
osThreadId thread_motor_0;
osThreadId thread_motor_1;
osThreadId thread_cmd_parse;

#endif /* __FREERTOS_H */