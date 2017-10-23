/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphores
extern osSemaphoreId sem_usb_irq;

// List of threads
extern osThreadId thread_motor_0;
extern osThreadId thread_motor_1;
extern osThreadId thread_cmd_parse;

#endif /* __FREERTOS_H */