/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphores
extern osSemaphoreId sem_usb_irq;
extern osSemaphoreId sem_uart_dma;
extern osSemaphoreId sem_usb_rx;
extern osSemaphoreId sem_usb_tx;

// List of threads
extern osThreadId thread_motor_0;
extern osThreadId thread_motor_1;
extern osThreadId thread_cmd_parse;
extern osThreadId thread_usb_pump;

#endif /* __FREERTOS_H */