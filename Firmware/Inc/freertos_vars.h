/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphores
osSemaphoreId sem_usb_irq;
osSemaphoreId sem_uart_dma;
osSemaphoreId sem_usb_rx;
osSemaphoreId sem_usb_tx;

// List of threads
osThreadId thread_motor_0;
osThreadId thread_motor_1;
osThreadId thread_cmd_parse;
osThreadId thread_usb_pump;

#endif /* __FREERTOS_H */