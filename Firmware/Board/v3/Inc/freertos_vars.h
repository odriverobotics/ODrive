/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphores
extern osSemaphoreId sem_usb_irq;
extern osSemaphoreId sem_uart_dma;
extern osSemaphoreId sem_usb_rx;
extern osSemaphoreId sem_usb_tx;
extern osSemaphoreId sem_can;

extern osThreadId defaultTaskHandle;
extern osThreadId usb_irq_thread;

#endif /* __FREERTOS_H */