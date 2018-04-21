/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphores
extern osSemaphoreId sem_usb_irq;
extern osSemaphoreId sem_uart_dma;
extern osSemaphoreId sem_usb_rx;
extern osSemaphoreId sem_usb_tx;

#endif /* __FREERTOS_H */