/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// TODO: this header is weird. Move these declarations to somewhere else.

// List of semaphores
extern osSemaphoreId sem_usb_irq;
extern osMessageQId uart_event_queue;
extern osMessageQId usb_event_queue;
extern osSemaphoreId sem_can;

extern osThreadId defaultTaskHandle;
extern const uint32_t stack_size_default_task;

#endif /* __FREERTOS_H */