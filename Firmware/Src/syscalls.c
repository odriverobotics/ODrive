/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include <cmsis_os.h>
#include <freertos_vars.h>
#include <sys/unistd.h>
#include <usart.h>
#include <usbd_cdc_if.h>
#include <legacy_commands.h> // TODO: make serial_printf_select constant


//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

#define UART_TX_BUFFER_SIZE 64
static uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE];

int _write(int file, char* data, int len) {
    //number of bytes written
    int written = 0;
    switch (serial_printf_select) {
        case SERIAL_PRINTF_IS_USB: {
            // Wait on semaphore for the interface to be available
            // Note that the USB driver will release the interface again when the TX completes
            const uint32_t usb_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_usb_tx, usb_tx_timeout);
            if (sem_stat == osOK) {
                uint8_t status = CDC_Transmit_FS((uint8_t*)data, len);  // transmit over CDC
                written = (status == USBD_OK) ? len : 0;
            } // If the semaphore times out, we simply leave "written" as 0
        } break;

        case SERIAL_PRINTF_IS_UART: {
            //Check length
            if (len > UART_TX_BUFFER_SIZE)
                return 0;
            // Wait on semaphore for the interface to be available
            // Note that HAL_UART_TxCpltCallback will release the interface again when the TX completes
            const uint32_t uart_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_uart_dma, uart_tx_timeout);
            if (sem_stat == osOK) {
                memcpy(uart_tx_buf, data, len);                    // memcpy data into uart_tx_buf
                HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, len);  // Start DMA background transfer
            } // If the semaphore times out, we simply leave "written" as 0
        } break;

        default: {
            written = 0;
        } break;
    }

    return written;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    osSemaphoreRelease(sem_uart_dma);
}
