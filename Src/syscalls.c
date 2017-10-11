/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include  <sys/unistd.h>
#include <usbd_cdc_if.h>
#include <usart.h>
#include <commands.h>


//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

#define UART_TX_BUFFER_SIZE 64
static uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE];

int _write(int file, char *data, int len) {
  //number of bytes written
  int written = 0;
  switch (serial_printf_select) {

    case SERIAL_PRINTF_IS_USB: {
      // transmit over CDC
      uint8_t status = CDC_Transmit_FS((uint8_t*)data, len);
      written = (status == USBD_OK) ? len : 0;
    } break;

    case SERIAL_PRINTF_IS_UART: {
      //Check length
      if (len > UART_TX_BUFFER_SIZE)
        return 0;
      // Check if transfer is already ongoing
      // TODO: use HAL_UART_GetState
      if(huart4.gState != HAL_UART_STATE_READY)
        return 0;
      // memcpy data into uart_tx_buf
      memcpy(uart_tx_buf, data, len);
      // Start DMA background trasnfer
      HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, len);
    } break;

    default: {
      written = 0;
    } break;
  }

  return written;
}
