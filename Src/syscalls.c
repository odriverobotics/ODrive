/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include  <sys/unistd.h>
#include "usbd_cdc_if.h"
#include <commands.h>

//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

//static char uart_tx_buf

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

    } break;
    default: {
      written = 0;
    } break;
  }

  return written;
}
