/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include  <sys/unistd.h>
#include "usbd_cdc_if.h"

//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

int _write(int file, char *data, int len) {

  // transmit over CDC
  uint8_t status = CDC_Transmit_FS(data, len);

  // return number of bytes written
  return (status == USBD_OK ? len : 0);
}
