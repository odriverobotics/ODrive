/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include <cmsis_os.h>
#include "utils.h"
#include "protocol.hpp"
#include "commands.h"


//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

extern "C" {
int _write(int file, const char* data, int len);
}

extern int abc;
int _write(int file, const char* data, int len) {
    //file = FILENO_UART;
    // resolve stdout and stderr
    if (abc == -1)
        abc = file;
    if (file == FILENO_STDOUT || file == FILENO_STDERR) {
        int written_bytes = 0;
#if defined(USB_PROTOCOL_STDOUT)
        written_bytes = _write(FILENO_USB, data, len);
#endif
#if defined(UART_PROTOCOL_STDOUT)
        written_bytes = _write(FILENO_UART, data, len);
#endif
        return written_bytes;
    }

    unsigned int ufile = static_cast<unsigned int>(file);
    if (ufile >= sizeof(file_table) / sizeof(file_table[0]))
        return 0;

    StreamSink *output = file_table[ufile];
    if (output == NULL)
        return 0;

    uint32_t t = push_timeout(100); // 100ms timeout for printf
    size_t bytes_written = 0;
    output->process_bytes(reinterpret_cast<const uint8_t *>(data), len, bytes_written);
    pop_timeout(t); // reset timeout
    return bytes_written;
}
