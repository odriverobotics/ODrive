/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include <sys/unistd.h>
#include <board.h>


//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

extern char _end; // provided by the linker script: it's end of statically allocated section, which is where the heap starts.
extern char _heap_end_max; // provided by the linker script
void* _end_ptr = &_end;
void* _heap_end_max_ptr = &_heap_end_max;
void* heap_end_ptr = 0;

/* @brief Increments the program break (aka heap end)
*
* This is called internally by malloc once it runs out
* of heap space. Malloc might expect a contiguous heap,
* so we don't call the FreeRTOS pvPortMalloc here.
* If this function returns -1, malloc will return NULL.
* Note that if this function returns NULL, malloc does not
* consider this as an error and will return the pointer 0x8.
*
* You should still be careful with using malloc though,
* as it does not guarantee thread safety.
*
* @return A pointer to the newly allocated block on success
*         or -1 otherwise.
*/
intptr_t _sbrk(size_t size) {
    intptr_t ptr;
	{
        uint32_t mask = cpu_enter_critical();
        if (!heap_end_ptr)
            heap_end_ptr = _end_ptr;
        if (heap_end_ptr + size > _heap_end_max_ptr) {
            ptr = -1;
        } else {
            ptr = (intptr_t)heap_end_ptr;
            heap_end_ptr += size;
        }
        cpu_exit_critical(mask);
	}
    return ptr;
}

// _write is defined in communication.cpp


