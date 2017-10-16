#ifndef COMMANDS_H
#define COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include "low_level.h"
/* Exported types ------------------------------------------------------------*/

typedef enum {
    SERIAL_PRINTF_IS_NONE,
    SERIAL_PRINTF_IS_USB,
    SERIAL_PRINTF_IS_UART,
} SerialPrintf_t;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SerialPrintf_t serial_printf_select;
// Exposed comms table during refactor transition
extern float* exposed_floats[];
extern int* exposed_ints[];
extern bool* exposed_bools[];
extern uint16_t* exposed_uint16[];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void legacy_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface);

#endif /* COMMANDS_H */
