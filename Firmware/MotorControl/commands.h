#ifndef COMMANDS_H
#define COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include <low_level.h>
/* Exported types ------------------------------------------------------------*/

typedef enum {
    GPIO_MODE_UART,
    GPIO_MODE_STEP_DIR,
} GpioMode_t;

typedef enum {
    SERIAL_PRINTF_IS_NONE,
    SERIAL_PRINTF_IS_USB,
    SERIAL_PRINTF_IS_UART,
} SerialPrintf_t;

typedef enum {
    CMD_USB_EVENT = 0x1,
    CMD_CAN_EVENT = 0x2
} CmdEvent_t;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SerialPrintf_t serial_printf_select;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void init_communication();
void cmd_parse_thread(void const * argument);
void motor_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface);

#endif /* COMMANDS_H */
