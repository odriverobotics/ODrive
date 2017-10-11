#ifndef COMMANDS_H
#define COMMANDS_H

#include <low_level.h>

/* Exported functions --------------------------------------------------------*/
void cmd_parse_thread(void const * argument);
void motor_parse_cmd(uint8_t* buffer, int len);

#endif /* COMMANDS_H */
