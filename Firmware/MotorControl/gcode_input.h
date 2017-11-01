/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GCODE_INPUT_H
#define __GCODE_INPUT_H

#include "commands.h"

// Null terminated line of gcode to be parsed and executed.   May block until gcode is executed.
void parse_gcode_line(unsigned char* line, SerialPrintf_t response_interface);

#endif