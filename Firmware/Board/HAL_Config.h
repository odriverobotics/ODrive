#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#if HW_VERSION_MAJOR == 3
    #include "Board/v3/HAL_Board_V3.x.h"
#else
    #error "unknown board version"
#endif

#include "HAL_Generics/HAL_Structs.h"

extern const float thermistor_poly_coeffs[];
extern const BoardHardwareConfig_t hw_configs[];
extern const size_t thermistor_num_coeffs;

#endif //BOARD_CONFIG_H
