#ifndef LEGACY_COMMANDS_H
#define LEGACY_COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "low_level.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// Exposed comms table during refactor transition
extern float* exposed_floats[];
extern int* exposed_ints[];
extern bool* exposed_bools[];
extern uint16_t* exposed_uint16[];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void legacy_parse_cmd(const uint8_t* buffer, size_t len, size_t buffer_length, int out_file);
void legacy_parse_stream(const uint8_t* buffer, size_t len, int out_file);

#ifdef __cplusplus
}
#endif

#endif /* LEGACY_COMMANDS_H */
