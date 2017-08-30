#pragma once

#include <cmsis_os.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    int type;
    int index;
} monitoring_slot;

void motor_parse_cmd(uint8_t *buffer, int len);
#ifdef __cplusplus
}
#endif