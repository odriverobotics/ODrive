#pragma once

#include "drv8301.h"

#ifdef __cplusplus
extern "C" {
#endif

void step_cb(uint16_t GPIO_Pin);
void vbusSenseADC_cb(ADC_HandleTypeDef *hadc, bool injected);

#ifdef __cplusplus
}
#endif