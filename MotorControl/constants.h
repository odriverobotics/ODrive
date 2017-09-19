#pragma once

#include <configuration.h>
#include <utils.h>

#define ONE_BY_SQRT3 (0.57735026919f)
#define sqrt3_by_2 (0.86602540378f)

float getCurrentMeasPeriod();
float getCurrentMeasHZ();

extern float vbus_voltage;
extern float elec_rad_per_enc;
extern float brake_resistance;
