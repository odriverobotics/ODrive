#pragma once

#include <utils.h>

#define ONE_BY_SQRT3                (0.57735026919f)
#define sqrt3_by_2                  (0.86602540378f)

#define STANDALONE_MODE // Drive operates without USB communication
#define PH_CURRENT_MEAS_TIMEOUT     (2) // [ms]
#define ENCODER_CPR                 (600*4)
#define POLE_PAIRS                  (7)

float get_current_meas_period();
float get_current_meas_hz();

extern float vbus_voltage;
extern float elec_rad_per_enc;
extern float brake_resistance;
