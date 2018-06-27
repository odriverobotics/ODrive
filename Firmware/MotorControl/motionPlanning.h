#ifndef __MOTIONPLANNING_H
#define __MOTIONPLANNING_H

#include "odrive_main.h"
#include <math.h>

struct TrpzInfo {
    // from t0 (0) to t1, the velocity increases
    // from t1 to t2, the velocity is constant
    // from t2 to t3, the velocity decreases
    float v_max;
    float acc;
    float t1;
    float t2;
    float t3;
    float start_time;
    float d1;      // the total distance moved at t1
    float d2;      // the total distance moved at t2
    float d3;      // the total distance moved at t3
};

extern volatile TrpzInfo trpz_info;

void computePeriodsTrapezoid(float Xpeak[], float T[]);
float getSetPointTrapezoid(float time);
void computePeriodsTrapezoid(float distance, float v_max, float acc);

#endif
