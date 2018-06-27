#include "motionPlanning.h"
#include "bbpr.h"

volatile TrpzInfo trpz_info = {};

float getSetPointTrapezoid(float time) {
    if (time >= 0 && time <= trpz_info.t1) {
        return time*time * trpz_info.acc/2;
    } else if (time >= trpz_info.t1 && time <= trpz_info.t2) {
        return trpz_info.d1 + (time - trpz_info.t1) * trpz_info.v_max;
    } else if (trpz_info.t2 <= time && time <= trpz_info.t3) {
        volatile float dt = time - trpz_info.t2;
        return trpz_info.d2 + trpz_info.v_max * dt - dt*dt * trpz_info.acc/2;
    } else {
        return trpz_info.d3;
    }
}


void computePeriodsTrapezoid(float Xpeak[], float T[]) {
    int n = 0;
    float coeffs[3] = {0, 0, 0};
    float Xmax = 0;

    n = 2;
    coeffs[0] = Xpeak[2];
    coeffs[1] = T[1] * Xpeak[2];
    coeffs[2] = -Xpeak[0];
    T[2] = findRoot(coeffs, n);

    Xmax = T[2] * Xpeak[2];

    if (Xmax > Xpeak[1]) {
        n = 1;
        coeffs[0] = Xpeak[2];
        coeffs[1] = -Xpeak[1];
        T[2] = findRoot(coeffs, n);
    } else {
        Xpeak[1] = Xmax;
    }

    n = 1;
    coeffs[0] = T[2] * Xpeak[2];
    coeffs[1] = pow(T[2], 2) * Xpeak[2] - Xpeak[0];
    T[1] = findRoot(coeffs, n);
}
