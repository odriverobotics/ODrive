#include <math.h>
#include "odrive_main.h"

// Standard sign function, implemented to match the Python impelmentation
template <typename T>
int sign(T val) {
    if (val == T(0))
        return T(0);
    else
        return (std::signbit(val)) ? -1 : 1;
}

TrapezoidalTrajectory::TrapezoidalTrajectory(TrapTrajConfig_t &config) : config_(config) {}

float TrapezoidalTrajectory::planTrapezoidal(float Xf, float Xi,
                                             float Vi, float Vmax,
                                             float Amax, float Dmax) {
    float dx_stop = (Vi * Vi) / (Dmax * 2.0f);
    float dX = Xf - Xi;
    int s = sign(dX);

    float Ar = s * Amax;          // Maximum Acceleration (signed)
    float Dr = -1.0f * s * Dmax;  // Maximum Deceleration (signed)
    float Vr = s * Vmax;          // Maximum Velocity (signed)

    float Ta;
    float Tv;
    float Td;

    // Checking for overshoot on "minimum stop"
    if (fabs(dX) <= dx_stop) {
        Ta = 0;
        Tv = 0;
        Vr = Vi;
        Dr = -1.0f * sign(Vi) * Dmax;
        Td = fabs(Vi) / Dmax;
    } else {
        // Handle the case where initial velocity > Max velocity
        if ((s * Vi) > (s * Vr)) {
            Ar = -1.0f * s * Amax;
        }

        Ta = (Vr - Vi) / Ar;  // Acceleration time
        Td = (-Vr) / Dr;      // Deceleration time

        // Peak Velocity handling
        float dXmin = Ta * (Vr + Vi) / 2.0f + Td * Vr / 2.0f;

        // Short move handling
        if (fabs(dX) < fabs(dXmin)) {
            Vr = s * sqrt((-((Vi * Vi) / Ar) - 2.0f * dX) / (1.0f / Dr - 1.0f / Ar));
            Ta = std::max(0.0f, (Vr - Vi) / Ar);
            Tv = 0;
            Td = std::max(0.0f, -Vr / Dr);
        } else {
            Tv = (dX - dXmin) / Vr;
        }
    }

    // Populate object's values

    Xf_ = Xf;
    Xi_ = Xi;
    Vi_ = Vi;

    Ar_ = Ar;
    Dr_ = Dr;
    Vr_ = Vr;

    Ta_ = Ta;
    Tv_ = Tv;
    Td_ = Td;

    yAccel_ = (Ar * Ta * Ta) / 2.0f + (Vi * Ta) + Xi;
    Tav_ = Ta + Tv;

    return Ta + Tv + Td;
}

TrapTrajStep_t TrapezoidalTrajectory::evalTrapTraj(float t) {
    TrapTrajStep_t trajStep;
    if (t < 0.0f) {  // Initial Conditions
        trajStep.Y = Xi_;
        trajStep.Yd = Vi_;
        trajStep.Ydd = Ar_;
    } else if (t < Ta_) {  // Accelerating
        trajStep.Y = (Ar_ * (t * t) / 2.0f) + (Vi_ * t) + Xi_;
        trajStep.Yd = (Ar_ * t) + Vi_;
        trajStep.Ydd = Ar_;
    } else if (t < Ta_ + Tv_) {  // Coasting
        trajStep.Y = yAccel_ + (Vr_ * (t - Ta_));
        trajStep.Yd = Vr_;
        trajStep.Ydd = 0;
    } else if (t < Ta_ + Tv_ + Td_) {  // Deceleration
        float Tdc = t - Tav_;
        trajStep.Y = yAccel_ + (Vr_ * (t - Ta_)) + Dr_ * (Tdc * Tdc) / 2.0f;
        trajStep.Yd = Vr_ + Dr_ * Tdc;
        trajStep.Ydd = Dr_;
    }

    return trajStep;
}