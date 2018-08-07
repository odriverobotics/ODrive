#include "odrive_main.h"

// Standard sign function, implemented to match the Python impelmentation
template <typename T> int sign(T val){
    if(val == T(0))
        return T(0);
    else
        return (std::signbit(val)) ? -1 : 1;
}

float TrapezoidalTrajectory::planTrapezoidal(float Xf, float Xi,
                                            float Vf, float Vi,
                                            float Af, float Ai,
                                            float Vmax, float Amax, float Dmax) {

    float dX = Xf - Xi;
    int s = sign(dX);

    float Ar = s*Amax;  // Maximum Acceleration (signed)
    float Dr = -1.0f*s*Dmax; // Maximum Deceleration (signed)
    float Vr = s*Vmax;  // Maximum Velocity (signed)

    // Handle the case where initial velocity > Max velocity
    if(s*Vi > s*Vr){
        Ar = -1.0f*s*Amax;
    }

    float Ta = (Vr - Vi)/Ar;
    float Tv;
    float Td = (Vf - Vr)/Dr;

    // Peak Velocity handling
    float dXmin;
    if(Vf == 0.0f){
        dXmin = Ta*(Vr + Vi)/2.0f + (Td*Vr)/2.0f; // Basic wedge profile
    } else if(sign(Vf) == sign(Vr)){
        dXmin = Ta*(Vr + Vi)/2.0f + Td*(Vr - Vf)/2.0f + Td*Vf;    // Wedge profile with a non-zero end
    } else {
        dXmin = Ta*(Vr + Vi)/2.0f + Td*(Vr - s*Vf)/2.0f;  // Wedge profile that crosses Y axis during decel
    }

    // Short move handling
    if(s*dXmin > s*dX){
        Vr = s*std::sqrt(-1.0f*Ar*(Vf*Vf-2*Dr*dX))/sqrt(Dr-Ar);
        Ta = std::max(0.0f, (Vr - Vi)/Ar);
        Tv = 0;
        Td = std::max(0.0f, (Vf - Vr)/Dr);
    } else {
        Tv = (dX - dXmin)/Vr;
    }

    // Populate object's values
    yAccel_ = (Ar*Ta*Ta) / 2.0f + (Vi * Ta) + Xi;
    Xi_ = Xi;
    Vi_ = Vi;
    Ai_ = Ai;

    Ar_ = Ar;
    Dr_ = Dr;
    Vr_ = Vr;

    Ta_ = Ta;
    Tv_ = Tv;
    Td_ = Td;
    Tav_ = Ta + Tv;

    return Ta + Tv + Td;
}

TrapezoidalTrajectory::TrajectoryStep_t TrapezoidalTrajectory::evalTrapTraj(float t){
    TrapezoidalTrajectory::TrajectoryStep_t trajStep;
    if( t <= 0.0f){ // Initial Conditions
        trajStep.Y = Xi_;
        trajStep.Yd = Vi_;
        trajStep.Ydd = Ai_;
    } else if( t <= Ta_){   // Accelerating
        trajStep.Y = (Ar_ * (t*t))/2.0f + (Vi_ * t) + Xi_;
        trajStep.Yd = (Ar_ * t) + Vi_;
        trajStep.Ydd = Ar_;
    } else if( t <= Ta_ + Tv_){ // Coasting
        trajStep.Y = yAccel_ + (Vr_ * (t - Ta_));
        trajStep.Yd = Vr_;
        trajStep.Ydd = 0;
    } else if( t <= Ta_ + Tv_ + Td_){   // Deceleration
        float Tdc = t - Ta_;
        trajStep.Y = yAccel_ + (Vr_ * (Tdc)) + Dr_*(Tdc*Tdc)/2.0f;
        trajStep.Yd = Vr_ + Dr_*Tdc;
        trajStep.Ydd = Dr_;       
    }
}