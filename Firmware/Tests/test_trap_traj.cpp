
#include <doctest.h>
#include <limits.h>
#include <cmath>
#include <iostream>
#include <random>

#include "MotorControl/utils.hpp"

// TODO: This is currently a copy-paste of the real code due to non-trivial
// include dependencies. Should include real code.

class TrapezoidalTrajectory {
public:
    struct Step_t {
        float Y;
        float Yd;
        float Ydd;
    };

    explicit TrapezoidalTrajectory();
    bool planTrapezoidal(float Xf, float Xi, float Vi,
                         float Vmax, float Amax, float Dmax);
    Step_t eval(float t);

    float Xi_;
    float Xf_;
    float Vi_;

    float Ar_;
    float Vr_;
    float Dr_;

    float Ta_;
    float Tv_;
    float Td_;
    float Tf_;

    float yAccel_;

    float t_;
};



// A sign function where input 0 has positive sign (not 0)
float sign_hard(float val) {
    return (std::signbit(val)) ? -1.0f : 1.0f;
}

// Symbol                     Description
// Ta, Tv and Td              Duration of the stages of the AL profile
// Xi and Vi                  Adapted initial conditions for the AL profile
// Xf                         Position set-point
// s                          Direction (sign) of the trajectory
// Vmax, Amax, Dmax and jmax  Kinematic bounds
// Ar, Dr and Vr              Reached values of acceleration and velocity

TrapezoidalTrajectory::TrapezoidalTrajectory() {}

bool TrapezoidalTrajectory::planTrapezoidal(float Xf, float Xi, float Vi,
                                            float Vmax, float Amax, float Dmax) {
    float dX = Xf - Xi;  // Distance to travel
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
    float dXstop = std::copysign(stop_dist, Vi); // Minimum stopping displacement
    float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any)
    Ar_ = s * Amax;  // Maximum Acceleration (signed)
    Dr_ = -s * Dmax; // Maximum Deceleration (signed)
    Vr_ = s * Vmax;  // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel
    // aka "double deceleration move" in the paper
    if ((s * Vi) > (s * Vr_)) {
        Ar_ = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Ta_ = (Vr_ - Vi) / Ar_;
    Td_ = -Vr_ / Dr_;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f*Ta_*(Vr_ + Vi) + 0.5f*Td_*Vr_;

    // Are we displacing enough to reach cruising speed?
    if (s*dX < s*dXmin) {
        // Short move (triangle profile)
        Vr_ = s * std::sqrt(std::max((Dr_*SQ(Vi) + 2*Ar_*Dr_*dX) / (Dr_ - Ar_), 0.0f));
        //Vr_ = s * std::sqrt((Dr_*SQ(Vi) + 2*Ar_*Dr_*dX) / (Dr_ - Ar_));
        Ta_ = std::max(0.0f, (Vr_ - Vi) / Ar_);
        Td_ = std::max(0.0f, -Vr_ / Dr_);
        Tv_ = 0.0f;
    } else {
        // Long move (trapezoidal profile)
        Tv_ = (dX - dXmin) / Vr_;
    }

    // Fill in the rest of the values used at evaluation-time
    Tf_ = Ta_ + Tv_ + Td_;
    Xi_ = Xi;
    Xf_ = Xf;
    Vi_ = Vi;
    yAccel_ = Xi + Vi*Ta_ + 0.5f*Ar_*SQ(Ta_); // pos at end of accel phase

    return true;
}

TrapezoidalTrajectory::Step_t TrapezoidalTrajectory::eval(float t) {
    Step_t trajStep;
    if (t < 0.0f) {  // Initial Condition
        trajStep.Y   = Xi_;
        trajStep.Yd  = Vi_;
        trajStep.Ydd = 0.0f;
    } else if (t < Ta_) {  // Accelerating
        trajStep.Y   = Xi_ + Vi_*t + 0.5f*Ar_*SQ(t);
        trajStep.Yd  = Vi_ + Ar_*t;
        trajStep.Ydd = Ar_;
    } else if (t < Ta_ + Tv_) {  // Coasting
        trajStep.Y   = yAccel_ + Vr_*(t - Ta_);
        trajStep.Yd  = Vr_;
        trajStep.Ydd = 0.0f;
    } else if (t < Tf_) {  // Deceleration
        float td     = t - Tf_;
        trajStep.Y   = Xf_ + 0.5f*Dr_*SQ(td);
        trajStep.Yd  = Dr_*td;
        trajStep.Ydd = Dr_;
    } else if (t >= Tf_) {  // Final Condition
        trajStep.Y   = Xf_;
        trajStep.Yd  = 0.0f;
        trajStep.Ydd = 0.0f;
    } else {
        // TODO: report error here
    }

    return trajStep;
}

static_assert(sizeof(float) * CHAR_BIT == 32);


void run_trajectory_test(float goal, float position, float velocity, float Vmax, float Amax, float Dmax) {
    float dt = 0.000125f;
    int replan_interval = 10; // must be > 2 (see note below)
    float t = 0.0f;
    float Vmax_test = std::max(Vmax, std::abs(velocity));

    TrapezoidalTrajectory traj{};

    int replan_counter = 0;

    do {
        if (replan_counter <= 0) {
            CHECK(traj.planTrapezoidal(goal, position, velocity, Vmax, Amax, Dmax));
            t = 0.0f;
            replan_counter = replan_interval;
        } else {
            replan_counter--;
        }

        TrapezoidalTrajectory::Step_t step = traj.eval(t);
        t += dt;

        //std::cerr << "vel: " << step.Yd << ", pos: " << step.Y << "\n";
        
        // Check if acceleration within bounds
        if (velocity >= 0.0f) {
            CHECK(step.Ydd <= Amax);
            CHECK(step.Ydd >= -Dmax);
            CHECK((step.Yd - velocity) / dt <= Amax * 1.002f);
            CHECK((step.Yd - velocity) / dt >= -Dmax * 1.002f);
        } else {
            CHECK(step.Ydd <= Dmax);
            CHECK(step.Ydd >= -Amax);
            CHECK((step.Yd - velocity) / dt <= Dmax * 1.002f);
            CHECK((step.Yd - velocity) / dt >= -Amax * 1.002f);
        }

        // Check if velocity within bounds
        CHECK(step.Yd >= -Vmax_test);
        CHECK(step.Yd <= Vmax_test);
        CHECK((step.Y - position) / dt >= -Vmax_test * 1.002f);
        CHECK((step.Y - position) / dt <= Vmax_test * 1.002f);
        velocity = step.Yd;

        // Check if position is making progress
        // TODO: the trajectory planner currently needs three "warm-up" iterations
        // until its position makes progress. This should probably be revisited.
        // TODO: this is disabled currently because there are legitimate trajectories
        // where the position first moves in the wrong direction.
        //if ((replan_counter < replan_interval - 2) && (t <= traj.Tf_)) {
        //    CHECK(std::abs(step.Y - goal) < std::abs(position - goal));
        //}
        position = step.Y;

    } while (t <= traj.Tf_);

    CHECK(position >= goal - 1.0f);
    CHECK(position <= goal + 1.0f);
    CHECK(velocity >= -Dmax * dt);
    CHECK(velocity <= Dmax * dt);
}


TEST_SUITE("Trajectory Planner") {
    // these form a triangle trajectory because 2*v^2/(2*a) = 2 * 27712^2 / (2*22288) = 34456 > 16384
    TEST_CASE("neg-dir-triangle") {
        run_trajectory_test(-8192.0f, 8192.0f, 0.0f, 27712.0f, 22288.0f, 22288.0f);
    }
    TEST_CASE("pos-dir-triangle") {
        run_trajectory_test(8192.0f, -8192.0f, 0.0f, 27712.0f, 22288.0f, 22288.0f);
    }

    // these form a trapezoid trajectory because 2*v^2/(2*a) = 2 * 27712^2 / (2*22288) = 34456 < 16384
    TEST_CASE("neg-dir-trapezoid") {
        run_trajectory_test(-25000.0f, 25000.0f, 0.0f, 27712.0f, 22288.0f, 22288.0f);
    }
    TEST_CASE("pos-dir-trapezoid") {
        run_trajectory_test(25000.0f, -25000.0f, 0.0f, 27712.0f, 22288.0f, 22288.0f);
    }

    // for the following tests note that v^2/(2*a) = 27712^2 / (2*22288) = 17227 > 16384
    TEST_CASE("neg-dir-not-enough-braking-distance") {
        run_trajectory_test(-8192.0f, 8192.0f, -27712.0f, 27712.0f, 22288.0f, 22288.0f);
    }
    TEST_CASE("pos-dir-not-enough-braking-distance") {
        run_trajectory_test(8192.0f, -8192.0f, 27712.0f, 27712.0f, 22288.0f, 22288.0f);
    }

    TEST_CASE("neg-dir-over-speed") {
        run_trajectory_test(-8192.0f, 8192.0f, -40000.0f, 27712.0f, 22288.0f, 22288.0f);
    }
    TEST_CASE("pos-dir-over-speed") {
        run_trajectory_test(8192.0f, -8192.0f, 40000.0f, 27712.0f, 22288.0f, 22288.0f);
    }
}
