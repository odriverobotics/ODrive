#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

struct TrapTrajConfig_t {
    float vel_limit = 20000.0f;  // [count/s]
    float accel_limit = 5000.0f; // [count/s^2]
    float decel_limit = 5000.0f; // [count/s^2]
    float cpss_to_A = 0.0f;      // [A/(count/s^2)]
};

struct TrapTrajStep_t {
    float Y;
    float Yd;
    float Ydd;
};

class TrapezoidalTrajectory {
   public:
    TrapezoidalTrajectory(TrapTrajConfig_t& config);
    bool planTrapezoidal(float Xf, float Xi, float Vi,
                         float Vmax, float Amax, float Dmax);
    TrapTrajStep_t evalTrapTraj(float t);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_object("config",
                make_protocol_property("vel_limit", &config_.vel_limit),
                make_protocol_property("accel_limit", &config_.accel_limit),
                make_protocol_property("decel_limit", &config_.decel_limit),
                make_protocol_property("cpss_to_A", &config_.cpss_to_A)
            )
        );
    }

    Axis* axis_ = nullptr;  // set by Axis constructor
    TrapTrajConfig_t& config_;

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
};

#endif