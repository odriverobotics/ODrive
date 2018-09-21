#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

struct TrapTrajConfig_t {
    float vel_limit = 20000.0f;
    float accel_limit = 5000.0f;
    float decel_limit = 5000.0f;
    float A_to_cpss = 0.0f;
};

struct TrapTrajStep_t {
    float Y;
    float Yd;
    float Ydd;
};

class TrapezoidalTrajectory {
   public:
    Axis *axis_ = nullptr;  // set by Axis constructor
    TrapTrajConfig_t &config_;

    TrapezoidalTrajectory(TrapTrajConfig_t &config);

    float planTrapezoidal(float Xf, float Xi,
                          float Vi, float Vmax,
                          float Amax, float Dmax);

    TrapTrajStep_t evalTrapTraj(float t);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_object("config",
                make_protocol_property("vel_limit", &config_.vel_limit),
                make_protocol_property("accel_limit", &config_.accel_limit),
                make_protocol_property("decel_limit", &config_.decel_limit)));
    }

   private:
    float yAccel_;

    float Xi_;
    float Xf_;
    float Vi_;

    float Ar_;
    float Dr_;
    float Vr_;

    float Ta_;
    float Tv_;
    float Td_;
    float Tav_;
};

#endif