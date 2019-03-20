#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

class TrapezoidalTrajectory {
public:
    struct Config_t {
        float vel_limit = 20000.0f;  // [count/s]
        float accel_limit = 5000.0f; // [count/s^2]
        float decel_limit = 5000.0f; // [count/s^2]
        float A_per_css = 0.0f;      // [A/(count/s^2)]
    };
    
    struct Step_t {
        float Y;
        float Yd;
        float Ydd;
    };

    explicit TrapezoidalTrajectory(Config_t& config);
    bool planTrapezoidal(float Xf, float Xi, float Vi,
                         float Vmax, float Amax, float Dmax);
    Step_t eval(float t);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_object("config",
                make_protocol_property("vel_limit", &config_.vel_limit),
                make_protocol_property("accel_limit", &config_.accel_limit),
                make_protocol_property("decel_limit", &config_.decel_limit),
                make_protocol_property("A_per_css", &config_.A_per_css)
            )
        );
    }

    Axis* axis_ = nullptr;  // set by Axis constructor
    Config_t& config_;

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