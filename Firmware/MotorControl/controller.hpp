#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Controller {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_OVERSPEED = 0x01,
    };

    // Note: these should be sorted from lowest level of control to
    // highest level of control, to allow "<" style comparisons.
    enum ControlMode_t{
        CTRL_MODE_VOLTAGE_CONTROL = 0,
        CTRL_MODE_CURRENT_CONTROL = 1,
        CTRL_MODE_VELOCITY_CONTROL = 2,
        CTRL_MODE_POSITION_CONTROL = 3,
        CTRL_MODE_TRAJECTORY_CONTROL = 4
    };

    struct Config_t {
        ControlMode_t control_mode = CTRL_MODE_POSITION_CONTROL;  //see: Motor_control_mode_t
        float pos_gain = 20.0f;  // [(counts/s) / counts]
        float vel_gain = 5.0f / 10000.0f;  // [A/(counts/s)]
        // float vel_gain = 5.0f / 200.0f, // [A/(rad/s)] <sensorless example>
        float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
        float vel_limit = 20000.0f;        // [counts/s]
        float vel_limit_tolerance = 1.2f;  // ratio to vel_lim. 0.0f to disable
        float vel_ramp_rate = 10000.0f;  // [(counts/s) / s]
        bool setpoints_in_cpr = false;
    };

    explicit Controller(Config_t& config);
    void reset();
    void set_error(Error_t error);

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    bool update(float pos_estimate, float vel_estimate, float* current_setpoint);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: anticogging overhaul:
    // - expose selected (all?) variables on protocol
    // - make calibration user experience similar to motor & encoder calibration
    // - use python tools to Fourier transform and write back the smoothed map or Fourier coefficients
    // - make the calibration persistent

    typedef struct {
        int index;
        float *cogging_map;
        bool use_anticogging;
        bool calib_anticogging;
        float calib_pos_threshold;
        float calib_vel_threshold;
    } Anticogging_t;
    Anticogging_t anticogging_ = {
        .index = 0,
        .cogging_map = nullptr,
        .use_anticogging = false,
        .calib_anticogging = false,
        .calib_pos_threshold = 1.0f,
        .calib_vel_threshold = 1.0f,
    };

    Error_t error_ = ERROR_NONE;
    // variables exposed on protocol
    float pos_setpoint_ = 0.0f;
    float vel_setpoint_ = 0.0f;
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_current_ = 0.0f;  // [A]
    float current_setpoint_ = 0.0f;        // [A]
    float vel_ramp_target_ = 0.0f;
    bool vel_ramp_enable_ = false;

    uint32_t traj_start_loop_count_ = 0;

    float goal_point_ = 0.0f;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("pos_setpoint", &pos_setpoint_),
            make_protocol_property("vel_setpoint", &vel_setpoint_),
            make_protocol_property("vel_integrator_current", &vel_integrator_current_),
            make_protocol_property("current_setpoint", &current_setpoint_),
            make_protocol_property("vel_ramp_target", &vel_ramp_target_),
            make_protocol_property("vel_ramp_enable", &vel_ramp_enable_),
            make_protocol_object("config",
                make_protocol_property("control_mode", &config_.control_mode),
                make_protocol_property("pos_gain", &config_.pos_gain),
                make_protocol_property("vel_gain", &config_.vel_gain),
                make_protocol_property("vel_integrator_gain", &config_.vel_integrator_gain),
                make_protocol_property("vel_limit", &config_.vel_limit),
                make_protocol_property("vel_limit_tolerance", &config_.vel_limit_tolerance),
                make_protocol_property("vel_ramp_rate", &config_.vel_ramp_rate),
                make_protocol_property("setpoints_in_cpr", &config_.setpoints_in_cpr)
            ),
            make_protocol_function("set_pos_setpoint", *this, &Controller::set_pos_setpoint,
                "pos_setpoint", "vel_feed_forward", "current_feed_forward"),
            make_protocol_function("set_vel_setpoint", *this, &Controller::set_vel_setpoint,
                "vel_setpoint", "current_feed_forward"),
            make_protocol_function("set_current_setpoint", *this, &Controller::set_current_setpoint,
                                   "current_setpoint"),
            make_protocol_function("move_to_pos", *this, &Controller::move_to_pos, "pos_setpoint"),
            make_protocol_function("move_incremental", *this, &Controller::move_incremental, "displacement", "from_goal_point"),
            make_protocol_function("start_anticogging_calibration", *this, &Controller::start_anticogging_calibration)
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Controller::Error_t)

#endif // __CONTROLLER_HPP
