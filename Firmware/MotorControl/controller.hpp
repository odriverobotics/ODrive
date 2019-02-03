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
        ERROR_INVALID_INPUT_MODE = 0x02,
        ERROR_UNSTABLE_GAIN = 0x04,
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
    

    enum InputMode_t{
        INPUT_MODE_INACTIVE,
        INPUT_MODE_PASSTHROUGH,
        INPUT_MODE_VEL_RAMP,
        INPUT_MODE_POS_FILTER,
        INPUT_MODE_MIX_CHANNELS,
        INPUT_MODE_TRAP_TRAJ,
    };

    static const int32_t NUM_HARMONICS = 16;
    typedef struct {
        uint32_t index;
        float real;
        float imaginary;
    }Harmonic_t ;

    struct Config_t {
        ControlMode_t control_mode = CTRL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode_t input_mode = INPUT_MODE_INACTIVE;  //see: InputMode_t
        float pos_gain = 20.0f;  // [(counts/s) / counts]
        float vel_gain = 5.0f / 10000.0f;  // [A/(counts/s)]
        // float vel_gain = 5.0f / 200.0f, // [A/(rad/s)] <sensorless example>
        float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
        float vel_limit = 20000.0f;        // [counts/s]
        float vel_limit_tolerance = 1.2f;  // ratio to vel_lim. 0.0f to disable
        float vel_ramp_rate = 10000.0f;  // [(counts/s) / s]
        bool setpoints_in_cpr = false;
<<<<<<< HEAD
        Harmonic_t harmonics[NUM_HARMONICS];
=======
        float inertia = 0.0f;      // [A/(count/s^2)]
        float input_filter_bandwidth = 2.0f; // [1/s]
        float gain_scheduling_width = 10.0f; // [counts] width either size of pos_setpoint
        bool enable_gain_scheduling = false; // enable gain scheduling when in position control
        float homing_speed = 2000.0f;   // [counts/s]
>>>>>>> c702f38c07712547df389cac4e77ac4c180d1bf3
    };

    Controller(Config_t& config);
    void reset();
    void set_error(Error_t error);

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);

    bool home_axis();
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);
    void extract_harmonics();
    void find_n_highest_indices(float* arr, uint32_t arr_size, uint32_t* indices, uint32_t n);
    float write_anticogging_map(int32_t index, float value);
    float write_harmonics(int32_t index, int32_t harmonic, int32_t imag, float value);
    void init_anticogging_map();

    void update_filter_gains();
    bool update(float pos_estimate, float vel_estimate, float* current_setpoint);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: anticogging overhaul:
    // - expose selected (all?) variables on protocol
    // - make calibration user experience similar to motor & encoder calibration
    // - use python tools to Fourier transform and write back the smoothed map or Fourier coefficients
    // - make the calibration persistent

    enum AnticoggingState_t {
      ANTICOG_STATE_FWD,
      ANTICOG_STATE_REV  
    };

    typedef struct {
        int index;
        float *cogging_map;
        bool use_anticogging;
        bool calib_anticogging;
        float calib_pos_threshold;
        float calib_vel_threshold;
        AnticoggingState_t state;
    } Anticogging_t;
    Anticogging_t anticogging_ = {
        .index = 0,
        .cogging_map = nullptr,
        .use_anticogging = false,
        .calib_anticogging = false,
        .calib_pos_threshold = 1.0f,
        .calib_vel_threshold = 1.0f,
        .state = ANTICOG_STATE_FWD,
    };

    Error_t error_ = ERROR_NONE;
    // variables exposed on protocol
    float pos_setpoint_ = 0.0f;
    float vel_setpoint_ = 0.0f;
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_current_ = 0.0f;  // [A]
    float current_setpoint_ = 0.0f;        // [A]

    float input_pos_ = 0.0f;
    float input_vel_ = 0.0f;
    float input_current_ = 0.0f;
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    uint32_t traj_start_loop_count_ = 0;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("input_pos", &input_pos_),
            make_protocol_property("input_vel", &input_vel_),
            make_protocol_property("input_current", &input_current_),
            make_protocol_ro_property("pos_setpoint", &pos_setpoint_),
            make_protocol_ro_property("vel_setpoint", &vel_setpoint_),
            make_protocol_ro_property("vel_integrator_current", &vel_integrator_current_),
            make_protocol_ro_property("current_setpoint", &current_setpoint_),
            make_protocol_object("config",
                make_protocol_property("control_mode", &config_.control_mode),
                make_protocol_property("input_mode", &config_.input_mode),
                make_protocol_property("pos_gain", &config_.pos_gain),
                make_protocol_property("vel_gain", &config_.vel_gain),
                make_protocol_property("vel_integrator_gain", &config_.vel_integrator_gain),
                make_protocol_property("vel_limit", &config_.vel_limit),
                make_protocol_property("vel_limit_tolerance", &config_.vel_limit_tolerance),
<<<<<<< HEAD
                make_protocol_property("vel_ramp_rate", &config_.vel_ramp_rate),
<<<<<<< HEAD
=======
                make_protocol_property("vel_ramp_limit", &config_.vel_ramp_limit),
                make_protocol_property("use_anticogging", &anticogging_.use_anticogging),
>>>>>>> fcd3c8b4d13ccb2e9d176873b0423384af696d28
                make_protocol_property("setpoints_in_cpr", &config_.setpoints_in_cpr)
            ),
            make_protocol_function("set_pos_setpoint", *this, &Controller::set_pos_setpoint,
                "pos_setpoint", "vel_feed_forward", "current_feed_forward"),
            make_protocol_function("set_vel_setpoint", *this, &Controller::set_vel_setpoint,
                "vel_setpoint", "current_feed_forward"),
            make_protocol_function("set_current_setpoint", *this, &Controller::set_current_setpoint,
                "current_setpoint"),
            make_protocol_function("move_to_pos", *this, &Controller::move_to_pos, "goal_point"),
            make_protocol_function("write_anticogging_map", *this, &Controller::write_anticogging_map,
                "index", "value"),
            make_protocol_function("write_harmonics", *this, &Controller::write_harmonics,
                        "index", "harmonic", "imag", "value"),
            make_protocol_function("init_anticogging_map", *this, &Controller::init_anticogging_map),
            make_protocol_function("start_anticogging_calibration", *this, &Controller::start_anticogging_calibration)
=======
                make_protocol_property("setpoints_in_cpr", &config_.setpoints_in_cpr),
                make_protocol_property("inertia", &config_.inertia),
                make_protocol_property("input_filter_bandwidth", &config_.input_filter_bandwidth,
                    [](void* ctx) { static_cast<Controller*>(ctx)->update_filter_gains(); }, this),
                make_protocol_property("homing_speed", &config_.homing_speed),
                make_protocol_property("gain_scheduling_width", &config_.gain_scheduling_width),
                make_protocol_property("enable_gain_scheduling", &config_.enable_gain_scheduling)
            ),
            make_protocol_function("start_anticogging_calibration", *this, &Controller::start_anticogging_calibration),
            make_protocol_function("home_axis", *this, &Controller::home_axis)
>>>>>>> c702f38c07712547df389cac4e77ac4c180d1bf3
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Controller::Error_t)

#endif // __CONTROLLER_HPP
