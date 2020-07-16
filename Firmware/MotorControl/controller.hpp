#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

class Controller : public ODriveIntf::ControllerIntf {
public:
    typedef struct {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 1.0f;
        float calib_vel_threshold = 1.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    } Anticogging_t;

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;  //see: InputMode_t
        float pos_gain = 20.0f;                         // [(counts/s) / counts]
        float vel_gain = 0.2f / 10000.0f;               // [Nm/(counts/s)]
        // float vel_gain = 0.2f / 200.0f,              // [Nm/(rad/s)] <sensorless example>
        float vel_integrator_gain = 0.4f / 10000.0f;    // [Nm/(counts/s * s)]
        float vel_limit = 20000.0f;                     // [counts/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;               // ratio to vel_lim. Infinity to disable.
        float vel_ramp_rate = 10000.0f;                 // [(counts/s) / s]
        float torque_ramp_rate = 0.01f;                 // Nm / sec
        bool setpoints_in_cpr = false;
        float inertia = 0.0f;                           // [A/(count/s^2)]
        float input_filter_bandwidth = 2.0f;            // [1/s]
        float homing_speed = 2000.0f;                   // [counts/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_current_mode_vel_limit = true;           // enable velocity limit in current control mode (requires a valid velocity estimator)
        uint8_t axis_to_mirror = -1;
        float mirror_ratio = 1.0f;
        uint8_t load_encoder_axis = -1;                 // default depends on Axis number and is set in load_configuration()

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
    };

    explicit Controller(Config_t& config);
    void reset();
    void set_error(Error error);

    void input_pos_updated();
    bool select_encoder(size_t encoder_num);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    void update_filter_gains();
    bool update(float* torque_setpoint);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;

    float* pos_estimate_src_ = nullptr;
    bool* pos_estimate_valid_src_ = nullptr;
    float* vel_estimate_src_ = nullptr;
    bool* vel_estimate_valid_src_ = nullptr;
    int32_t* pos_wrap_src_ = nullptr; // enables circular position setpoints if not null. The value pointed to is the maximum position value.

    float pos_setpoint_ = 0.0f;
    float vel_setpoint_ = 0.0f;
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;          // [Nm]

    float input_pos_ = 0.0f;
    float input_vel_ = 0.0f;
    float input_torque_ = 0.0f;
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    bool anticogging_valid_ = false;

    // custom setters
    void set_input_pos(float value) { input_pos_ = value; input_pos_updated(); }
};

#endif // __CONTROLLER_HPP
