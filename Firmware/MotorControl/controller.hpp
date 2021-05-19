#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

class Controller : public ODriveIntf::ControllerIntf {
public:
    struct Anticogging_t {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 1.0f;
        float calib_vel_threshold = 1.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    };

    struct Autotuning_t {
        float frequency = 0.0f;
        float pos_amplitude = 0.0f;
        float vel_amplitude = 0.0f;
        float torque_amplitude = 0.0f;
    };

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;             //see: InputMode_t
        float pos_gain = 20.0f;                  // [(turn/s) / turn]
        float vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
        // float vel_gain = 0.2f / 200.0f,       // [Nm/(rad/s)] <sensorless example>
        float vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
        float vel_differentiator_gain = 0.0f;    // [Nm/turn]
        float vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
        uint32_t steps_per_circular_range = 1024;
        float inertia = 0.0f;                    // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;     // [1/s]
        float homing_speed = 0.25f;              // [turn/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_torque_mode_vel_limit = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
        uint8_t axis_to_mirror = -1;
        float mirror_ratio = 1.0f;
        float torque_mirror_ratio = 0.0f;
        uint8_t load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
        float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
        float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
        float spinout_electrical_power_threshold = 10.0f; // [W] electrical power threshold for spinout detection
        float spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
        void set_steps_per_circular_range(uint32_t value) { steps_per_circular_range = value > 0 ? value : steps_per_circular_range; }
    };

    
    bool apply_config();

    void reset();
    void set_error(Error error);

    constexpr void input_pos_updated() {
        input_pos_updated_ = true;
    }

    bool select_encoder(size_t encoder_num);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    void update_filter_gains();
    bool update();

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;
    float last_error_time_ = 0.0f;

    // Inputs
    InputPort<float> pos_estimate_linear_src_;
    InputPort<float> pos_estimate_circular_src_;
    InputPort<float> vel_estimate_src_;
    InputPort<float> pos_wrap_src_; 

    float pos_setpoint_ = 0.0f; // [turns]
    float vel_setpoint_ = 0.0f; // [turn/s]
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm]
    float vel_err_previous_ = 0.0f;

    float input_pos_ = 0.0f;     // [turns]
    float input_vel_ = 0.0f;     // [turn/s]
    float input_torque_ = 0.0f;  // [Nm]
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    Autotuning_t autotuning_;
    float autotuning_phase_ = 0.0f;
    
    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    bool anticogging_valid_ = false;
    float mechanical_power_ = 0.0f; // [W]
    float electrical_power_ = 0.0f; // [W]

    // Outputs
    OutputPort<float> torque_output_ = 0.0f;

    // custom setters
    void set_input_pos(float value) { input_pos_ = value; input_pos_updated(); }
};

#endif // __CONTROLLER_HPP
