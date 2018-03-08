#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_HPP
#error "This file should not be included directly. Include odrive_main.hpp instead."
#endif

// Note: these should be sorted from lowest level of control to
// highest level of control, to allow "<" style comparisons.
typedef enum {
    CTRL_MODE_VOLTAGE_CONTROL = 0,
    CTRL_MODE_CURRENT_CONTROL = 1,
    CTRL_MODE_VELOCITY_CONTROL = 2,
    CTRL_MODE_POSITION_CONTROL = 3
} Motor_control_mode_t;

struct ControllerConfig_t {
    Motor_control_mode_t control_mode = CTRL_MODE_POSITION_CONTROL;  //see: Motor_control_mode_t
    float pos_gain = 20.0f;  // [(counts/s) / counts]
    float vel_gain = 5.0f / 10000.0f;  // [A/(counts/s)]
    // float vel_gain = 15.0f / 200.0f, // [A/(rad/s)] <sensorless example>
    float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
    float vel_limit = 20000.0f;           // [counts/s]
};

class Controller {
public:
    Controller(ControllerConfig_t& config);

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);

    // TODO: make this more similar to other calibration loops
    bool anti_cogging_calibration(float pos_estimate, float vel_estimate);

    bool update(float pos_estimate, float vel_estimate, float* current_setpoint);

    ControllerConfig_t& config;
    Axis* axis = nullptr; // set by Axis constructor

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
    Anticogging_t anticogging = {
        .index = 0,
        .cogging_map = nullptr,
        .use_anticogging = false,
        .calib_anticogging = false,
        .calib_pos_threshold = 1.0f,
        .calib_vel_threshold = 1.0f,
    };

    // variables exposed on protocol
    float pos_setpoint = 0.0f;
    float vel_setpoint = 0.0f;
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_current = 0.0f;  // [A]
    float current_setpoint = 0.0f;        // [A]

    // Cache for remote procedure calls arguments TODO: remove
    struct {
        float pos_setpoint; 
        float vel_feed_forward;
        float current_feed_forward;
    } set_pos_setpoint_args;
    struct {
        float vel_setpoint;
        float current_feed_forward;
    } set_vel_setpoint_args;
    struct {
        float current_setpoint;
    } set_current_setpoint_args;
};

#endif // __CONTROLLER_HPP
