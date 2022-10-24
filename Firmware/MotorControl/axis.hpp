#ifndef __AXIS_HPP
#define __AXIS_HPP

class Axis;

#include "encoder.hpp"
#include "acim_estimator.hpp"
#include "sensorless_estimator.hpp"
#include "controller.hpp"
#include "open_loop_controller.hpp"
#include "trapTraj.hpp"
#include "endstop.hpp"
#include "mechanical_brake.hpp"
#include "low_level.h"
#include "utils.hpp"
#include "task_timer.hpp"

#include <array>

class Axis : public ODriveIntf::AxisIntf {
public:
    struct LockinConfig_t {
        float current = 10.0f;           // [A]
        float ramp_time = 0.4f;          // [s]
        float ramp_distance = 1 * M_PI;  // [rad]
        float accel = 20.0f;     // [rad/s^2]
        float vel = 40.0f; // [rad/s]
        float finish_distance = 100.0f;  // [rad]
        bool finish_on_vel = false;
        bool finish_on_distance = false;
        bool finish_on_enc_idx = false;
    };

    struct TaskTimes {
        TaskTimer thermistor_update;
        TaskTimer encoder_update;
        TaskTimer sensorless_estimator_update;
        TaskTimer endstop_update;
        TaskTimer can_heartbeat;
        TaskTimer controller_update;
        TaskTimer open_loop_controller_update;
        TaskTimer acim_estimator_update;
        TaskTimer motor_update;
        TaskTimer current_controller_update;
        TaskTimer dc_calib;
        TaskTimer current_sense;
        TaskTimer pwm_update;
    };

    static LockinConfig_t default_calibration();
    static LockinConfig_t default_sensorless();
    static LockinConfig_t default_lockin();

    struct CANConfig_t {
        uint32_t node_id = 0;
        bool is_extended = false;
        uint32_t heartbeat_rate_ms = 100;
        uint32_t encoder_rate_ms = 10;
        uint32_t motor_error_rate_ms = 0;
        uint32_t encoder_error_rate_ms = 0;
        uint32_t controller_error_rate_ms = 0;
        uint32_t sensorless_error_rate_ms = 0;
        uint32_t encoder_count_rate_ms = 0;
        uint32_t iq_rate_ms = 0;
        uint32_t sensorless_rate_ms = 0;
        uint32_t bus_vi_rate_ms = 0;
    };

    struct Config_t {
        bool startup_motor_calibration = false;   //<! run motor calibration at startup, skip otherwise
        bool startup_encoder_index_search = false; //<! run encoder index search after startup, skip otherwise
                                                // this only has an effect if encoder.config.use_index is also true
        bool startup_encoder_offset_calibration = false; //<! run encoder offset calibration after startup, skip otherwise
        bool startup_closed_loop_control = false; //<! enable closed loop control after calibration/startup
        bool startup_homing = false; //<! enable homing after calibration/startup

        bool enable_step_dir = false; //<! enable step/dir input after calibration
                                    //   For M0 this has no effect if enable_uart is true
        bool step_dir_always_on = false; //<! Keep step/dir enabled while the motor is disabled.
                                         //<! This is ignored if enable_step_dir is false.
                                         //<! This setting only takes effect on a state transition
                                         //<! into idle or out of closed loop control.

        bool enable_sensorless_mode = false;

        float watchdog_timeout = 0.0f; // [s]
        bool enable_watchdog = false;

        // Defaults loaded from hw_config in load_configuration in main.cpp
        uint16_t step_gpio_pin = 0;
        uint16_t dir_gpio_pin = 0;

        LockinConfig_t calibration_lockin = default_calibration();
        LockinConfig_t sensorless_ramp = default_sensorless();
        LockinConfig_t general_lockin;

        CANConfig_t can;

        // custom setters
        Axis* parent = nullptr;
        void set_step_gpio_pin(uint16_t value) { step_gpio_pin = value; parent->decode_step_dir_pins(); }
        void set_dir_gpio_pin(uint16_t value) { dir_gpio_pin = value; parent->decode_step_dir_pins(); }
    };

    struct Homing_t {
        bool is_homed = false;
    };

    struct CAN_t {
        uint32_t last_heartbeat = 0;
        uint32_t last_encoder = 0;
        uint32_t last_motor_error = 0;
        uint32_t last_encoder_error = 0;
        uint32_t last_controller_error = 0;
        uint32_t last_sensorless_error = 0;
        uint32_t last_encoder_count = 0;
        uint32_t last_iq = 0;
        uint32_t last_sensorless = 0;
        uint32_t last_bus_vi = 0;
    };

    Axis(int axis_num,
            uint16_t default_step_gpio_pin,
            uint16_t default_dir_gpio_pin,
            osPriority thread_priority,
            Encoder& encoder,
            SensorlessEstimator& sensorless_estimator,
            Controller& controller,
            Motor& motor,
            TrapezoidalTrajectory& trap,
            Endstop& min_endstop,
            Endstop& max_endstop,
            MechanicalBrake& mechanical_brake);

    bool apply_config();
    void clear_config();

    void start_thread();
    bool wait_for_control_iteration();

    void step_cb();
    void set_step_dir_active(bool enable);
    void decode_step_dir_pins();

    bool do_checks(uint32_t timestamp);

    void watchdog_feed();
    bool watchdog_check();

    // True if there are no errors
    bool inline check_for_errors() {
        return error_ == ERROR_NONE;
    }

    bool start_closed_loop_control();
    bool stop_closed_loop_control();
    bool run_lockin_spin(const LockinConfig_t &lockin_config, bool remain_armed,
                std::function<bool(bool)> loop_cb = {} );
    bool run_closed_loop_control_loop();
    bool run_homing();
    bool run_idle_loop();

    constexpr uint32_t get_watchdog_reset() {
        return static_cast<uint32_t>(std::clamp<float>(config_.watchdog_timeout, 0, UINT32_MAX / (current_meas_hz + 1)) * current_meas_hz);
    }

    void run_state_machine_loop();

    // hardware config
    int axis_num_;
    uint16_t default_step_gpio_pin_;
    uint16_t default_dir_gpio_pin_;
    osPriority thread_priority_;
    Config_t config_;

    Encoder& encoder_;
    AcimEstimator acim_estimator_;
    SensorlessEstimator& sensorless_estimator_;
    Controller& controller_;
    OpenLoopController open_loop_controller_;
    Motor& motor_;
    TrapezoidalTrajectory& trap_traj_;
    Endstop& min_endstop_;
    Endstop& max_endstop_;
    MechanicalBrake& mechanical_brake_;
    TaskTimes task_times_;

    osThreadId thread_id_ = 0;
    const uint32_t stack_size_ = 2048; // Bytes
    volatile bool thread_id_valid_ = false;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    bool step_dir_active_ = false; // auto enabled after calibration, based on config.enable_step_dir
    int64_t steps_ = 0; // Steps counted at interface
    uint32_t last_drv_fault_ = 0;

    // updated from config in constructor, and on protocol hook
    Stm32Gpio step_gpio_;
    Stm32Gpio dir_gpio_;

    AxisState requested_state_ = AXIS_STATE_STARTUP_SEQUENCE;
    std::array<AxisState, 10> task_chain_ = { AXIS_STATE_UNDEFINED };
    AxisState& current_state_ = task_chain_.front();
    Homing_t homing_;
    CAN_t can_;


    // watchdog
    uint32_t watchdog_current_value_= 0;
};


#endif /* __AXIS_HPP */
