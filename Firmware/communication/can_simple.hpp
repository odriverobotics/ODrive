#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include <stm32_can.hpp>

#include "odrive_main.h"

#define CAN_CLK_HZ (42000000)
#define CAN_CLK_MHZ (42)

class CANSimple {
public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,         // CANOpen NMT Message REC
        MSG_CO_HEARTBEAT_CMD = 0x700,    // CANOpen NMT Heartbeat  SEND
        MSG_ODRIVE_HEARTBEAT = 0x001,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_MOVE_TO_POS,
        MSG_SET_POS_SETPOINT,
        MSG_SET_VEL_SETPOINT,
        MSG_SET_CUR_SETPOINT,
        MSG_SET_VEL_LIMIT,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_A_PER_CSS,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,
        MSG_GET_VBUS_VOLTAGE,
    };

    enum Error_t {
        ERROR_NONE = 0x00,
        ERROR_DUPLICATE_CAN_IDS = 0x01
    };

    struct Config_t {
        CAN_baudrate_t baud = CAN_BAUD_250K;
    };

    CANSimple(STM32_CAN_t* can, Config_t &config);

    bool start_can_server();
    void server_thread();
    void send_heartbeat(Axis *axis);

    void set_error(Error_t error);

    // I/O Functions
    //uint32_t available() { return can_->get_rx_pending(0) + can_->get_rx_pending(1); }

    void handle_can_message(CAN_message_t& msg);

    // Thread Relevant Data
    osThreadId thread_id_;
    osSemaphoreId sem_can_;
    volatile bool thread_id_valid_ = false;

    Error_t error_ = ERROR_NONE;

    void nmt_callback(Axis* axis, CAN_message_t& msg);
    void estop_callback(Axis* axis, CAN_message_t& msg);
    void get_motor_error_callback(Axis* axis, CAN_message_t& msg);
    void get_encoder_error_callback(Axis* axis, CAN_message_t& msg);
    void get_controller_error_callback(Axis* axis, CAN_message_t& msg);
    void get_sensorless_error_callback(Axis* axis, CAN_message_t& msg);
    void set_axis_nodeid_callback(Axis* axis, CAN_message_t& msg);
    void set_axis_requested_state_callback(Axis* axis, CAN_message_t& msg);
    void set_axis_startup_config_callback(Axis* axis, CAN_message_t& msg);
    void get_encoder_estimates_callback(Axis* axis, CAN_message_t& msg);
    void get_encoder_count_callback(Axis* axis, CAN_message_t& msg);
    void move_to_pos_callback(Axis* axis, CAN_message_t& msg);
    void set_pos_setpoint_callback(Axis* axis, CAN_message_t& msg);
    void set_vel_setpoint_callback(Axis* axis, CAN_message_t& msg);
    void set_current_setpoint_callback(Axis* axis, CAN_message_t& msg);
    void set_vel_limit_callback(Axis* axis, CAN_message_t& msg);
    void start_anticogging_callback(Axis* axis, CAN_message_t& msg);
    void set_traj_vel_limit_callback(Axis* axis, CAN_message_t& msg);
    void set_traj_accel_limits_callback(Axis* axis, CAN_message_t& msg);
    void set_traj_A_per_css_callback(Axis* axis, CAN_message_t& msg);
    void get_iq_callback(Axis* axis, CAN_message_t& msg);
    void get_sensorless_estimates_callback(Axis* axis, CAN_message_t& msg);
    void get_vbus_voltage_callback(Axis* axis, CAN_message_t& msg);


    // Utility functions
    static uint8_t get_node_id(uint32_t msgID);
    static uint8_t get_cmd_id(uint32_t msgID);

    static int16_t get_16bit_val(CAN_message_t& msg, uint8_t start_byte);
    static int32_t get_32bit_val(CAN_message_t& msg, uint8_t start_byte);
    static float get_float(CAN_message_t& msg, uint8_t start_byte);

    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking
    //
    // const std::map<uint32_t, std::function<void(CAN_message_t&)>> callback_map = {
    //     {0x000, std::bind(&CANSimple::heartbeat_callback, this, _1)}
    // };

    // Communication Protocol Handling
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_object("config",
                                 make_protocol_ro_property("baud_rate", &config_.baud)),
            make_protocol_function("set_baud_rate", *this, &CANSimple::set_baud_rate, "baudRate"));
    }

private:
    bool reinit();

    bool set_baud_rate(uint32_t baudRate) {
        config_.baud = static_cast<CAN_baudrate_t>(baudRate);
        return reinit();
    }

    STM32_CAN_t* can_ = nullptr;
    Config_t& config_;
};

DEFINE_ENUM_FLAG_OPERATORS(CANSimple::Error_t)

extern CANSimple::Config_t can_config;

#endif