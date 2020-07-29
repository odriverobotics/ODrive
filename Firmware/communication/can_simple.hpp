#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,       // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_SET_CONTROLLER_MODES,
        MSG_SET_INPUT_POS,
        MSG_SET_INPUT_VEL,
        MSG_SET_INPUT_TORQUE,
        MSG_SET_VEL_LIMIT,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_INERTIA,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,
        MSG_GET_VBUS_VOLTAGE,
        MSG_CLEAR_ERRORS,
        MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
    };

    static void handle_can_message(can_Message_t& msg);
    static void send_heartbeat(Axis* axis);

   private:
    static void nmt_callback(Axis* axis, can_Message_t& msg);
    static void estop_callback(Axis* axis, can_Message_t& msg);
    static void get_motor_error_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_error_callback(Axis* axis, can_Message_t& msg);
    static void get_controller_error_callback(Axis* axis, can_Message_t& msg);
    static void get_sensorless_error_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_nodeid_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_startup_config_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_estimates_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_count_callback(Axis* axis, can_Message_t& msg);
    static void set_input_pos_callback(Axis* axis, can_Message_t& msg);
    static void set_input_vel_callback(Axis* axis, can_Message_t& msg);
    static void set_input_torque_callback(Axis* axis, can_Message_t& msg);
    static void set_controller_modes_callback(Axis* axis, can_Message_t& msg);
    static void set_vel_limit_callback(Axis* axis, can_Message_t& msg);
    static void start_anticogging_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_inertia_callback(Axis* axis, can_Message_t& msg);
    static void get_iq_callback(Axis* axis, can_Message_t& msg);
    static void get_sensorless_estimates_callback(Axis* axis, can_Message_t& msg);
    static void get_vbus_voltage_callback(Axis* axis, can_Message_t& msg);
    static void clear_errors_callback(Axis* axis, can_Message_t& msg);

    // Utility functions
    static uint32_t get_node_id(uint32_t msgID);
    static uint8_t get_cmd_id(uint32_t msgID);

    // Fetch a specific signal from the message

    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking
    //
    // const std::map<uint32_t, std::function<void(can_Message_t&)>> callback_map = {
    //     {0x000, std::bind(&CANSimple::heartbeat_callback, this, _1)}
    // };
};





#endif