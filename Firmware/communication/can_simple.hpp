#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,         // CANOpen NMT Message REC
        MSG_CO_SYNC_CTRL = 0x080,        // CANOpen SYNC message   REC
        MSG_CO_HEARTBEAT_CMD = 0x700,    // CANOpen NMT Heartbeat  SEND
        MSG_CO_EMERGENCY = 0x080,        // CANOpen Emergency Message  SEND
        MSG_GET_MOTOR_ERROR = 0x001,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_MOVE_TO_POS,
        MSG_SET_POS_SETPOINT,
        MSG_SET_VEL_SETPOINT,
        MSG_SET_CUR_SETPOINT,
        MSG_SET_VEL_LIMIT,
        MSG_START_ANTICOGGING
    };

    static void handle_can_message(CAN_message_t& msg);
    static void send_heartbeat(Axis* axis);

   private:
    static void nmt_callback(Axis* axis, CAN_message_t& msg);
    static void sync_callback(Axis* axis, CAN_message_t& msg);
    static void estop_callback(Axis* axis, CAN_message_t& msg);
    static void get_motor_error_callback(Axis* axis, CAN_message_t& msg);
    static void get_encoder_error_callback(Axis* axis, CAN_message_t& msg);
    static void get_controller_error_callback(Axis* axis, CAN_message_t& msg);
    static void get_sensorless_error_callback(Axis* axis, CAN_message_t& msg);
    static void set_axis_nodeid_callback(Axis* axis, CAN_message_t& msg);
    static void set_axis_requested_state_callback(Axis* axis, CAN_message_t& msg);
    static void set_axis_startup_config_callback(Axis* axis, CAN_message_t& msg);
    static void get_encoder_estimates_callback(Axis* axis, CAN_message_t& msg);
    static void move_to_pos_callback(Axis* axis, CAN_message_t& msg);
    static void set_pos_setpoint_callback(Axis* axis, CAN_message_t& msg);
    static void set_vel_setpoint_callback(Axis* axis, CAN_message_t& msg);
    static void set_current_setpoint_callback(Axis* axis, CAN_message_t& msg);
    static void set_vel_limit_callback(Axis* axis, CAN_message_t& msg);
    static void start_anticogging_callback(Axis* axis, CAN_message_t& msg);

    // Utility functions
    static uint8_t get_node_id(uint32_t msgID);
    static uint8_t get_cmd_id(uint32_t msgID);

    static uint16_t get_16bit_val(CAN_message_t& msg, uint8_t start_byte);
    static uint32_t get_32bit_val(CAN_message_t& msg, uint8_t start_byte);

    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking
    //
    // const std::map<uint32_t, std::function<void(CAN_message_t&)>> callback_map = {
    //     {0x000, std::bind(&CANSimple::heartbeat_callback, this, _1)}
    // };
};

#endif