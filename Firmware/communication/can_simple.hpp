#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
    enum {
        MSG_NMT_CTRL            = 0x000, // CANOpen NMT Message
        MSG_SYNC_CTRL           = 0x080, // CANOpen SYNC message
        MSG_HEARTBEAT_CMD       = 0x700, // CANOpen NMT Heartbeat
        MSG_EMERGENCY           = 0x080, // CANOpen Emergency Message
        MSG_GET_MOTOR_ERROR     = 0x001, // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_CONTROLLER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_GET_VBUS_VOLTAGE,            // ODrive-level properties
        MSG_GET_SERIAL_NUMBER,
        MSG_GET_HW_VERSION,
        MSG_GET_FW_VERSION,
        MSG_REBOOT_ODRIVE,
        MSG_SAVE_CONFIG,
        MSG_ERASE_CONFIG,
        MSG_SET_AXIS_NODE_ID,           // Axis properties
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_STARTUP_CONFIG,
        MSG_SET_MOTOR_PRECALIBRATED,    // Motor properties
        MSG_GET_MOTOR_CURRENT_LIM,
        MSG_SET_MOTOR_CURRENT_LIM,
        MSG_SET_MOTOR_POLE_PAIRS,
        MSG_GET_ENCODER_CPR,            // Encoder Properties
        MSG_SET_ENCODER_CPR,
        MSG_GET_ENCODER_INDEX_FOUND,
        MSG_GET_ENCODER_POS_ESTIMATE,
        MSG_GET_ENCODER_VEL_ESTIMATE,
        MSG_SET_ENCODER_USE_INDEX,
        MSG_SET_ENCODER_PRECALIBRATED,
        MSG_MOVE_TO_POS,                // Controller properties
        MSG_SET_POS_SETPOINT,
        MSG_SET_VEL_SETPOINT,
        MSG_SET_CUR_SETPOINT,
        MSG_SET_VEL_LIMIT,
        MSG_START_ANTICOGGING
    };

    static void handle_can_message(CAN_message_t& msg);
    static void send_heartbeat(Axis* axis);

   private:
    static void estop_callback();

    // Controller
    static void move_to_pos_callback(Axis* axis, CAN_message_t& msg);
    static void set_pos_setpoint_callback(Axis* axis, CAN_message_t& msg);
    static void set_vel_setpoint_callback(Axis* axis, CAN_message_t& msg);
    static void set_current_setpoint_callback(Axis* axis, CAN_message_t& msg);

    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking
    //
    // const std::map<uint32_t, std::function<void(CAN_message_t&)>> callback_map = {
    //     {0x000, std::bind(&CANSimple::heartbeat_callback, this, _1)}
    // };
};

#endif