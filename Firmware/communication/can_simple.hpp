#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
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
        MSG_SET_PERIODIC_UPDATES,
        MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
    };

    static void handle_can_message(const can_Message_t& msg);
    static void doCommand(Axis& axis, const can_Message_t& cmd);

    // Cyclic Senders
    static int32_t send_heartbeat(const Axis& axis);
    static void send_cyclic(Axis& axis);

   private:
    // Get functions (msg.rtr bit must be set)
    static int32_t get_motor_error_callback(const Axis& axis);
    static int32_t get_encoder_error_callback(const Axis& axis);
    static int32_t get_controller_error_callback(const Axis& axis);
    static int32_t get_sensorless_error_callback(const Axis& axis);
    static int32_t get_encoder_estimates_callback(const Axis& axis);
    static int32_t get_encoder_count_callback(const Axis& axis);
    static int32_t get_iq_callback(const Axis& axis);
    static int32_t get_sensorless_estimates_callback(const Axis& axis);
    static int32_t get_vbus_voltage_callback(const Axis& axis);

    // Set functions
    static void set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_pos_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_vel_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_torque_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_modes_callback(Axis& axis, const can_Message_t& msg);
    static void set_vel_limit_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_inertia_callback(Axis& axis, const can_Message_t& msg);
    static void set_linear_count_callback(Axis& axis, const can_Message_t& msg);
    static void enable_periodic_update(Axis& axis, const can_Message_t& msg);

    // Can TPDO similar
    static void periodic_handler_update(Axis::CANConfig_t& can_conf, const uint32_t id, const uint32_t command_id, const uint32_t rate_ms, bool construct);
    static int32_t call_periodic_handler(Axis& axis, uint32_t command);

    // Other functions
    static void nmt_callback(const Axis& axis, const can_Message_t& msg);
    static void estop_callback(Axis& axis, const can_Message_t& msg);
    static void clear_errors_callback(Axis& axis, const can_Message_t& msg);
    static void start_anticogging_callback(const Axis& axis, const can_Message_t& msg);

    static constexpr uint8_t NUM_NODE_ID_BITS = 6;
    static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

    // Utility functions
    static constexpr uint32_t get_node_id(uint32_t msgID) {
        return (msgID >> NUM_CMD_ID_BITS);  // Upper 6 or more bits
    };

    static constexpr uint8_t get_cmd_id(uint32_t msgID) {
        return (msgID & 0x01F);  // Bottom 5 bits
    }
};

#endif
