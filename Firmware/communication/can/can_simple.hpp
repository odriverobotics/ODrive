#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include <interfaces/canbus.hpp>
#include "axis.hpp"

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
        MSG_SET_LIMITS,
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

    CANSimple(CanBusBase* canbus) : canbus_(canbus) {}

    bool init(fibre::Callback<bool, float, fibre::Callback<void>> timer, uint32_t tx_slots_start, uint32_t tx_slots_end, uint32_t rx_slot);

   private:
    struct PeriodicHandler {
        CANSimple* parent_;
        Axis* axis;
        int type;
        uint32_t* interval;
        uint32_t last;

        void trigger() { parent_->send_periodic(this); }
    };

    bool renew_subscription(size_t i);
    void on_received(const can_Message_t& msg);
    //void on_sent(bool success);
    void send_periodic(PeriodicHandler* handler);

    void do_command(Axis& axis, const can_Message_t& cmd);
    
    // Get functions (msg.rtr bit must be set)
    void get_heartbeat(const Axis& axis, can_Message_t& txmsg);
    void get_motor_error_callback(const Axis& axis, can_Message_t& txmsg);
    void get_encoder_error_callback(const Axis& axis, can_Message_t& txmsg);
    void get_sensorless_error_callback(const Axis& axis, can_Message_t& txmsg);
    void get_encoder_estimates_callback(const Axis& axis, can_Message_t& txmsg);
    void get_encoder_count_callback(const Axis& axis, can_Message_t& txmsg);
    void get_iq_callback(const Axis& axis, can_Message_t& txmsg);
    void get_sensorless_estimates_callback(const Axis& axis, can_Message_t& txmsg);
    void get_vbus_voltage_callback(const Axis& axis, can_Message_t& txmsg);

    // Set functions
    static void set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_pos_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_vel_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_torque_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_modes_callback(Axis& axis, const can_Message_t& msg);
    static void set_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_inertia_callback(Axis& axis, const can_Message_t& msg);
    static void set_linear_count_callback(Axis& axis, const can_Message_t& msg);

    // Other functions
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

    CanBusBase* canbus_;
    CanBusBase::CanSubscription* subscription_handles_[AXIS_COUNT];

    fibre::Callback<bool, float, fibre::Callback<void>> timer_;
    uint32_t tx_slots_start_;
    uint32_t tx_slots_end_;
    uint32_t response_tx_slot_;
    uint32_t rx_slot_;

    std::array<PeriodicHandler, 2 * AXIS_COUNT> periodic_handlers_;
};

#endif