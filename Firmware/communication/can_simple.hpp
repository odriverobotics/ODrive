#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
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