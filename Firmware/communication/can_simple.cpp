
#include "can_simple.hpp"

void CANSimple::handle_can_message(CAN_message_t& msg) {
    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking to fix the syntax.
    //
    // auto func = callback_map.find(msg.id);
    // if(func != callback_map.end()){
    //     func->second(msg);
    // }

    //     Frame
    // nodeID | CMD
    // 4 bits | 7 bits
    auto nodeID = (msg.id >> 7 & 0x15);
    Axis* axis = nullptr;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axes[i]->config_.can_node_id == nodeID) {
            axis = axes[i];
        }
    }
    if (axis != nullptr) {
        switch (msg.id & 0x7F) {
            case 0x020:
                move_to_pos_callback(axis, msg);
                break;
            case 0x021:
                set_pos_setpoint_callback(axis, msg);
                break;
            case 0x022:
                set_vel_setpoint_callback(axis, msg);
                break;
            case 0x023:
                set_current_setpoint_callback(axis, msg);
                break;
        }
    }
}

void CANSimple::estop_callback(){
    for(Axis* axis : axes){
        axis->error_ |= Axis::ERROR_ESTOP_REQUESTED;
    }
}

void CANSimple::move_to_pos_callback(Axis* axis, CAN_message_t& msg) {
    float pos = msg.buf[0];
    pos += msg.buf[1] << 8;
    pos += msg.buf[2] << 16;
    pos += msg.buf[3] << 24;

    axis->controller_.move_to_pos(pos);
}

void CANSimple::set_pos_setpoint_callback(Axis* axis, CAN_message_t& msg) {
    float pos = msg.buf[0];
    pos += msg.buf[1] << 8;
    pos += msg.buf[2] << 16;
    pos += msg.buf[3] << 24;

    float vel = msg.buf[4];
    vel += msg.buf[5] << 8;
    vel *= 0.1f;  // Factor of 10

    float current = msg.buf[6];
    current += (msg.buf[7] << 8);
    current *= 0.01f;  // Factor of 100

    axis->controller_.set_pos_setpoint(pos, vel, current);
}

void CANSimple::set_vel_setpoint_callback(Axis* axis, CAN_message_t& msg) {
    float vel = msg.buf[0];
    vel += msg.buf[1] << 8;
    vel += msg.buf[2] << 16;
    vel += msg.buf[3] << 24;
    vel *= 0.01f;

    float current = msg.buf[4];
    current += msg.buf[5] << 8;
    current += msg.buf[6] << 16;
    current += msg.buf[7] << 24;
    current *= 0.01f;

    axis->controller_.set_vel_setpoint(vel, current);
}

void CANSimple::set_current_setpoint_callback(Axis* axis, CAN_message_t& msg) {
    float current = msg.buf[0];
    current += msg.buf[1] << 8;
    current += msg.buf[2] << 16;
    current += msg.buf[3] << 24;
    current *= 0.01f;

    axis->controller_.set_current_setpoint(current);
}

void CANSimple::send_heartbeat(Axis* axis){
    CAN_message_t txmsg;
    txmsg.id = axis->config_.can_node_id << 7;
    txmsg.id += 0x001; // heartbeat ID
    txmsg.isExt = false;
    txmsg.len = 8;
    
    // Axis errors in 1st 32-bit value
    txmsg.buf[0] = axis->error_;
    txmsg.buf[1] = axis->error_ >> 8;
    txmsg.buf[2] = axis->error_ >> 16;
    txmsg.buf[3] = axis->error_ >> 24;

    // Current state of axis in 2nd 32-bit value
    txmsg.buf[4] = axis->current_state_;
    txmsg.buf[5] = axis->current_state_ >> 8;
    txmsg.buf[6] = axis->current_state_ >> 16;
    txmsg.buf[7] = axis->current_state_ >> 24;
    odCAN->write(txmsg);
}