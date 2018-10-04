#include "can_simple.hpp"
#include "odrive_main.h"

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
    for(int i = 0; i < AXIS_COUNT; i++){
        if(axes[i]->config_.nodeID)
    }
    switch (msg.id & 0x7F) {
        case 0x010: move_to_pos_callback(); break;
    }
}

void move_to_pos_callback(Axis& axis, uint32_t pos){
}