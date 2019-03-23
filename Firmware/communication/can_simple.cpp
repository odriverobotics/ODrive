
#include "can_simple.hpp"
#include <odrive_main.h>

#include <cstring>

static const uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

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
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(msg.id);
    uint32_t cmd = get_cmd_id(msg.id);

    Axis* axis = nullptr;

    bool validAxis = false;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axes[i]->config_.can_node_id == nodeID) {
            axis = axes[i];
            if (!validAxis) {
                validAxis = true;
            } else {
                // Duplicate can IDs, don't assign to any axis
                odCAN->set_error(ODriveCAN::ERROR_DUPLICATE_CAN_IDS);
                validAxis = false;
                break;
            }
        }
    }

    if (validAxis) {
        axis->watchdog_feed();
        switch (cmd) {
            case MSG_CO_NMT_CTRL:
                break;
            case MSG_CO_HEARTBEAT_CMD:
                break;
            case MSG_ODRIVE_HEARTBEAT:
                // We don't currently do anything to respond to ODrive heartbeat messages
                break;
            case MSG_ODRIVE_ESTOP:
                estop_callback(axis, msg);
                break;
            case MSG_GET_MOTOR_ERROR:
                get_motor_error_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_ERROR:
                get_encoder_error_callback(axis, msg);
                break;
            case MSG_GET_SENSORLESS_ERROR:
                get_sensorless_error_callback(axis, msg);
                break;
            case MSG_SET_AXIS_NODE_ID:
                set_axis_nodeid_callback(axis, msg);
                break;
            case MSG_SET_AXIS_REQUESTED_STATE:
                set_axis_requested_state_callback(axis, msg);
                break;
            case MSG_SET_AXIS_STARTUP_CONFIG:
                set_axis_startup_config_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_ESTIMATES:
                get_encoder_estimates_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_COUNT:
                get_encoder_count_callback(axis, msg);
                break;
            case MSG_MOVE_TO_POS:
                move_to_pos_callback(axis, msg);
                break;
            case MSG_SET_POS_SETPOINT:
                set_pos_setpoint_callback(axis, msg);
                break;
            case MSG_SET_VEL_SETPOINT:
                set_vel_setpoint_callback(axis, msg);
                break;
            case MSG_SET_CUR_SETPOINT:
                set_current_setpoint_callback(axis, msg);
                break;
            case MSG_SET_VEL_LIMIT:
                set_vel_limit_callback(axis, msg);
                break;
            case MSG_START_ANTICOGGING:
                start_anticogging_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_A_PER_CSS:
                set_traj_A_per_css_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_ACCEL_LIMITS:
                set_traj_accel_limits_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_VEL_LIMIT:
                set_traj_vel_limit_callback(axis, msg);
                break;
            case MSG_GET_IQ:
                get_iq_callback(axis, msg);
                break;
            case MSG_GET_SENSORLESS_ESTIMATES:
                get_sensorless_estimates_callback(axis, msg);
                break;
            case MSG_RESET_ODRIVE:
                NVIC_SystemReset();
                break;
            case MSG_GET_VBUS_VOLTAGE:
                get_vbus_voltage_callback(axis, msg);
                break;
            default:
                break;
        }
    }
}

void CANSimple::nmt_callback(Axis* axis, CAN_message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis* axis, CAN_message_t& msg) {
    axis->error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

void CANSimple::get_motor_error_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_MOTOR_ERROR;  // heartbeat ID
        txmsg.isExt = false;
        txmsg.len = 8;

        txmsg.buf[0] = axis->motor_.error_;
        txmsg.buf[1] = axis->motor_.error_ >> 8;
        txmsg.buf[2] = axis->motor_.error_ >> 16;
        txmsg.buf[3] = axis->motor_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_error_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_ERROR;  // heartbeat ID
        txmsg.isExt = false;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.error_;
        txmsg.buf[1] = axis->encoder_.error_ >> 8;
        txmsg.buf[2] = axis->encoder_.error_ >> 16;
        txmsg.buf[3] = axis->encoder_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_error_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
        txmsg.isExt = false;
        txmsg.len = 8;

        txmsg.buf[0] = axis->sensorless_estimator_.error_;
        txmsg.buf[1] = axis->sensorless_estimator_.error_ >> 8;
        txmsg.buf[2] = axis->sensorless_estimator_.error_ >> 16;
        txmsg.buf[3] = axis->sensorless_estimator_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_axis_nodeid_callback(Axis* axis, CAN_message_t& msg) {
    axis->config_.can_node_id = msg.buf[0] & 0x3F;  // Node ID bitmask
}

void CANSimple::set_axis_requested_state_callback(Axis* axis, CAN_message_t& msg) {
    axis->requested_state_ = static_cast<Axis::State_t>(get_16bit_val(msg, 0));
}
void CANSimple::set_axis_startup_config_callback(Axis* axis, CAN_message_t& msg) {
    // Not Implemented
}

void CANSimple::get_encoder_estimates_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
        txmsg.isExt = false;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->encoder_.pos_estimate_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->encoder_.pos_estimate_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->encoder_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->encoder_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_estimates_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
        txmsg.isExt = false;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->sensorless_estimator_.pll_pos_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.pll_pos_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->sensorless_estimator_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_count_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_COUNT;
        txmsg.isExt = false;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.shadow_count_;
        txmsg.buf[1] = axis->encoder_.shadow_count_ >> 8;
        txmsg.buf[2] = axis->encoder_.shadow_count_ >> 16;
        txmsg.buf[3] = axis->encoder_.shadow_count_ >> 24;

        txmsg.buf[4] = axis->encoder_.count_in_cpr_;
        txmsg.buf[5] = axis->encoder_.count_in_cpr_ >> 8;
        txmsg.buf[6] = axis->encoder_.count_in_cpr_ >> 16;
        txmsg.buf[7] = axis->encoder_.count_in_cpr_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::move_to_pos_callback(Axis* axis, CAN_message_t& msg) {
    axis->controller_.move_to_pos(get_32bit_val(msg, 0));
}

void CANSimple::set_pos_setpoint_callback(Axis* axis, CAN_message_t& msg) {
    axis->controller_.set_pos_setpoint(get_32bit_val(msg, 0), get_16bit_val(msg, 4) * 0.1f, get_16bit_val(msg, 6) * 0.01f);
}

void CANSimple::set_vel_setpoint_callback(Axis* axis, CAN_message_t& msg) {
    axis->controller_.set_vel_setpoint(get_32bit_val(msg, 0) * 0.01f, get_32bit_val(msg, 4) * 0.01f);
}

void CANSimple::set_current_setpoint_callback(Axis* axis, CAN_message_t& msg) {
    axis->controller_.set_current_setpoint(get_32bit_val(msg, 0) * 0.01f);
}

void CANSimple::set_vel_limit_callback(Axis* axis, CAN_message_t& msg) {
    axis->controller_.config_.vel_limit = get_float(msg, 0);
}

void CANSimple::start_anticogging_callback(Axis* axis, CAN_message_t& msg) {
    axis->controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis* axis, CAN_message_t& msg) {
    axis->trap_.config_.vel_limit = get_float(msg, 0);
}

void CANSimple::set_traj_accel_limits_callback(Axis* axis, CAN_message_t& msg) {
    axis->trap_.config_.accel_limit = get_float(msg, 0);
    axis->trap_.config_.decel_limit = get_float(msg, 4);
}

void CANSimple::set_traj_A_per_css_callback(Axis* axis, CAN_message_t& msg) {
    axis->trap_.config_.A_per_css = get_float(msg, 0);
}

void CANSimple::get_iq_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_IQ;
        txmsg.isExt = false;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof axis->motor_.current_control_.Iq_setpoint == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_setpoint, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->motor_.current_control_.Iq_measured);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_measured, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_vbus_voltage_callback(Axis* axis, CAN_message_t& msg) {
    if (msg.rtr) {
        CAN_message_t txmsg;

        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_VBUS_VOLTAGE;
        txmsg.isExt = false;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof vbus_voltage == sizeof floatBytes);
        std::memcpy(&floatBytes, &vbus_voltage, sizeof floatBytes);

        // This also works in principle, but I don't have hardware to verify endianness
        // std::memcpy(&txmsg.buf[0], &vbus_voltage, sizeof vbus_voltage);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        txmsg.buf[4] = 0;
        txmsg.buf[5] = 0;
        txmsg.buf[6] = 0;
        txmsg.buf[7] = 0;

        odCAN->write(txmsg);
    }
}

void CANSimple::send_heartbeat(Axis* axis) {
    CAN_message_t txmsg;
    txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
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

uint8_t CANSimple::get_node_id(uint32_t msgID) {
    return ((msgID >> NUM_CMD_ID_BITS) & 0x03F);  // Upper 6 bits
}

uint8_t CANSimple::get_cmd_id(uint32_t msgID) {
    return (msgID & 0x01F);  // Bottom 5 bits
}

int16_t CANSimple::get_16bit_val(CAN_message_t& msg, uint8_t start_byte) {
    int16_t retVal = 0;
    if (msg.len >= start_byte && size_t(msg.len - start_byte) >= sizeof(retVal))
        std::memcpy(&retVal, &(msg.buf[start_byte]), sizeof(retVal));
    return retVal;
}

int32_t CANSimple::get_32bit_val(CAN_message_t& msg, uint8_t start_byte) {
    int32_t retVal = 0;
    if (msg.len >= start_byte && size_t(msg.len - start_byte) >= sizeof(retVal))
        std::memcpy(&retVal, &(msg.buf[start_byte]), sizeof(retVal));
    return retVal;
}

float CANSimple::get_float(CAN_message_t& msg, uint8_t start_byte) {
    int32_t val = get_32bit_val(msg, start_byte);
    float retVal;

    static_assert(sizeof retVal == sizeof val);
    std::memcpy(&retVal, &val, sizeof val);  // Sexier int32_t -> float cast that isn't UB
    return retVal;
}