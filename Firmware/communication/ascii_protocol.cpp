/*
* The ASCII protocol is a simpler, human readable alternative to the main native
* protocol.
* In the future this protocol might be extended to support selected GCode commands.
* For a list of supported commands see doc/ascii-protocol.md
*/

/* Includes ------------------------------------------------------------------*/

#include "odrive_main.h"
#include "communication.h"
#include "ascii_protocol.hpp"
#include <utils.hpp>
#include <fibre/cpp_utils.hpp>

#include "autogen/type_info.hpp"
#include "communication/interface_can.hpp"

using namespace fibre;

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)

/* Private variables ---------------------------------------------------------*/

#if HW_VERSION_MAJOR == 3
static Introspectable root_obj = ODrive3TypeInfo<ODrive>::make_introspectable(odrv);
#elif HW_VERSION_MAJOR == 4
static Introspectable root_obj = ODrive4TypeInfo<ODrive>::make_introspectable(odrv);
#endif

/* Private function prototypes -----------------------------------------------*/

/* Function implementations --------------------------------------------------*/

// @brief Sends a line on the specified output.
template<typename ... TArgs>
void AsciiProtocol::respond(bool include_checksum, const char * fmt, TArgs&& ... args) {
    char tx_buf[64];

    size_t len = snprintf(tx_buf, sizeof(tx_buf), fmt, std::forward<TArgs>(args)...);

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= tx_buf[i];
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "*%u\r\n", checksum);
    } else {
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "\r\n");
    }

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    sink_.write({(const uint8_t*)tx_buf, len});
    sink_.maybe_start_async_write();
}


// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void AsciiProtocol::process_line(cbufptr_t buffer) {
    static_assert(sizeof(char) == sizeof(uint8_t));
    
    // scan line to find beginning of checksum and prune comment
    uint8_t checksum = 0;
    size_t checksum_start = SIZE_MAX;
    for (size_t i = 0; i < buffer.size(); ++i) {
        if (buffer.begin()[i] == ';') { // ';' is the comment start char
            buffer = buffer.take(i);
            break;
        }
        if (checksum_start > i) {
            if (buffer[i] == '*') {
                checksum_start = i + 1;
            } else {
                checksum ^= buffer[i];
            }
        }
    }

    // copy everything into a local buffer so we can insert null-termination
    char cmd[MAX_LINE_LENGTH + 1];
    size_t len = std::min(buffer.size(), MAX_LINE_LENGTH);
    memcpy(cmd, buffer.begin(), len);
    cmd[len] = 0; // null-terminate

    // optional checksum validation
    bool use_checksum = (checksum_start < len);
    if (use_checksum) {
        unsigned int received_checksum;
        int numscan = sscanf(&cmd[checksum_start], "%u", &received_checksum);
        if ((numscan < 1) || (received_checksum != checksum))
            return;
        len = checksum_start - 1; // prune checksum and asterisk
        cmd[len] = 0; // null-terminate
    }


    // check incoming packet type
    switch(cmd[0]) {
        case 'p': cmd_set_position(cmd, use_checksum);                break;  // position control
        case 'q': cmd_set_position_wl(cmd, use_checksum);             break;  // position control with limits
        case 'v': cmd_set_velocity(cmd, use_checksum);                break;  // velocity control
        case 'c': cmd_set_torque(cmd, use_checksum);                  break;  // current control
        case 't': cmd_set_trapezoid_trajectory(cmd, use_checksum);    break;  // trapezoidal trajectory
        case 'f': cmd_get_feedback(cmd, use_checksum);                break;  // feedback
        case 'h': cmd_help(cmd, use_checksum);                        break;  // Help
        case 'i': cmd_info_dump(cmd, use_checksum);                   break;  // Dump device info
        case 's': cmd_system_ctrl(cmd, use_checksum);                 break;  // System
        case 'r': cmd_read_property(cmd,  use_checksum);              break;  // read property
        case 'w': cmd_write_property(cmd, use_checksum);              break;  // write property
        case 'u': cmd_update_axis_wdg(cmd, use_checksum);             break;  // Update axis watchdog. 
        case 'e': cmd_encoder(cmd, use_checksum);                     break;  // Encoder commands
        default : cmd_unknown(nullptr, use_checksum);                 break;
    }
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_position(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_feed_forward, torque_feed_forward;

    int numscan = sscanf(pStr, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &torque_feed_forward);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = pos_setpoint;
        if (numscan >= 3)
            axis.controller_.input_vel_ = vel_feed_forward;
        if (numscan >= 4)
            axis.controller_.input_torque_ = torque_feed_forward;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the set position with current and velocity limit command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_position_wl(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_limit, torque_lim;

    int numscan = sscanf(pStr, "q %u %f %f %f", &motor_number, &pos_setpoint, &vel_limit, &torque_lim);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = pos_setpoint;
        if (numscan >= 3)
            axis.controller_.config_.vel_limit = vel_limit;
        if (numscan >= 4)
            axis.motor_.config_.torque_lim = torque_lim;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the set velocity command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_velocity(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float vel_setpoint, torque_feed_forward;
    int numscan = sscanf(pStr, "v %u %f %f", &motor_number, &vel_setpoint, &torque_feed_forward);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
        axis.controller_.input_vel_ = vel_setpoint;
        if (numscan >= 3)
            axis.controller_.input_torque_ = torque_feed_forward;
        axis.watchdog_feed();
    }
}

// @brief Executes the set torque control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_torque(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float torque_setpoint;

    if (sscanf(pStr, "c %u %f", &motor_number, &torque_setpoint) < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();
    }
}

// @brief Sets the encoder linear count
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_encoder(char * pStr, bool use_checksum) {
    if (pStr[1] == 's') {
        pStr += 2; // Substring two characters to the right (ok because we have guaranteed null termination after all chars)

        unsigned motor_number;
        int encoder_count;

        if (sscanf(pStr, "l %u %i", &motor_number, &encoder_count) < 2) {
            respond(use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis& axis = axes[motor_number];
            axis.encoder_.set_linear_count(encoder_count);
            axis.watchdog_feed();
            respond(use_checksum, "encoder set to %u", encoder_count);
        }
    } else {
        respond(use_checksum, "invalid command format");
    }
}

// @brief Executes the set trapezoid trajectory command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_trapezoid_trajectory(char* pStr, bool use_checksum) {
    unsigned motor_number;
    float goal_point;

    if (sscanf(pStr, "t %u %f", &motor_number, &goal_point) < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = goal_point;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the get position and velocity feedback command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_get_feedback(char * pStr, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "f %u", &motor_number) < 1) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        respond(use_checksum, "%f %f",
                (double)axis.encoder_.pos_estimate_.any().value_or(0.0f),
                (double)axis.encoder_.vel_estimate_.any().value_or(0.0f));
    }
}

// @brief Shows help text
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_help(char * pStr, bool use_checksum) {
    (void)pStr;
    respond(use_checksum, "Please see documentation for more details");
    respond(use_checksum, "");
    respond(use_checksum, "Available commands syntax reference:");
    respond(use_checksum, "Position: q axis pos vel-lim I-lim");
    respond(use_checksum, "Position: p axis pos vel-ff I-ff");
    respond(use_checksum, "Velocity: v axis vel I-ff");
    respond(use_checksum, "Torque: c axis T");
    respond(use_checksum, "");
    respond(use_checksum, "Properties start at odrive root, such as axis0.requested_state");
    respond(use_checksum, "Read: r property");
    respond(use_checksum, "Write: w property value");
    respond(use_checksum, "");
    respond(use_checksum, "Save config: ss");
    respond(use_checksum, "Erase config: se");
    respond(use_checksum, "Reboot: sr");
}

// @brief Gets the hardware, firmware and serial details
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_info_dump(char * pStr, bool use_checksum) {
    // respond(use_checksum, "Signature: %#x", STM_ID_GetSignature());
    // respond(use_checksum, "Revision: %#x", STM_ID_GetRevision());
    // respond(use_checksum, "Flash Size: %#x KiB", STM_ID_GetFlashSize());
    respond(use_checksum, "Hardware version: %d.%d-%dV", odrv.hw_version_major_, odrv.hw_version_minor_, odrv.hw_version_variant_);
    respond(use_checksum, "Firmware version: %d.%d.%d", odrv.fw_version_major_, odrv.fw_version_minor_, odrv.fw_version_revision_);
    respond(use_checksum, "Serial number: %s", serial_number_str);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_system_ctrl(char * pStr, bool use_checksum) {
    switch (pStr[1])
    {
        case 's':   odrv.save_configuration();  break;  // Save config
        case 'e':   odrv.erase_configuration(); break;  // Erase config
        case 'r':   odrv.reboot();              break;  // Reboot
        case 'c':   odrv.clear_errors();        break;  // clear all errors and rearm brake resistor if necessary
        default:    /* default */               break;
    }
}

// @brief Executes the read parameter command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_read_property(char * pStr, bool use_checksum) {
    char name[MAX_LINE_LENGTH];

    if (sscanf(pStr, "r %255s", name) < 1) {
        respond(use_checksum, "invalid command format");
    } else {
        Introspectable property = root_obj.get_child(name, sizeof(name));
        const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
        if (!type_info) {
            respond(use_checksum, "invalid property");
        } else {
            char response[10];
            bool success = type_info->get_string(property, response, sizeof(response));
            respond(use_checksum, success ? response : "not implemented");
        }
    }
}

// @brief Executes the set write position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_write_property(char * pStr, bool use_checksum) {
    char name[MAX_LINE_LENGTH];
    char value[MAX_LINE_LENGTH];

    if (sscanf(pStr, "w %255s %255s", name, value) < 1) {
        respond(use_checksum, "invalid command format");
    } else {
        Introspectable property = root_obj.get_child(name, sizeof(name));
        const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
        if (!type_info) {
            respond(use_checksum, "invalid property");
        } else {
            bool success = type_info->set_string(property, value, sizeof(value));
            if (!success) {
                respond(use_checksum, "not implemented");
            }
        }
    }
}

// @brief Executes the motor watchdog update command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_update_axis_wdg(char * pStr, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "u %u", &motor_number) < 1) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        axes[motor_number].watchdog_feed();
    }
}

// @brief Sends the unknown command response
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_unknown(char * pStr, bool use_checksum) {
    (void)pStr;
    respond(use_checksum, "unknown command");
}

void AsciiProtocol::on_read_finished(ReadResult result) {
    if (result.status != kStreamOk) {
        return;
    }

    for (;;) {
        uint8_t* end_of_line = std::find_if(rx_buf_, result.end, [](uint8_t c) {
            return c == '\r' || c == '\n' || c == '!';
        });

        if (end_of_line >= result.end) {
            break;
        }

        if (read_active_) {
            process_line({rx_buf_, end_of_line});
        } else {
            // Ignoring this line cause it didn't start at a new-line character
            read_active_ = true;
        }
        
        // Discard the processed bytes and shift the remainder to the beginning of the buffer
        size_t n_remaining = result.end - end_of_line - 1;
        memmove(rx_buf_, end_of_line + 1, n_remaining);
        result.end = rx_buf_ + n_remaining;
    }

    // No more new-line characters in buffer

    if (result.end >= rx_buf_ + sizeof(rx_buf_)) {
        // If the line becomes too long, reset buffer and wait for the next line
        result.end = rx_buf_;
        read_active_ = false;
    }

    TransferHandle dummy;
    rx_channel_->start_read({result.end, rx_buf_ + sizeof(rx_buf_)}, &dummy, MEMBER_CB(this, on_read_finished));
}

void AsciiProtocol::start() {
    TransferHandle dummy;
    rx_channel_->start_read(rx_buf_, &dummy, MEMBER_CB(this, on_read_finished));
}
