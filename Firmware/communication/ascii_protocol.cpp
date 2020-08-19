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

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

#define MAX_LINE_LENGTH 256
#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)

/* Private variables ---------------------------------------------------------*/

static Introspectable root_obj = ODriveTypeInfo<ODrive>::make_introspectable(odrv);

/* Private function prototypes -----------------------------------------------*/

void cmd_set_position(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_set_position_wl(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_set_velocity(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_set_torque(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_set_trapezoid_trajectory(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_get_feedback(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_help(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_info_dump(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_system_ctrl(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_read_property(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_write_property(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_update_axis_wdg(char * pStr, StreamSink& response_channel, bool use_checksum);
void cmd_unknown(char * pStr, StreamSink& response_channel, bool use_checksum);

/* Function implementations --------------------------------------------------*/

// @brief Sends a line on the specified output.
template<typename ... TArgs>
void respond(StreamSink& output, bool include_checksum, const char * fmt, TArgs&& ... args) {
    char response[64]; // Hardcoded max buffer size. We silently truncate the output if it's too long for the buffer.
    size_t len = snprintf(response, sizeof(response), fmt, std::forward<TArgs>(args)...);
    len = std::min(len, sizeof(response));
    output.process_bytes((uint8_t*)response, len, nullptr); // TODO: use process_all instead
    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= response[i];
        len = snprintf(response, sizeof(response), "*%u", checksum);
        len = std::min(len, sizeof(response));
        output.process_bytes((uint8_t*)response, len, nullptr);
    }
    output.process_bytes((const uint8_t*)"\r\n", 2, nullptr);
}


// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void ASCII_protocol_process_line(const uint8_t* buffer, size_t len, StreamSink& response_channel) {
    static_assert(sizeof(char) == sizeof(uint8_t));

    // scan line to find beginning of checksum and prune comment
    uint8_t checksum = 0;
    size_t checksum_start = SIZE_MAX;
    for (size_t i = 0; i < len; ++i) {
        if (buffer[i] == ';') { // ';' is the comment start char
            len = i;
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
    if (len > MAX_LINE_LENGTH) len = MAX_LINE_LENGTH;
    memcpy(cmd, buffer, len);
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
        case 'p': cmd_set_position(cmd, response_channel, use_checksum);                break;  // position control
        case 'q': cmd_set_position_wl(cmd, response_channel, use_checksum);             break;  // position control with limits
        case 'v': cmd_set_velocity(cmd, response_channel, use_checksum);                break;  // velocity control
        case 'c': cmd_set_torque(cmd, response_channel, use_checksum);                  break;  // current control
        case 't': cmd_set_trapezoid_trajectory(cmd, response_channel, use_checksum);    break;  // trapezoidal trajectory
        case 'f': cmd_get_feedback(cmd, response_channel, use_checksum);                break;  // feedback
        case 'h': cmd_help(cmd, response_channel, use_checksum);                        break;  // Help
        case 'i': cmd_info_dump(cmd, response_channel, use_checksum);                   break;  // Dump device info
        case 's': cmd_system_ctrl(cmd, response_channel, use_checksum);                 break;  // System
        case 'r': cmd_read_property(cmd, response_channel,  use_checksum);              break;  // read property
        case 'w': cmd_write_property(cmd, response_channel, use_checksum);              break;  // write property
        case 'u': cmd_update_axis_wdg(cmd, response_channel, use_checksum);             break;  // Update axis watchdog. 
        default : cmd_unknown(nullptr, response_channel, use_checksum);                 break;
    }
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_set_position(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_feed_forward, torque_feed_forward;

    int numscan = sscanf(pStr, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &torque_feed_forward);
    if (numscan < 2) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
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
void cmd_set_position_wl(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_limit, torque_lim;

    int numscan = sscanf(pStr, "q %u %f %f %f", &motor_number, &pos_setpoint, &vel_limit, &torque_lim);
    if (numscan < 2) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
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
void cmd_set_velocity(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;
    float vel_setpoint, torque_feed_forward;
    int numscan = sscanf(pStr, "v %u %f %f", &motor_number, &vel_setpoint, &torque_feed_forward);
    if (numscan < 2) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
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
void cmd_set_torque(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;
    float torque_setpoint;

    if (sscanf(pStr, "c %u %f", &motor_number, &torque_setpoint) < 2) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();
    }
}

// @brief Executes the set trapezoid trajectory command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_set_trapezoid_trajectory(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;
    float goal_point;

    if (sscanf(pStr, "t %u %f", &motor_number, &goal_point) < 2) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
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
void cmd_get_feedback(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "f %u", &motor_number) < 1) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        respond(response_channel, use_checksum, "%f %f",
                (double)axis.encoder_.pos_estimate_,
                (double)axis.encoder_.vel_estimate_);
    }
}

// @brief Shows help text
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_help(char * pStr, StreamSink& response_channel, bool use_checksum) {
    (void)pStr;
    respond(response_channel, use_checksum, "Please see documentation for more details");
    respond(response_channel, use_checksum, "");
    respond(response_channel, use_checksum, "Available commands syntax reference:");
    respond(response_channel, use_checksum, "Position: q axis pos vel-lim I-lim");
    respond(response_channel, use_checksum, "Position: p axis pos vel-ff I-ff");
    respond(response_channel, use_checksum, "Velocity: v axis vel I-ff");
    respond(response_channel, use_checksum, "Torque: c axis T");
    respond(response_channel, use_checksum, "");
    respond(response_channel, use_checksum, "Properties start at odrive root, such as axis0.requested_state");
    respond(response_channel, use_checksum, "Read: r property");
    respond(response_channel, use_checksum, "Write: w property value");
    respond(response_channel, use_checksum, "");
    respond(response_channel, use_checksum, "Save config: ss");
    respond(response_channel, use_checksum, "Erase config: se");
    respond(response_channel, use_checksum, "Reboot: sr");
}

// @brief Gets the hardware, firmware and serial details
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_info_dump(char * pStr, StreamSink& response_channel, bool use_checksum) {
    // respond(response_channel, use_checksum, "Signature: %#x", STM_ID_GetSignature());
    // respond(response_channel, use_checksum, "Revision: %#x", STM_ID_GetRevision());
    // respond(response_channel, use_checksum, "Flash Size: %#x KiB", STM_ID_GetFlashSize());
    respond(response_channel, use_checksum, "Hardware version: %d.%d-%dV", odrv.hw_version_major_, odrv.hw_version_minor_, odrv.hw_version_variant_);
    respond(response_channel, use_checksum, "Firmware version: %d.%d.%d", odrv.fw_version_major_, odrv.fw_version_minor_, odrv.fw_version_revision_);
    respond(response_channel, use_checksum, "Serial number: %s", serial_number_str);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_system_ctrl(char * pStr, StreamSink& response_channel, bool use_checksum) {
    switch (pStr[1])
    {
        case 's':   odrv.save_configuration();  break;  // Save config
        case 'e':   odrv.erase_configuration(); break;  // Erase config
        case 'r':   odrv.reboot();              break;  // Reboot
        default:    /* default */               break;
    }
}

// @brief Executes the read parameter command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_read_property(char * pStr, StreamSink& response_channel, bool use_checksum) {
    char name[MAX_LINE_LENGTH];

    if (sscanf(pStr, "r %255s", name) < 1) {
        respond(response_channel, use_checksum, "invalid command format");
    } else {
        Introspectable property = root_obj.get_child(name, sizeof(name));
        const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
        if (!type_info) {
            respond(response_channel, use_checksum, "invalid property");
        } else {
            char response[10];
            bool success = type_info->get_string(property, response, sizeof(response));
            respond(response_channel, use_checksum, success ? response : "not implemented");
        }
    }
}

// @brief Executes the set write position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_write_property(char * pStr, StreamSink& response_channel, bool use_checksum) {
    char name[MAX_LINE_LENGTH];
    char value[MAX_LINE_LENGTH];

    if (sscanf(pStr, "w %255s %255s", name, value) < 1) {
        respond(response_channel, use_checksum, "invalid command format");
    } else {
        Introspectable property = root_obj.get_child(name, sizeof(name));
        const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
        if (!type_info) {
            respond(response_channel, use_checksum, "invalid property");
        } else {
            bool success = type_info->set_string(property, value, sizeof(value));
            if (!success) {
                respond(response_channel, use_checksum, "not implemented");
            }
        }
    }
}

// @brief Executes the motor watchdog update command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_update_axis_wdg(char * pStr, StreamSink& response_channel, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "u %u", &motor_number) < 1) {
        respond(response_channel, use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(response_channel, use_checksum, "invalid motor %u", motor_number);
    } else {
        axes[motor_number].watchdog_feed();
    }
}

// @brief Sends the unknown command response
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void cmd_unknown(char * pStr, StreamSink& response_channel, bool use_checksum) {
    (void)pStr;
    respond(response_channel, use_checksum, "unknown command");
}

// @brief Parses the received ASCII char stream
// @param buffer buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel) {
    static uint8_t parse_buffer[MAX_LINE_LENGTH];
    static bool read_active = true;
    static uint32_t parse_buffer_idx = 0;

    while (len--) {
        // if the line becomes too long, reset buffer and wait for the next line
        if (parse_buffer_idx >= MAX_LINE_LENGTH) {
            read_active = false;
            parse_buffer_idx = 0;
        }

        // Fetch the next char
        uint8_t c = *(buffer++);
        bool is_end_of_line = (c == '\r' || c == '\n' || c == '!');
        if (is_end_of_line) {
            if (read_active)
                ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
            parse_buffer_idx = 0;
            read_active = true;
        } else {
            if (read_active) {
                parse_buffer[parse_buffer_idx++] = c;
            }
        }
    }
}
