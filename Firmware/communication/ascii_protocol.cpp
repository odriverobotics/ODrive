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
        int numscan = sscanf((const char *)cmd + checksum_start, "%u", &received_checksum);
        if ((numscan < 1) || (received_checksum != checksum))
            return;
        len = checksum_start - 1; // prune checksum and asterisk
        cmd[len] = 0; // null-terminate
    }


    // check incoming packet type
    if (cmd[0] == 'p') { // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, torque_feed_forward;
        int numscan = sscanf(cmd, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &torque_feed_forward);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            axis->controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            axis->controller_.input_pos_ = pos_setpoint;
            if (numscan >= 3)
                axis->controller_.input_vel_ = vel_feed_forward;
            if (numscan >= 4)
                axis->controller_.input_torque_ = torque_feed_forward;
            axis->controller_.input_pos_updated();
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'q') { // position control with limits
        unsigned motor_number;
        float pos_setpoint, vel_limit, torque_lim;
        int numscan = sscanf(cmd, "q %u %f %f %f", &motor_number, &pos_setpoint, &vel_limit, &torque_lim);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            axis->controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            axis->controller_.input_pos_ = pos_setpoint;
            if (numscan >= 3)
                axis->controller_.config_.vel_limit = vel_limit;
            if (numscan >= 4)
                axis->motor_.config_.torque_lim = torque_lim;
            axis->controller_.input_pos_updated();
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'v') { // velocity control
        unsigned motor_number;
        float vel_setpoint, torque_feed_forward;
        int numscan = sscanf(cmd, "v %u %f %f", &motor_number, &vel_setpoint, &torque_feed_forward);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            axis->controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
            axis->controller_.input_vel_ = vel_setpoint;
            if (numscan >= 3)
                axis->controller_.input_torque_ = torque_feed_forward;
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'c') { // torque control
        unsigned motor_number;
        float torque_setpoint;
        int numscan = sscanf(cmd, "c %u %f", &motor_number, &torque_setpoint);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            axis->controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
            axis->controller_.input_torque_ = torque_setpoint;
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 't') { // trapezoidal trajectory
        unsigned motor_number;
        float goal_point;
        int numscan = sscanf(cmd, "t %u %f", &motor_number, &goal_point);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            axis->controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
            axis->controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            axis->controller_.input_pos_ = goal_point;
            axis->controller_.input_pos_updated();
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'f') { // feedback
        unsigned motor_number;
        int numscan = sscanf(cmd, "f %u", &motor_number);
        if (numscan < 1) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            respond(response_channel, use_checksum, "%f %f",
                    (double)axes[motor_number]->encoder_.pos_estimate_,
                    (double)axes[motor_number]->encoder_.vel_estimate_);
        }

    } else if (cmd[0] == 'h') {  // Help
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

    } else if (cmd[0] == 'i'){ // Dump device info
        // respond(response_channel, use_checksum, "Signature: %#x", STM_ID_GetSignature());
        // respond(response_channel, use_checksum, "Revision: %#x", STM_ID_GetRevision());
        // respond(response_channel, use_checksum, "Flash Size: %#x KiB", STM_ID_GetFlashSize());
        respond(response_channel, use_checksum, "Hardware version: %d.%d-%dV", odrv.hw_version_major_, odrv.hw_version_minor_, odrv.hw_version_variant_);
        respond(response_channel, use_checksum, "Firmware version: %d.%d.%d", odrv.fw_version_major_, odrv.fw_version_minor_, odrv.fw_version_revision_);
        respond(response_channel, use_checksum, "Serial number: %s", serial_number_str);

    } else if (cmd[0] == 's'){ // System
        if(cmd[1] == 's') { // Save config
            odrv.save_configuration();
        } else if (cmd[1] == 'e'){ // Erase config
            odrv.erase_configuration();
        } else if (cmd[1] == 'r'){ // Reboot
            odrv.reboot();
        }

    } else if (cmd[0] == 'r') { // read property
        char name[MAX_LINE_LENGTH];
        int numscan = sscanf(cmd, "r %255s", name);
        if (numscan < 1) {
            respond(response_channel, use_checksum, "invalid command format");
        } else {
            Introspectable property = root_obj.get_child(name, sizeof(name));
            const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
            if (!type_info) {
                respond(response_channel, use_checksum, "invalid property");
            } else {
                char response[10];
                bool success = type_info->get_string(property, response, sizeof(response));
                if (!success)
                    respond(response_channel, use_checksum, "not implemented");
                else
                    respond(response_channel, use_checksum, response);
            }
        }

    } else if (cmd[0] == 'w') { // write property
        char name[MAX_LINE_LENGTH];
        char value[MAX_LINE_LENGTH];
        int numscan = sscanf(cmd, "w %255s %255s", name, value);
        if (numscan < 1) {
            respond(response_channel, use_checksum, "invalid command format");
        } else {
            Introspectable property = root_obj.get_child(name, sizeof(name));
            const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
            if (!type_info) {
                respond(response_channel, use_checksum, "invalid property");
            } else {
                bool success = type_info->set_string(property, value, sizeof(value));
                if (!success)
                    respond(response_channel, use_checksum, "not implemented");
            }
        }

    } else if (cmd[0] == 'u') { // Update axis watchdog. 
        unsigned motor_number;
        int numscan = sscanf(cmd, "u %u", &motor_number);
        if(numscan < 1){
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        }else {
            axes[motor_number]->watchdog_feed();
        }

    } else if (cmd[0] != 0) {
        respond(response_channel, use_checksum, "unknown command");
    }
}

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
