
#include "low_level.h"
#include "protocol.h"

#include <memory>

Endpoint endpoints[] = {
    Endpoint("vbus_voltage", static_cast<const float>(vbus_voltage)),
    Endpoint("elec_rad_per_enc", elec_rad_per_enc),
    Endpoint("motor0", BEGIN_TREE, nullptr, nullptr, nullptr),
    Endpoint("pos_setpoint", motors[0].pos_setpoint),
    Endpoint("pos_gain", motors[0].pos_gain),
    Endpoint("vel_setpoint", motors[0].vel_setpoint),
    Endpoint(nullptr, END_TREE, nullptr, nullptr, nullptr) // motor0
};

constexpr size_t NUM_ENDPOINTS = sizeof(endpoints) / sizeof(endpoints[0]);

/* Monitoring */
size_t monitoring_slots[20] = {0};
constexpr size_t NUM_MONITORING_SLOTS = sizeof(monitoring_slots) / sizeof(monitoring_slots[0]);


void Protocol_print_json(void) {
    bool need_comma = false;
    printf("[");
    for (size_t i = 0; i < NUM_ENDPOINTS; ++i) {
        endpoints[i].print_json(i, need_comma);
    }
    printf("]");
}

void Protocol_print_monitoring(size_t limit) {
    for (size_t i = 0; i < limit; ++i) {
        if (monitoring_slots[i] < NUM_ENDPOINTS) {
            endpoints[monitoring_slots[i]].print_value();
        }
        printf("\t");
    }
    printf("\n");
}

void Protocol_parse_cmd(uint8_t* buffer, int len) {

    // TODO very hacky way of terminating sscanf at end of buffer:
    // We should do some proper struct packing instead of using sscanf altogether
    buffer[len] = 0;

    // check incoming packet type
    if (buffer[0] == 'p') {
        // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &current_feed_forward);
        if (numscan == 4 && motor_number < num_motors) {
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'v') {
        // velocity control
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "v %u %f %f", &motor_number, &vel_feed_forward, &current_feed_forward);
        if (numscan == 3 && motor_number < num_motors) {
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'c') {
        // current control
        unsigned motor_number;
        float current_feed_forward;
        int numscan = sscanf((const char*)buffer, "c %u %f", &motor_number, &current_feed_forward);
        if (numscan == 2 && motor_number < num_motors) {
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        }
    } else if (buffer[0] == 'j') {
        // Read JSON interface definition
        Protocol_print_json();
    } else if (buffer[0] == 'w') { // WRITE
        // s index value
        size_t index = 0;
        size_t pos = 0;
        int numscan = sscanf((const char*)buffer, "s %u %n", &index, &pos);
        if (numscan == 1) {
            if (index < NUM_ENDPOINTS) {
                endpoints[index].scan_value((const char*)buffer + pos);
            }
        }
    } else if (buffer[0] == 'r') { // READ
        // r index
        size_t index = 0;
        int numscan = sscanf((const char*)buffer, "r %u", &index);
        if (numscan == 1) {
            if (index < NUM_ENDPOINTS) {
                endpoints[index].print_value();
            }
        }
    } else if (buffer[0] == 'm') { // Setup Monitor
        // m index monitoring_slot
        size_t index = 0;
        size_t slot = 0;
        int numscan = sscanf((const char*)buffer, "m %u %u", &index, &slot);
        if (numscan == 2) {
            if (index < NUM_ENDPOINTS && slot < NUM_MONITORING_SLOTS) {
                monitoring_slots[slot] = index;
            }
        }
    } else if (buffer[0] == 'o') { // Output Monitor
        size_t limit = 0;
        int numscan = sscanf((const char*)buffer, "o %u", &limit);
        if (numscan == 1) {
            Protocol_print_monitoring(limit);
        }
    }
}
