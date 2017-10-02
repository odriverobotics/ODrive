
/* Includes ------------------------------------------------------------------*/

#include "low_level.h"
#include "protocol.h"

#include <memory>

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* variables exposed to usb interface via set/get/monitor
 * If you change something here, don't forget to regenerate the python interface with generate_api.py
 * ro/rw : read only/read write -> ro prevents the code generator from generating setter
 * */

float* exposed_floats[] = {
    &vbus_voltage, // ro
    &elec_rad_per_enc, // ro	
    &motors[0].pos_setpoint, // rw
    &motors[0].pos_gain, // rw
    &motors[0].vel_setpoint, // rw
    &motors[0].vel_gain, // rw
    &motors[0].vel_integrator_gain, // rw
    &motors[0].vel_integrator_current, // rw
    &motors[0].vel_limit, // rw
    &motors[0].current_setpoint, // rw
    &motors[0].calibration_current, // rw
    &motors[0].phase_inductance, // ro
    &motors[0].phase_resistance, // ro
    &motors[0].current_meas.phB, // ro
    &motors[0].current_meas.phC, // ro
    &motors[0].DC_calib.phB, // rw
    &motors[0].DC_calib.phC, // rw
    &motors[0].shunt_conductance, // rw
    &motors[0].phase_current_rev_gain, // rw
    &motors[0].current_control.current_lim, // rw
    &motors[0].current_control.p_gain, // rw
    &motors[0].current_control.i_gain, // rw
    &motors[0].current_control.v_current_control_integral_d, // rw
    &motors[0].current_control.v_current_control_integral_q, // rw
    &motors[0].current_control.Ibus, // ro
    &motors[0].encoder.phase, // ro
    &motors[0].encoder.pll_pos, // rw
    &motors[0].encoder.pll_vel, // rw
    &motors[0].encoder.pll_kp, // rw
    &motors[0].encoder.pll_ki, // rw
    &motors[1].pos_setpoint, // rw
    &motors[1].pos_gain, // rw
    &motors[1].vel_setpoint, // rw
    &motors[1].vel_gain, // rw
    &motors[1].vel_integrator_gain, // rw
    &motors[1].vel_integrator_current, // rw
    &motors[1].vel_limit, // rw
    &motors[1].current_setpoint, // rw
    &motors[1].calibration_current, // rw
    &motors[1].phase_inductance, // ro
    &motors[1].phase_resistance, // ro
    &motors[1].current_meas.phB, // ro
    &motors[1].current_meas.phC, // ro
    &motors[1].DC_calib.phB, // rw
    &motors[1].DC_calib.phC, // rw
    &motors[1].shunt_conductance, // rw
    &motors[1].phase_current_rev_gain, // rw
    &motors[1].current_control.current_lim, // rw
    &motors[1].current_control.p_gain, // rw
    &motors[1].current_control.i_gain, // rw
    &motors[1].current_control.v_current_control_integral_d, // rw
    &motors[1].current_control.v_current_control_integral_q, // rw
    &motors[1].current_control.Ibus, // ro
    &motors[1].encoder.phase, // ro
    &motors[1].encoder.pll_pos, // rw
    &motors[1].encoder.pll_vel, // rw
    &motors[1].encoder.pll_kp, // rw
    &motors[1].encoder.pll_ki, // rw
};

int* exposed_ints[] = {
    (int*)&motors[0].control_mode, // rw
    &motors[0].encoder.encoder_offset, // rw
    &motors[0].encoder.encoder_state, // ro
    &motors[0].error, // rw
    (int*)&motors[1].control_mode, // rw
    &motors[1].encoder.encoder_offset, // rw
    &motors[1].encoder.encoder_state, // ro
    &motors[1].error, // rw
};

bool* exposed_bools[] = {
    &motors[0].thread_ready, // ro
    NULL, // rw
    NULL, // rw
    NULL, // ro
    &motors[1].thread_ready, // ro
    NULL, // rw
    NULL, // rw
    NULL, // ro
};

uint16_t* exposed_uint16[] = {
    &motors[0].control_deadline, // rw
    &motors[0].last_cpu_time, // ro
    &motors[1].control_deadline, // rw
    &motors[1].last_cpu_time, // ro
};

/* Monitoring */
size_t monitoring_slots[20] = {0};
constexpr size_t NUM_MONITORING_SLOTS = sizeof(monitoring_slots) / sizeof(monitoring_slots[0]);


/* Variables exposed to USB & UART via read/write commands */
// TODO: include range information in JSON description

// clang-format off
Endpoint endpoints[] = {
    Endpoint("vbus_voltage", static_cast<const float>(vbus_voltage)),
    Endpoint("elec_rad_per_enc", static_cast<const float>(elec_rad_per_enc)),
    Endpoint("motor0", BEGIN_TREE, nullptr, nullptr, nullptr),
        Endpoint("pos_setpoint", motors[0].pos_setpoint),
        Endpoint("pos_gain", motors[0].pos_gain),
        Endpoint("vel_setpoint", motors[0].vel_setpoint),
    Endpoint(nullptr, END_TREE, nullptr, nullptr, nullptr) // motor0
};
// clang-format on

constexpr size_t NUM_ENDPOINTS = sizeof(endpoints) / sizeof(endpoints[0]);


/* Private function prototypes -----------------------------------------------*/

int Protocol_print_simple_type(TypeInfo_t type_info, const void *ctx);
int Protocol_scan_simple_type(TypeInfo_t type_info, const char* buffer, void *ctx);
void Protocol_print_json(void);
void Protocol_print_monitoring(size_t limit);
void Protocol_parse_cmd(uint8_t* buffer, int len);

/* Function implementations --------------------------------------------------*/

void Endpoint::print_json(size_t id, bool& need_comma) {
    if (type_info_ == END_TREE) {
        printf("]}");
        need_comma = true;
        return;
    } else if (type_info_ < END_TREE) {
        if (need_comma)
            printf(",");
        printf("{\"name\":\"%s\",\"id\":%u,\"type\":\"%s\"",
                name_ ? name_ : "", id,
                type_names_[type_info_] ? type_names_[type_info_] : "");
        if (type_info_ == BEGIN_TREE) {
            printf(",\"content\":[");
            need_comma = false;
        } else {
            printf(",\"access\":\"");
            if (print_callback_)
                printf("r");
            if (scan_callback_)
                printf("w");
            printf("\"}");
            need_comma = true;
        }
    }
}

// Returns 0 on success, otherwise a non-zero error code
int Protocol_print_simple_type(TypeInfo_t type_info, const void *ctx) {
    switch (type_info) {
        case AS_FLOAT:
            printf("%f", *reinterpret_cast<const float*>(ctx)); break;
        case AS_INT: printf("%d", *reinterpret_cast<const int*>(ctx)); break;
        case AS_BOOL: printf("%d", *reinterpret_cast<const bool*>(ctx)); break;
        case AS_UINT16: printf("%hu", *reinterpret_cast<const uint16_t*>(ctx)); break;
        default:
            return -1;
    }
    return 0;
}

// Returns 0 on success, otherwise a non-zero error code
int Protocol_scan_simple_type(TypeInfo_t type_info, const char* buffer, void *ctx) {
    switch (type_info) {
        case AS_FLOAT: sscanf(buffer, "%f", reinterpret_cast<float*>(ctx)); break;
        case AS_INT: sscanf(buffer, "%d", reinterpret_cast<int*>(ctx)); break;
        case AS_BOOL: sscanf(buffer, "%d", reinterpret_cast<int*>(ctx)); break;
        case AS_UINT16: sscanf(buffer, "%hu", reinterpret_cast<uint16_t*>(ctx)); break;
        default:
            return -1;
    }
    return 0;
}

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
        int numscan = sscanf((const char*)buffer, "w %u %n", &index, &pos);
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
    } else if (buffer[0] == 'g') { // GET		
        // g <0:float,1:int,2:bool,3:uint16> index		
        int type = 0;		
        int index = 0;		
        int numscan = sscanf((const char*)buffer, "g %u %u", &type, &index);		
        if (numscan == 2) {		
            switch(type){		
            case 0: {		
                printf("%f\n",*exposed_floats[index]);		
                break;		
            };		
            case 1: {		
                printf("%d\n",*exposed_ints[index]);		
                break;		
            };		
            case 2: {		
                printf("%d\n",*exposed_bools[index]);		
                break;		
            };		
            case 3: {		
                printf("%hu\n",*exposed_uint16[index]);		
                break;		
            };		
            }		
        }		
    } else if (buffer[0] == 's') { // SET		
        // s <0:float,1:int,2:bool,3:uint16> index value		
        int type = 0;		
        int index = 0;		
        int numscan = sscanf((const char*)buffer, "s %u %u", &type, &index);		
        if (numscan == 2) {		
            switch(type) {		
            case 0: {		
                sscanf((const char*)buffer, "s %u %u %f", &type, &index, exposed_floats[index]);		
                break;		
            };		
            case 1: {		
                sscanf((const char*)buffer, "s %u %u %d", &type, &index, exposed_ints[index]);		
                break;		
            };		
            case 2: {		
                int btmp = 0;		
                sscanf((const char*)buffer, "s %u %u %d", &type, &index, &btmp);		
                *exposed_bools[index] = btmp ? true : false;		
                break;		
            };		
            case 3: {		
                sscanf((const char*)buffer, "s %u %u %hu", &type, &index, exposed_uint16[index]);		
                break;		
            };		
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
