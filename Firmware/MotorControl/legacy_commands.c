/* Includes ------------------------------------------------------------------*/
#include "legacy_commands.h"
#include <utils.h>

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
// This automatically updates to the interface that most
// recently recieved a command. In the future we may want to separate
// debug printf and the main serial comms.
SerialPrintf_t serial_printf_select = SERIAL_PRINTF_IS_UART;

/* Private constant data -----------------------------------------------------*/

// variables exposed to usb/serial interface via set/get/monitor
// Note: this will be depricated soon
float* exposed_floats[] = {
    &vbus_voltage, // ro
    NULL, //&elec_rad_per_enc, // ro
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
    (int*)&motors[0].encoder.encoder_offset, // rw
    (int*)&motors[0].encoder.encoder_state, // ro
    (int*)&motors[0].error, // rw
    (int*)&motors[1].control_mode, // rw
    (int*)&motors[1].encoder.encoder_offset, // rw
    (int*)&motors[1].encoder.encoder_state, // ro
    (int*)&motors[1].error, // rw
};

bool* exposed_bools[] = {
    &motors[0].thread_ready, // ro
    //For now these are written by Axis::SetupLegacyMappings
    NULL, // &motors[0].enable_control, // rw
    NULL, // &motors[0].do_calibration, // rw
    NULL, // &motors[0].calibration_ok, // ro
    &motors[1].thread_ready, // ro
    NULL, // &motors[1].enable_control, // rw
    NULL, // &motors[1].do_calibration, // rw
    NULL, // &motors[1].calibration_ok, // ro
};

uint16_t* exposed_uint16[] = {
    &motors[0].control_deadline, // rw
    &motors[0].last_cpu_time, // ro
    &motors[1].control_deadline, // rw
    &motors[1].last_cpu_time, // ro
};

/* Private variables ---------------------------------------------------------*/
monitoring_slot monitoring_slots[20] = {0};
/* Private function prototypes -----------------------------------------------*/
static void print_monitoring(int limit);

/* Function implementations --------------------------------------------------*/

void legacy_parse_cmd(const uint8_t* buffer, size_t len, size_t buffer_capacity, SerialPrintf_t response_interface) {
    // Set response interface
    serial_printf_select = response_interface;

    // Cast away const and write beyond the array bounds. Because we can.
    // (TODO: yeah maybe not, but this should be gone once we disable legacy commands)
    ((uint8_t *)buffer)[len < buffer_capacity ? len : (buffer_capacity - 1)] = 0;

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
    } else if(buffer[0] == 'i'){ // Dump device info
        // Retrieves the device signature, revision, flash size, and UUID
        printf("Signature: %#x\n", STM_ID_GetSignature());
        printf("Revision: %#x\n", STM_ID_GetRevision());
        printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));
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
    } else if (buffer[0] == 'h'){  // HALT
        for(int i = 0; i < num_motors; i++){
            set_vel_setpoint(&motors[i], 0.0f, 0.0f);
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
        // m <0:float,1:int,2:bool,3:uint16> index monitoring_slot
        int type = 0;
        int index = 0;
        int slot = 0;
        int numscan = sscanf((const char*)buffer, "m %u %u %u", &type, &index, &slot);
        if (numscan == 3) {
            monitoring_slots[slot].type = type;
            monitoring_slots[slot].index = index;
        }
    } else if (buffer[0] == 'o') { // Output Monitor
        int limit = 0;
        int numscan = sscanf((const char*)buffer, "o %u", &limit);
        if (numscan == 1) {
            print_monitoring(limit);
        }
    } else if (buffer[0] == 't') { // Run Anti-Cogging Calibration
        for (int i = 0; i < num_motors; i++) {
            // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }
    }
}

void legacy_parse_stream(const uint8_t* buffer, size_t len) {
    #define PARSE_BUFFER_SIZE 64
    static uint8_t parse_buffer[PARSE_BUFFER_SIZE];
    static bool read_active = false;
    static uint32_t parse_buffer_idx = 0;

    while (len--) {
        // Fetch the next char
        uint8_t c = *(buffer++);
        // Look for start character
        if (c == '$') {
            read_active = true;
            continue; // do not record start char
        }
        // Record into parse buffer when actively reading
        if (read_active) {
            parse_buffer[parse_buffer_idx++] = c;
            if (c == '\r' || c == '\n' || c == '!') {
                // End of command string
                legacy_parse_cmd(parse_buffer, parse_buffer_idx, PARSE_BUFFER_SIZE, SERIAL_PRINTF_IS_UART);
                // Reset receieve state machine
                read_active = false;
                parse_buffer_idx = 0;
            } else if (parse_buffer_idx == PARSE_BUFFER_SIZE - 1) {
                // We are not at end of command, and receiving another character after this
                // would go into the last slot, which is reserved for terminating null.
                // We have effectively overflowed parse buffer: abort.
                read_active = false;
                parse_buffer_idx = 0;
            }
        }
    }
}

static void print_monitoring(int limit) {
    for (int i=0;i<limit;i++) {
        switch (monitoring_slots[i].type) {
        case 0:
            printf("%f\t",*exposed_floats[monitoring_slots[i].index]);
            break;
        case 1:
            printf("%d\t",*exposed_ints[monitoring_slots[i].index]);
            break;
        case 2:
            printf("%d\t",*exposed_bools[monitoring_slots[i].index]);
            break;
        case 3:
            printf("%hu\t",*exposed_uint16[monitoring_slots[i].index]);
            break;
        default:
            i=100;
        }
    }
    printf("\n");
}
