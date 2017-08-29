#include <commands.h>
#include <controller.h>
#include <motor.h>

/* variables exposed to usb interface via set/get/monitor
 * If you change something here, don't forget to regenerate the python interface
 * with generate_api.py
 * ro/rw : read only/read write -> ro prevents the code generator from
 * generating setter
 * */

static void print_monitoring(int limit);

/* Monitoring */
monitoring_slot monitoring_slots[20] = {0};

float *exposed_floats[] = {
    &vbus_voltage,                                            // ro
    &elec_rad_per_enc,                                        // ro
    &motors[0].pos_setpoint,                                  // rw
    &motors[0].pos_gain,                                      // rw
    &motors[0].vel_setpoint,                                  // rw
    &motors[0].vel_gain,                                      // rw
    &motors[0].vel_integrator_gain,                           // rw
    &motors[0].vel_integrator_current,                        // rw
    &motors[0].vel_limit,                                     // rw
    &motors[0].current_setpoint,                              // rw
    &motors[0].calibration_current,                           // rw
    &motors[0].phase_inductance,                              // ro
    &motors[0].phase_resistance,                              // ro
    &motors[0].current_meas.phB,                              // ro
    &motors[0].current_meas.phC,                              // ro
    &motors[0].DC_calib.phB,                                  // rw
    &motors[0].DC_calib.phC,                                  // rw
    &motors[0].shunt_conductance,                             // rw
    &motors[0].phase_current_rev_gain,                        // rw
    &motors[0].current_control.current_lim,                   // rw
    &motors[0].current_control.p_gain,                        // rw
    &motors[0].current_control.i_gain,                        // rw
    &motors[0].current_control.v_current_control_integral_d,  // rw
    &motors[0].current_control.v_current_control_integral_q,  // rw
    &motors[0].current_control.Ibus,                          // ro
    &motors[0].encoder.phase,                                 // ro
    &motors[0].encoder.pll_pos,                               // rw
    &motors[0].encoder.pll_vel,                               // rw
    &motors[0].encoder.pll_kp,                                // rw
    &motors[0].encoder.pll_ki,                                // rw
    &motors[1].pos_setpoint,                                  // rw
    &motors[1].pos_gain,                                      // rw
    &motors[1].vel_setpoint,                                  // rw
    &motors[1].vel_gain,                                      // rw
    &motors[1].vel_integrator_gain,                           // rw
    &motors[1].vel_integrator_current,                        // rw
    &motors[1].vel_limit,                                     // rw
    &motors[1].current_setpoint,                              // rw
    &motors[1].calibration_current,                           // rw
    &motors[1].phase_inductance,                              // ro
    &motors[1].phase_resistance,                              // ro
    &motors[1].current_meas.phB,                              // ro
    &motors[1].current_meas.phC,                              // ro
    &motors[1].DC_calib.phB,                                  // rw
    &motors[1].DC_calib.phC,                                  // rw
    &motors[1].shunt_conductance,                             // rw
    &motors[1].phase_current_rev_gain,                        // rw
    &motors[1].current_control.current_lim,                   // rw
    &motors[1].current_control.p_gain,                        // rw
    &motors[1].current_control.i_gain,                        // rw
    &motors[1].current_control.v_current_control_integral_d,  // rw
    &motors[1].current_control.v_current_control_integral_q,  // rw
    &motors[1].current_control.Ibus,                          // ro
    &motors[1].encoder.phase,                                 // ro
    &motors[1].encoder.pll_pos,                               // rw
    &motors[1].encoder.pll_vel,                               // rw
    &motors[1].encoder.pll_kp,                                // rw
    &motors[1].encoder.pll_ki,                                // rw
};

int *exposed_ints[] = {
    (int *)&motors[0].control_mode,     // rw
    &motors[0].encoder.encoder_offset,  // rw
    &motors[0].encoder.encoder_state,   // ro
    &motors[0].error,                   // rw
    (int *)&motors[1].control_mode,     // rw
    &motors[1].encoder.encoder_offset,  // rw
    &motors[1].encoder.encoder_state,   // ro
    &motors[1].error,                   // rw
};

bool *exposed_bools[] = {
    &motors[0].thread_ready,    // ro
    &motors[0].enable_control,  // rw
    &motors[0].do_calibration,  // rw
    &motors[0].calibration_ok,  // ro
    &motors[1].thread_ready,    // ro
    &motors[1].enable_control,  // rw
    &motors[1].do_calibration,  // rw
    &motors[1].calibration_ok,  // ro
};

uint16_t *exposed_uint16[] = {
    &motors[0].control_deadline,  // rw
    &motors[0].last_cpu_time,     // ro
    &motors[1].control_deadline,  // rw
    &motors[1].last_cpu_time,     // ro
};

static void print_monitoring(int limit) {
    for (int i = 0; i < limit; i++) {
        switch (monitoring_slots[i].type) {
            case 0:
                printf("%f\t", *exposed_floats[monitoring_slots[i].index]);
                break;
            case 1:
                printf("%d\t", *exposed_ints[monitoring_slots[i].index]);
                break;
            case 2:
                printf("%d\t", *exposed_bools[monitoring_slots[i].index]);
                break;
            case 3:
                printf("%hu\t", *exposed_uint16[monitoring_slots[i].index]);
                break;
            default:
                i = 100;
        }
    }
    printf("\n");
}

void motor_parse_cmd(uint8_t *buffer, int len) {
    // TODO very hacky way of terminating sscanf at end of buffer:
    // We should do some proper struct packing instead of using sscanf
    // altogether
    buffer[len] = 0;

    // check incoming packet type
    if (buffer[0] == 'p') {
        // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;
        int numscan =
            sscanf((const char *)buffer, "p %u %f %f %f", &motor_number,
                   &pos_setpoint, &vel_feed_forward, &current_feed_forward);
        if (numscan == 4 && motor_number < num_motors) {
            set_pos_setpoint(&motors[motor_number], pos_setpoint,
                             vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'v') {
        // velocity control
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char *)buffer, "v %u %f %f", &motor_number,
                             &vel_feed_forward, &current_feed_forward);
        if (numscan == 3 && motor_number < num_motors) {
            set_vel_setpoint(&motors[motor_number], vel_feed_forward,
                             current_feed_forward);
        }
    } else if (buffer[0] == 'c') {
        // current control
        unsigned motor_number;
        float current_feed_forward;
        int numscan = sscanf((const char *)buffer, "c %u %f", &motor_number,
                             &current_feed_forward);
        if (numscan == 2 && motor_number < num_motors) {
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        }
    } else if (buffer[0] == 'g') {  // GET
        // g <0:float,1:int,2:bool,3:uint16> index
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char *)buffer, "g %u %u", &type, &index);
        if (numscan == 2) {
            switch (type) {
                case 0: {
                    printf("%f\n", *exposed_floats[index]);
                    break;
                };
                case 1: {
                    printf("%d\n", *exposed_ints[index]);
                    break;
                };
                case 2: {
                    printf("%d\n", *exposed_bools[index]);
                    break;
                };
                case 3: {
                    printf("%hu\n", *exposed_uint16[index]);
                    break;
                };
            }
        }
    } else if (buffer[0] == 's') {  // SET
        // s <0:float,1:int,2:bool,3:uint16> index value
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char *)buffer, "s %u %u", &type, &index);
        if (numscan == 2) {
            switch (type) {
                case 0: {
                    sscanf((const char *)buffer, "s %u %u %f", &type, &index,
                           exposed_floats[index]);
                    break;
                };
                case 1: {
                    sscanf((const char *)buffer, "s %u %u %d", &type, &index,
                           exposed_ints[index]);
                    break;
                };
                case 2: {
                    int btmp = 0;
                    sscanf((const char *)buffer, "s %u %u %d", &type, &index,
                           &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 3: {
                    sscanf((const char *)buffer, "s %u %u %hu", &type, &index,
                           exposed_uint16[index]);
                    break;
                };
            }
        }
    } else if (buffer[0] == 'm') {  // Setup Monitor
        // m <0:float,1:int,2:bool,3:uint16> index monitoring_slot
        int type = 0;
        int index = 0;
        int slot = 0;
        int numscan =
            sscanf((const char *)buffer, "m %u %u %u", &type, &index, &slot);
        if (numscan == 3) {
            monitoring_slots[slot].type = type;
            monitoring_slots[slot].index = index;
        }
    } else if (buffer[0] == 'o') {  // Output Monitor
        int limit = 0;
        int numscan = sscanf((const char *)buffer, "o %u", &limit);
        if (numscan == 1) {
            print_monitoring(limit);
        }
    } else if (buffer[0] == 'e') {
        // Check for Errors
        ODrive_Error_t err;
        for (int i = 0; i < num_motors; i++) {
            err = motors[i].error;

            printf("Motor %d: ", i);
            switch (err) {
                case ERROR_NO_ERROR:
                    printf("No Error");
                    break;
                case ERROR_PHASE_RESISTANCE_TIMING:
                    printf("Phase Resistance Timing");
                    break;
                case ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT:
                    printf("Phase Resistance Measurement Timeout");
                    break;
                case ERROR_PHASE_RESISTANCE_OUT_OF_RANGE:
                    printf("Phase Resistance Out of Range");
                    break;
                case ERROR_PHASE_INDUCTANCE_TIMING:
                    printf("Phase Inductance Timing");
                    break;
                case ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT:
                    printf("Phase Inductance Measurement Timeout");
                    break;
                case ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE:
                    printf("Inductance Out of Range");
                    break;
                case ERROR_ENCODER_RESPONSE:
                    printf("Encoder Response");
                    break;
                case ERROR_ENCODER_MEASUREMENT_TIMEOUT:
                    printf("Encoder Measurement Timeout");
                    break;
                case ERROR_ADC_FAILED:
                    printf("ADC Failed");
                    break;
                case ERROR_CALIBRATION_TIMING:
                    printf("Calibration Timing");
                    break;
                case ERROR_FOC_TIMING:
                    printf("FOC Timing");
                    break;
                case ERROR_FOC_MEASUREMENT_TIMEOUT:
                    printf("FOC Measurement Timeout");
                    break;
                case ERROR_SCAN_MOTOR_TIMING:
                    printf("Scan Motor Timing");
                    break;
                case ERROR_FOC_VOLTAGE_TIMING:
                    printf("FOC Voltage Timing");
                    break;
                case ERROR_GATEDRIVER_INVALID_GAIN:
                    printf("Gate Driver Invalid Gain");
                    break;
                case ERROR_PWM_SRC_FAIL:
                    printf("PWM SRC Fail");
                    break;
                case ERROR_UNEXPECTED_STEP_SRC:
                    printf("Unexpected Step SRC");
                    break;
                case ERROR_POS_CTRL_DURING_SENSORLESS:
                    printf("POS Control During Sensorless");
                    break;
                case ERROR_SPIN_UP_TIMEOUT:
                    printf("Spinup Timeout");
                    break;
            }
            printf("\n");
        }
    }
}
