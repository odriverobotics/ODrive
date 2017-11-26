/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <commands.h>
#include <usart.h>
#include <gpio.h>
#include <freertos_vars.h>
#include <usbd_cdc.h>
#include <utils.h>

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
// This automatically updates to the interface that most
// recently recieved a command. In the future we may want to separate
// debug printf and the main serial comms.
SerialPrintf_t serial_printf_select = SERIAL_PRINTF_IS_NONE;

/* Private constant data -----------------------------------------------------*/
// TODO: make command to switch gpio_mode during run-time
static const GpioMode_t gpio_mode = GPIO_MODE_NONE;     //GPIO 1,2 is not configured
// static const GpioMode_t gpio_mode = GPIO_MODE_UART;     //GPIO 1,2 is UART Tx,Rx
// static const GpioMode_t gpio_mode = GPIO_MODE_STEP_DIR; //GPIO 1,2 is M0 Step,Dir

static uint8_t* usb_buf;
static uint32_t usb_len;
extern USBD_HandleTypeDef hUsbDeviceFS;

// variables exposed to usb/serial interface via set/get/monitor
// Note: this will be depricated soon
static float* const exposed_floats[] = {
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

static int* const exposed_ints[] = {
    (int*)&motors[0].control_mode, // rw
    &motors[0].encoder.encoder_offset, // rw
    &motors[0].encoder.encoder_state, // ro
    &motors[0].error, // rw
    (int*)&motors[1].control_mode, // rw
    &motors[1].encoder.encoder_offset, // rw
    &motors[1].encoder.encoder_state, // ro
    &motors[1].error, // rw
};

static bool* const exposed_bools[] = {
    &motors[0].thread_ready, // ro
    &motors[0].enable_control, // rw
    &motors[0].do_calibration, // rw
    &motors[0].calibration_ok, // ro
    &motors[1].thread_ready, // ro
    &motors[1].enable_control, // rw
    &motors[1].do_calibration, // rw
    &motors[1].calibration_ok, // ro
};

static uint16_t* const exposed_uint16[] = {
    &motors[0].control_deadline, // rw
    &motors[0].last_cpu_time, // ro
    &motors[1].control_deadline, // rw
    &motors[1].last_cpu_time, // ro
};

/* Private variables ---------------------------------------------------------*/
monitoring_slot monitoring_slots[20];
/* Private function prototypes -----------------------------------------------*/
static void print_monitoring(int limit);

/* Function implementations --------------------------------------------------*/
void init_communication() {
    switch (gpio_mode) {
        case GPIO_MODE_NONE:
        break; //do nothing
        case GPIO_MODE_UART: {
            SetGPIO12toUART();
        } break;
        case GPIO_MODE_STEP_DIR: {
            SetGPIO12toStepDir();
        } break;
        default:
        //TODO: report error unexpected mode
        break;
    }
}

void motor_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface) {
    // Set response interface
    serial_printf_select = response_interface;

    // TODO very hacky way of terminating sscanf at end of buffer:
    // We should do some proper struct packing instead of using sscanf altogether
    buffer[len-1] = 0;

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

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void cmd_parse_thread(void const * argument) {
    //DMA open loop continous circular buffer
    //1ms delay periodic, chase DMA ptr around, on new data:
        // Check for start char
        // copy into parse-buffer
        // check for end-char
        // checksum, etc.

    #define UART_RX_BUFFER_SIZE 64
    static uint8_t dma_circ_buffer[UART_RX_BUFFER_SIZE];
    static uint8_t parse_buffer[UART_RX_BUFFER_SIZE];

    // DMA is set up to recieve in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));

    uint32_t last_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
    // Re-run state-machine forever
    for (;;) {
        //Inialize recieve state machine
        bool reset_read_state = false;
        bool read_active = false;
        uint32_t parse_buffer_idx = 0;
        //Run state machine until reset
        do {
            // Check for UART errors and restart recieve DMA transfer if required
            if (huart4.ErrorCode != HAL_UART_ERROR_NONE) {
                HAL_UART_AbortReceive(&huart4);
                HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));
                break; //reset state machine
            }
            // Fetch the circular buffer "write pointer", where it would write next
            uint32_t rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
            // During sleeping, we may have fallen several characters behind, so we keep
            // going until we are caught up, before we sleep again
            while (rcv_idx != last_rcv_idx) {
                // Fetch the next char, rotate read ptr
                uint8_t c = dma_circ_buffer[last_rcv_idx];
                if (++last_rcv_idx == UART_RX_BUFFER_SIZE)
                    last_rcv_idx = 0;
                // Look for start character
                if (c == '$') {
                    read_active = true;
                    continue; // do not record start char
                }
                // Record into parse buffer when actively reading
                if (read_active) {
                    parse_buffer[parse_buffer_idx++] = c;
                    if (c == '\r' || c == '\n' || c == '!') {
                        // End of command string: exchange end char with terminating null
                        parse_buffer[parse_buffer_idx-1] = '\0';
                        motor_parse_cmd(parse_buffer, parse_buffer_idx, SERIAL_PRINTF_IS_UART);
                        // Reset receieve state machine
                        reset_read_state = true;
                        break;
                    } else if (parse_buffer_idx == UART_RX_BUFFER_SIZE - 1) {
                        // We are not at end of command, and receiving another character after this
                        // would go into the last slot, which is reserved for terminating null.
                        // We have effectively overflowed parse buffer: abort.
                        reset_read_state = true;
                        break;
                    }
                }
            }
            // When we reach here, we are out of immediate characters to fetch out of UART buffer
            // Now we check if there is any USB processing to do: we wait for up to 1 ms,
            // before going back to checking UART again.
            osStatus sem_stat = osSemaphoreWait(sem_usb_rx, 1); // 1ms
            if (sem_stat == osOK) {
                motor_parse_cmd(usb_buf, usb_len, SERIAL_PRINTF_IS_USB);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);  // Allow next packet
            }
        } while (!reset_read_state);
    }
    // If we get here, then this task is done
    vTaskDelete(osThreadGetId());
}

// Called from CDC_Receive_FS callback function, this allows motor_parse_cmd to access the
// incoming USB data
void set_cmd_buffer(uint8_t *buf, uint32_t len) {
    usb_buf = buf;
    usb_len = len;
}

void usb_update_thread() {
    for (;;) {
        // Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
        osStatus semaphore_status = osSemaphoreWait(sem_usb_irq, osWaitForever);
        if (semaphore_status == osOK) {
            // We have a new incoming USB transmission: handle it
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
            // Let the irq (OTG_FS_IRQHandler) fire again.
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
    vTaskDelete(osThreadGetId());
}

void can_update_thread() {
    for (;;) {
        // Wait for signalling from CAN interrupt (CAN1_RX0_IRQHandler)
        osStatus semaphore_status = osSemaphoreWait(sem_can_irq, osWaitForever);
        if (semaphore_status == osOK) {
            // We have a new incoming CAN transmission: handle it
            HAL_CAN_IRQHandler(&hcan1);
            // Let the irq (CAN1_RX0_IRQn) fire again.
            HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        }
    }
    vTaskDelete(osThreadGetId());
}
