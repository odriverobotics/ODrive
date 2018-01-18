
/* Includes ------------------------------------------------------------------*/

// TODO: remove this option
// and once the legacy protocol is phased out, remove the seq-no hack in protocol.py
// todo: make clean switches for protocol
#define ENABLE_LEGACY_PROTOCOL

#include "commands.h"
#include "low_level.h"
#include "protocol.hpp"
#include "freertos_vars.h"
#include "utils.h"

#ifdef ENABLE_LEGACY_PROTOCOL
#include "legacy_commands.h"
#endif

#include <cmsis_os.h>
#include <memory>
#include <usbd_cdc_if.h>
#include <usb_device.h>
#include <usart.h>
#include <gpio.h>

#define UART_TX_BUFFER_SIZE 64

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private constant data -----------------------------------------------------*/
// TODO: make command to switch gpio_mode during run-time
#if defined(USE_GPIO_MODE_STEP_DIR)
static const GpioMode_t gpio_mode = GPIO_MODE_STEP_DIR; //GPIO 1,2 is M0 Step,Dir
#elif !defined(UART_PROTOCOL_NONE)
static const GpioMode_t gpio_mode = GPIO_MODE_UART;     //GPIO 1,2 is UART Tx,Rx
#else
static const GpioMode_t gpio_mode = GPIO_MODE_NONE;     //GPIO 1,2 is not configured
#endif

/* Private variables ---------------------------------------------------------*/

static uint8_t* usb_buf;
static uint32_t usb_len;

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
static thread_local uint32_t deadline_ms = 0;

/* Variables exposed to USB & UART via read/write commands */
// TODO: include range information in JSON description


// TODO: Autogenerate these functions
void motors_0_set_pos_setpoint_func(void) {
    set_pos_setpoint(&motors[0],
        motors[0].set_pos_setpoint_args.pos_setpoint,
        motors[0].set_pos_setpoint_args.vel_feed_forward,
        motors[0].set_pos_setpoint_args.current_feed_forward);
}
void motors_0_set_vel_setpoint_func(void) {
    set_vel_setpoint(&motors[0],
        motors[0].set_vel_setpoint_args.vel_setpoint,
        motors[0].set_vel_setpoint_args.current_feed_forward);
}
void motors_0_set_current_setpoint_func(void) {
    set_current_setpoint(&motors[0],
        motors[0].set_current_setpoint_args.current_setpoint);
}
void motors_1_set_pos_setpoint_func(void) {
    set_pos_setpoint(&motors[1],
        motors[1].set_pos_setpoint_args.pos_setpoint,
        motors[1].set_pos_setpoint_args.vel_feed_forward,
        motors[1].set_pos_setpoint_args.current_feed_forward);
}
void motors_1_set_vel_setpoint_func(void) {
    set_vel_setpoint(&motors[1],
        motors[1].set_vel_setpoint_args.vel_setpoint,
        motors[1].set_vel_setpoint_args.current_feed_forward);
}
void motors_1_set_current_setpoint_func(void) {
    set_current_setpoint(&motors[1],
        motors[1].set_current_setpoint_args.current_setpoint);
}

// This table specifies which fields and functions are exposed on the USB and UART ports.
// TODO: Autogenerate this table. It will come up again very soon in the Arduino library.
// clang-format off
const Endpoint endpoints[] = {
    Endpoint::make_property("vbus_voltage", const_cast<const float*>(&vbus_voltage)),
    Endpoint::make_property("elec_rad_per_enc", const_cast<const float*>(&elec_rad_per_enc)),
	Endpoint::make_property("UUID_0", (const uint32_t*)(ID_UNIQUE_ADDRESS + 0*4)),
	Endpoint::make_property("UUID_1", (const uint32_t*)(ID_UNIQUE_ADDRESS + 1*4)),
	Endpoint::make_property("UUID_2", (const uint32_t*)(ID_UNIQUE_ADDRESS + 2*4)),
    Endpoint::make_object("motor0"),
        Endpoint::make_property("control_mode", reinterpret_cast<int32_t*>(&motors[0].control_mode)),
        Endpoint::make_property("error", reinterpret_cast<int32_t*>(&motors[0].error)),
        Endpoint::make_property("pos_setpoint", &motors[0].pos_setpoint),
        Endpoint::make_property("pos_gain", &motors[0].pos_gain),
        Endpoint::make_property("vel_setpoint", &motors[0].vel_setpoint),
        Endpoint::make_property("vel_gain", &motors[0].vel_gain),
        Endpoint::make_property("vel_integrator_gain", &motors[0].vel_integrator_gain),
        Endpoint::make_property("vel_integrator_current", &motors[0].vel_integrator_current),
        Endpoint::make_property("vel_limit", &motors[0].vel_limit),
        Endpoint::make_property("current_setpoint", &motors[0].current_setpoint),
        Endpoint::make_property("calibration_current", &motors[0].calibration_current),
        Endpoint::make_property("phase_inductance", const_cast<const float*>(&motors[0].phase_inductance)),
        Endpoint::make_property("phase_resistance", const_cast<const float*>(&motors[0].phase_resistance)),
        Endpoint::make_property("current_meas_phB", const_cast<const float*>(&motors[0].current_meas.phB)),
        Endpoint::make_property("current_meas_phC", const_cast<const float*>(&motors[0].current_meas.phC)),
        Endpoint::make_property("DC_calib.phB", &motors[0].DC_calib.phB),
        Endpoint::make_property("DC_calib.phC", &motors[0].DC_calib.phC),
        Endpoint::make_property("shunt_conductance", &motors[0].shunt_conductance),
        Endpoint::make_property("phase_current_rev_gain", &motors[0].phase_current_rev_gain),
        Endpoint::make_property("thread_ready", reinterpret_cast<uint8_t*>(&motors[0].thread_ready)),
        Endpoint::make_property("control_deadline", &motors[0].control_deadline),
        Endpoint::make_property("last_cpu_time", &motors[0].last_cpu_time),
        Endpoint::make_object("current_control"),
            Endpoint::make_property("current_lim", &motors[0].current_control.current_lim),
            Endpoint::make_property("p_gain", &motors[0].current_control.p_gain),
            Endpoint::make_property("i_gain", &motors[0].current_control.i_gain),
            Endpoint::make_property("v_current_control_integral_d", &motors[0].current_control.v_current_control_integral_d),
            Endpoint::make_property("v_current_control_integral_q", &motors[0].current_control.v_current_control_integral_q),
            Endpoint::make_property("Iq_command", &motors[0].current_control.Iq),
            Endpoint::make_property("Ibus", const_cast<const float*>(&motors[0].current_control.Ibus)),
        Endpoint::close_tree(),
        Endpoint::make_object("encoder"),
            Endpoint::make_property("phase", const_cast<const float*>(&motors[0].encoder.phase)),
            Endpoint::make_property("pll_pos", &motors[0].encoder.pll_pos),
            Endpoint::make_property("pll_vel", &motors[0].encoder.pll_vel),
            Endpoint::make_property("pll_kp", &motors[0].encoder.pll_kp),
            Endpoint::make_property("pll_ki", &motors[0].encoder.pll_ki),
            Endpoint::make_property("encoder_offset", &motors[0].encoder.encoder_offset),
            Endpoint::make_property("encoder_state", &motors[0].encoder.encoder_state),
        Endpoint::close_tree(),
        Endpoint::make_function("set_pos_setpoint", &motors_0_set_pos_setpoint_func),
            Endpoint::make_property("pos_setpoint", &motors[0].set_pos_setpoint_args.pos_setpoint),
            Endpoint::make_property("vel_feed_forward", &motors[0].set_pos_setpoint_args.vel_feed_forward),
            Endpoint::make_property("current_feed_forward", &motors[0].set_pos_setpoint_args.current_feed_forward),
        Endpoint::close_tree(),
        Endpoint::make_function("set_vel_setpoint", &motors_0_set_vel_setpoint_func),
            Endpoint::make_property("vel_setpoint", &motors[0].set_vel_setpoint_args.vel_setpoint),
            Endpoint::make_property("current_feed_forward", &motors[0].set_vel_setpoint_args.current_feed_forward),
        Endpoint::close_tree(),
        Endpoint::make_function("set_current_setpoint", &motors_0_set_current_setpoint_func),
            Endpoint::make_property("current_setpoint", &motors[0].set_current_setpoint_args.current_setpoint),
        Endpoint::close_tree(),
    Endpoint::close_tree(), // motor0
    Endpoint::make_object("motor1"),
        Endpoint::make_property("control_mode", reinterpret_cast<int32_t*>(&motors[1].control_mode)),
        Endpoint::make_property("error", reinterpret_cast<int32_t*>(&motors[1].error)),
        Endpoint::make_property("pos_setpoint", &motors[1].pos_setpoint),
        Endpoint::make_property("pos_gain", &motors[1].pos_gain),
        Endpoint::make_property("vel_setpoint", &motors[1].vel_setpoint),
        Endpoint::make_property("vel_gain", &motors[1].vel_gain),
        Endpoint::make_property("vel_integrator_gain", &motors[1].vel_integrator_gain),
        Endpoint::make_property("vel_integrator_current", &motors[1].vel_integrator_current),
        Endpoint::make_property("vel_limit", &motors[1].vel_limit),
        Endpoint::make_property("current_setpoint", &motors[1].current_setpoint),
        Endpoint::make_property("calibration_current", &motors[1].calibration_current),
        Endpoint::make_property("phase_inductance", const_cast<const float*>(&motors[1].phase_inductance)),
        Endpoint::make_property("phase_resistance", const_cast<const float*>(&motors[1].phase_resistance)),
        Endpoint::make_property("current_meas_phB", const_cast<const float*>(&motors[1].current_meas.phB)),
        Endpoint::make_property("current_meas_phC", const_cast<const float*>(&motors[1].current_meas.phC)),
        Endpoint::make_property("DC_calib.phB", &motors[1].DC_calib.phB),
        Endpoint::make_property("DC_calib.phC", &motors[1].DC_calib.phC),
        Endpoint::make_property("shunt_conductance", &motors[1].shunt_conductance),
        Endpoint::make_property("phase_current_rev_gain", &motors[1].phase_current_rev_gain),
        Endpoint::make_property("thread_ready", reinterpret_cast<uint8_t*>(&motors[1].thread_ready)),
        Endpoint::make_property("control_deadline", &motors[1].control_deadline),
        Endpoint::make_property("last_cpu_time", &motors[1].last_cpu_time),
        Endpoint::make_object("current_control"),
            Endpoint::make_property("current_lim", &motors[1].current_control.current_lim),
            Endpoint::make_property("p_gain", &motors[1].current_control.p_gain),
            Endpoint::make_property("i_gain", &motors[1].current_control.i_gain),
            Endpoint::make_property("v_current_control_integral_d", &motors[1].current_control.v_current_control_integral_d),
            Endpoint::make_property("v_current_control_integral_q", &motors[1].current_control.v_current_control_integral_q),
            Endpoint::make_property("Iq_command", &motors[1].current_control.Iq),
            Endpoint::make_property("Ibus", const_cast<const float*>(&motors[1].current_control.Ibus)),
        Endpoint::close_tree(),
        Endpoint::make_object("encoder"),
            Endpoint::make_property("phase", const_cast<const float*>(&motors[1].encoder.phase)),
            Endpoint::make_property("pll_pos", &motors[1].encoder.pll_pos),
            Endpoint::make_property("pll_vel", &motors[1].encoder.pll_vel),
            Endpoint::make_property("pll_kp", &motors[1].encoder.pll_kp),
            Endpoint::make_property("pll_ki", &motors[1].encoder.pll_ki),
            Endpoint::make_property("encoder_offset", reinterpret_cast<int32_t*>(&motors[1].encoder.encoder_offset)),
            Endpoint::make_property("encoder_state", reinterpret_cast<int32_t*>(&motors[1].encoder.encoder_state)),
        Endpoint::close_tree(),
        Endpoint::make_function("set_pos_setpoint", &motors_1_set_pos_setpoint_func),
            Endpoint::make_property("pos_setpoint", &motors[1].set_pos_setpoint_args.pos_setpoint),
            Endpoint::make_property("vel_feed_forward", &motors[1].set_pos_setpoint_args.vel_feed_forward),
            Endpoint::make_property("current_feed_forward", &motors[1].set_pos_setpoint_args.current_feed_forward),
        Endpoint::close_tree(),
        Endpoint::make_function("set_vel_setpoint", &motors_1_set_vel_setpoint_func),
            Endpoint::make_property("vel_setpoint", &motors[1].set_vel_setpoint_args.vel_setpoint),
            Endpoint::make_property("current_feed_forward", &motors[1].set_vel_setpoint_args.current_feed_forward),
        Endpoint::close_tree(),
        Endpoint::make_function("set_current_setpoint", &motors_1_set_current_setpoint_func),
            Endpoint::make_property("current_setpoint", &motors[1].set_current_setpoint_args.current_setpoint),
        Endpoint::close_tree(),
    Endpoint::close_tree() // motor1
};
// clang-format on

constexpr size_t NUM_ENDPOINTS = sizeof(endpoints) / sizeof(endpoints[0]);


#if defined(USB_PROTOCOL_NATIVE)

class USBSender : public PacketSink {
public:
    int process_packet(const uint8_t* buffer, size_t length) {
        // cannot send partial packets
        if (length > USB_TX_DATA_SIZE)
            return -1;
        // wait for USB interface to become ready
        if (osSemaphoreWait(sem_usb_tx, deadline_to_timeout(deadline_ms)) != osOK)
            return -1;
        // transmit packet
        uint8_t status = CDC_Transmit_FS(
                const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                well... it's not actually. Stupid STM. */, length);
        return (status == USBD_OK) ? 0 : -1;
    }
} usb_sender;

BidirectionalPacketBasedChannel usb_channel(endpoints, NUM_ENDPOINTS, usb_sender);

#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)

class USBSender : public StreamSink {
public:
    int process_bytes(const uint8_t* buffer, size_t length) {
        // Loop to ensure all bytes get sent
        while (length) {
            size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
            // wait for USB interface to become ready
            if (osSemaphoreWait(sem_usb_tx, deadline_to_timeout(deadline_ms)) != osOK)
                return -1;
            // transmit chunk
            if (CDC_Transmit_FS(
                    const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                    well... it's not actually. Stupid STM. */, chunk) != USBD_OK)
                return -1;
            buffer += chunk;
            length -= chunk;
        }
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }
} usb_sender;

PacketToStreamConverter usb_packet_sender(usb_sender);
BidirectionalPacketBasedChannel usb_channel(endpoints, NUM_ENDPOINTS, usb_packet_sender);
StreamToPacketConverter usb_stream_sink(usb_channel);

#endif

#if defined(UART_PROTOCOL_NATIVE)
class UART4Sender : public StreamSink {
public:
    int process_bytes(const uint8_t* buffer, size_t length) {
        // Loop to ensure all bytes get sent
        while (length) {
            size_t chunk = length < UART_TX_BUFFER_SIZE ? length : UART_TX_BUFFER_SIZE;
            // wait for USB interface to become ready
            // TODO: implement ring buffer to get a more continuous stream of data
            if (osSemaphoreWait(sem_uart_dma, deadline_to_timeout(deadline_ms)) != osOK)
                return -1;
            // transmit chunk
            memcpy(tx_buf_, buffer, chunk);
            if (HAL_UART_Transmit_DMA(&huart4, tx_buf_, chunk) != HAL_OK)
                return -1;
            buffer += chunk;
            length -= chunk;
        }
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }
private:
    uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
} uart4_sender;

PacketToStreamConverter uart4_packet_sender(uart4_sender);
BidirectionalPacketBasedChannel uart4_channel(endpoints, NUM_ENDPOINTS, uart4_packet_sender);
StreamToPacketConverter UART4_stream_sink(uart4_channel);
#endif

/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

void init_communication(void) {
    switch (gpio_mode) {
        case GPIO_MODE_NONE:
        break; //do nothing
        case GPIO_MODE_UART: {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
            SetGPIO12toUART();
#endif
        } break;
        case GPIO_MODE_STEP_DIR: {
            SetGPIO12toStepDir();
        } break;
        default:
        //TODO: report error unexpected mode
        break;
    }
}

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void communication_task(void const * argument) {
    (void) argument;
    

#if !defined(UART_PROTOCOL_NONE)
    //DMA open loop continous circular buffer
    //1ms delay periodic, chase DMA ptr around

    #define UART_RX_BUFFER_SIZE 64
    static uint8_t dma_circ_buffer[UART_RX_BUFFER_SIZE];

    // DMA is set up to recieve in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));
    uint32_t last_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
#endif

    // Re-run state-machine forever
    for (;;) {
#if !defined(UART_PROTOCOL_NONE)
        // Check for UART errors and restart recieve DMA transfer if required
        if (huart4.ErrorCode != HAL_UART_ERROR_NONE) {
            HAL_UART_AbortReceive(&huart4);
            HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));
        }
        // Fetch the circular buffer "write pointer", where it would write next
        uint32_t new_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;

        deadline_ms = timeout_to_deadline(PROTOCOL_SERVER_TIMEOUT_MS);
#if defined(UART_PROTOCOL_NATIVE)
        // Process bytes in one or two chunks (two in case there was a wrap)
        if (new_rcv_idx < last_rcv_idx) {
            UART4_stream_sink.process_bytes(dma_circ_buffer + last_rcv_idx,
                    UART_RX_BUFFER_SIZE - last_rcv_idx);
            last_rcv_idx = 0;
        }
        if (new_rcv_idx > last_rcv_idx) {
            UART4_stream_sink.process_bytes(dma_circ_buffer + last_rcv_idx,
                    new_rcv_idx - last_rcv_idx);
            last_rcv_idx = new_rcv_idx;
        }
#elif defined(UART_PROTOCOL_LEGACY)
        // Process bytes in one or two chunks (two in case there was a wrap)
        if (new_rcv_idx < last_rcv_idx) {
            legacy_parse_stream(dma_circ_buffer + last_rcv_idx,
                    UART_RX_BUFFER_SIZE - last_rcv_idx);
            last_rcv_idx = 0;
        }
        if (new_rcv_idx > last_rcv_idx) {
            legacy_parse_stream(dma_circ_buffer + last_rcv_idx,
                    new_rcv_idx - last_rcv_idx);
            last_rcv_idx = new_rcv_idx;
        }
#endif
#endif

#if !defined(USB_PROTOCOL_NONE)
        // When we reach here, we are out of immediate characters to fetch out of UART buffer
        // Now we check if there is any USB processing to do: we wait for up to 1 ms,
        // before going back to checking UART again.
        const uint32_t usb_check_timeout = 1; // ms
        osStatus sem_stat = osSemaphoreWait(sem_usb_rx, usb_check_timeout);
        if (sem_stat == osOK) {
            deadline_ms = timeout_to_deadline(PROTOCOL_SERVER_TIMEOUT_MS);
#if defined(USB_PROTOCOL_NATIVE)
            usb_channel.process_packet(usb_buf, usb_len);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
            usb_stream_sink.process_bytes(usb_buf, usb_len);
#elif defined(USB_PROTOCOL_LEGACY)
            legacy_parse_cmd(usb_buf, usb_len, USB_RX_DATA_SIZE, SERIAL_PRINTF_IS_USB);
#endif
            USBD_CDC_ReceivePacket(&hUsbDeviceFS);  // Allow next packet
        }
#endif
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
