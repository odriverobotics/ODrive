
/* Includes ------------------------------------------------------------------*/

// TODO: remove this option
// and once the legacy protocol is phased out, remove the seq-no hack in protocol.py
#define ENABLE_LEGACY_PROTOCOL

#include "low_level.h"
#include "protocol.hpp"
#include "freertos_vars.h"
#include "commands.h"

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

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
// TODO: make command to switch gpio_mode during run-time
static const GpioMode_t gpio_mode = GPIO_MODE_UART;     //GPIO 1,2 is UART Tx,Rx
// static const GpioMode_t gpio_mode = GPIO_MODE_STEP_DIR; //GPIO 1,2 is M0 Step,Dir

/* Private variables ---------------------------------------------------------*/

/* Variables exposed to USB & UART via read/write commands */
// TODO: include range information in JSON description

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

// clang-format off
// TODO: Autogenerate this table. It will come up again very soon in the Arduino library.
const Endpoint endpoints[] = {
    Endpoint::make_property("vbus_voltage", const_cast<const float*>(&vbus_voltage)),
    Endpoint::make_property("elec_rad_per_enc", const_cast<const float*>(&elec_rad_per_enc)),
    Endpoint::make_object("motor0"),
        Endpoint::make_property("pos_setpoint", &motors[0].pos_setpoint),
        Endpoint::make_property("pos_gain", &motors[0].pos_gain),
        Endpoint::make_property("vel_setpoint", &motors[0].vel_setpoint),
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
    Endpoint::close_tree() // motor0
};
// clang-format on

constexpr size_t NUM_ENDPOINTS = sizeof(endpoints) / sizeof(endpoints[0]);



//#define STREAM_ON_USB
#ifdef STREAM_ON_USB
// We could theoretically implement the USB channel as a packet based channel,
// but on some platforms there's no direct USB endpoint access, so the device
// should better just behave like a serial device.

class USBSender : public StreamSink {
public:
    int process_bytes(const uint8_t* buffer, size_t length) {
        // Loop to ensure all bytes get sent
        // TODO: add timeout
        while (length) {
            size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
            while (CDC_Transmit_FS(
                const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                well... it's not actually. Stupid STM. */, chunk) != USBD_OK)
                osDelay(1);
            buffer += chunk;
            length -= chunk;
        }
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }
} usb_sender;

PacketToStreamConverter usb_packet_sender(usb_sender);
BidirectionalPacketBasedChannel usb_connection(endpoints, NUM_ENDPOINTS, usb_packet_sender);
StreamToPacketConverter usb_stream_sink(usb_connection);

#else

class USBSender : public PacketSink {
public:
    int process_packet(const uint8_t* buffer, size_t length) {
        // cannot send partial packets
        if (length > USB_TX_DATA_SIZE)
            return -1;
        while (CDC_Transmit_FS(
            const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
            well... it's not actually. Stupid STM. */, length) != USBD_OK)
            osDelay(1);
        return 0;
    }
} usb_sender;

BidirectionalPacketBasedChannel usb_connection(endpoints, NUM_ENDPOINTS, usb_sender);
#endif

class UART4Sender : public StreamSink {
public:
    int process_bytes(const uint8_t* buffer, size_t length) {
        //Check length
        if (length > UART_TX_BUFFER_SIZE)
            return -1;
        // Loop until the UART is ready
        // TODO: implement ring buffer to get a more continuous stream of data
        while (huart4.gState != HAL_UART_STATE_READY)
            osDelay(1);
        // memcpy data into uart_tx_buf
        memcpy(tx_buf_, buffer, length);
        // Start DMA background trasnfer
        HAL_UART_Transmit_DMA(&huart4, tx_buf_, length);
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }
private:
    uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
} uart4_sender;

PacketToStreamConverter uart4_packet_sender(uart4_sender);
BidirectionalPacketBasedChannel uart4_connection(endpoints, NUM_ENDPOINTS, uart4_packet_sender);
StreamToPacketConverter UART4_stream_sink(uart4_connection);

/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

void init_communication(void) {
    switch (gpio_mode) {
        case GPIO_MODE_UART: {
            SetGPIO12toUART();
        } break;
        case GPIO_MODE_STEP_DIR: {
            SetGPIO12toStepDir();
        }
    }
}

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void communication_task(void const * argument) {
    (void) argument;
    
    //DMA open loop continous circular buffer
    //1ms delay periodic, chase DMA ptr around

    #define UART_RX_BUFFER_SIZE 64
    static uint8_t dma_circ_buffer[UART_RX_BUFFER_SIZE];

    // DMA is set up to recieve in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));

    uint32_t last_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;

    // Re-run state-machine forever
    for (;;) {
        // Check for UART errors and restart recieve DMA transfer if required
        if (huart4.ErrorCode != HAL_UART_ERROR_NONE) {
            HAL_UART_AbortReceive(&huart4);
            HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));
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
            UART4_stream_sink.process_bytes(&c, 1);
        }

        // When we reach here, we are out of immediate characters to fetch out of UART buffer
        // Now we check if there is any USB processing to do: we wait for up to 1 ms,
        // before going back to checking UART again.
        int USB_check_timeout = 1;
        int32_t status = osSemaphoreWait(sem_usb_irq, USB_check_timeout);
        if (status == osOK) {
            // We have a new incoming USB transmission: handle it
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
            // Let the irq (OTG_FS_IRQHandler) fire again.
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }

    // If we get here, then this task is done
    vTaskDelete(osThreadGetId());
}

void USB_receive_packet(const uint8_t *buffer, size_t length) {
    //printf("[USB] got %d bytes, first is %c\r\n", length, buffer[0]); osDelay(5);
#ifdef ENABLE_LEGACY_PROTOCOL
    const uint8_t* legacy_commands = (const uint8_t*)"pvcgsmo";
    while (*legacy_commands && length) {
        if (buffer[0] == *(legacy_commands++)) {
            //printf("[USB] process legacy command %c\r\n", buffer[0]); osDelay(5);
            legacy_parse_cmd(buffer, length);
            length = 0;
        }
    }
#endif
    
#ifdef STREAM_ON_USB
    usb_stream_sink.process_bytes(buffer, length);
#else
    usb_connection.process_packet(buffer, length);
#endif
}
