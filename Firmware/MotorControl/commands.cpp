
/* Includes ------------------------------------------------------------------*/

// TODO: remove this option
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


// We could theoretically implement the USB channel as a packet based channel,
// but on some platforms there's no direct USB endpoint access, so the device
// should better just behave like a serial device.
class USBSender : public StreamWriter {
public:
    int write_bytes(const uint8_t* buffer, size_t length) {
        // Loop to ensure all bytes get sent
        // TODO: add timeout
        while (length) {
            size_t chunk = length < 64 ? length : 64;
            while (CDC_Transmit_FS(
                const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                well... it's not actually. Stupid STM. */, chunk) != USBD_OK)
                osDelay(1);
            buffer += chunk;
            length -= chunk;
        }
        //printf("USB TX done\r\n"); osDelay(5);
        return 0;
    }
} usb_sender;

PacketToStreamConverter usb_packet_sender(usb_sender);
BidirectionalPacketBasedChannel usb_connection(endpoints, NUM_ENDPOINTS, usb_packet_sender);
StreamToPacketConverter usb_stream_writer(usb_connection);

class UART4Sender : public StreamWriter {
private:
    uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
public:
    int write_bytes(const uint8_t* buffer, size_t length) {
        //Check length
        if (length > UART_TX_BUFFER_SIZE)
            return -1;
        // Check if transfer is already ongoing
        if (huart4.gState != HAL_UART_STATE_READY)
            return -1;
        // memcpy data into uart_tx_buf
        memcpy(tx_buf_, buffer, length);
        // Start DMA background trasnfer
        HAL_UART_Transmit_DMA(&huart4, tx_buf_, length);
        return 0;
    }
} uart4_sender;

PacketToStreamConverter uart4_packet_sender(uart4_sender);
BidirectionalPacketBasedChannel uart4_connection(endpoints, NUM_ENDPOINTS, uart4_packet_sender);
StreamToPacketConverter UART4_stream_writer(uart4_connection);

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
            UART4_stream_writer.write_bytes(&c, 1);
        }

        // When we reach here, we are out of immediate characters to fetch out of UART buffer
        // Now we check if there is any USB processing to do: we wait for up to 1 ms,
        // before going back to checking UART again.
        int USB_check_timeout = 1;
        int32_t status = osSemaphoreWait(sem_usb_irq, USB_check_timeout);
        if (status == osOK) {
            USB_receive_packet(USBRxBuffer, USBRxBufferLen);
            // Allow receiving more bytes
            USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USBRxBuffer);
            USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        }
    }

    // If we get here, then this task is done
    vTaskDelete(osThreadGetId());
}

void USB_receive_packet(const uint8_t *buffer, size_t length) {
    //printf("[USB] got %d bytes, first is %c\r\n", length, buffer[0]); osDelay(5);
#ifdef ENABLE_LEGACY_PROTOCOL
    const uint8_t *legacy_commands = (const uint8_t*)"pvcgsmo";
    while (*legacy_commands && length) {
        if (buffer[0] == *(legacy_commands++)) {
            //printf("[USB] process legacy command %c\r\n", buffer[0]); osDelay(5);
            legacy_parse_cmd(buffer, length);
            length = 0;
        }
    }
#endif
    
    usb_stream_writer.write_bytes(buffer, length);
}
