
#include "interface_usb.h"
#include "protocol.hpp"

#include <MotorControl/utils.h>

#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include <usb_device.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

static uint8_t* usb_buf;
static uint32_t usb_len;

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
static thread_local uint32_t deadline_ms = 0;



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
} usb_packet_output;

#if !defined(USB_PROTOCOL_NATIVE)
class TreatPacketSinkAsStreamSink : public StreamSink {
public:
    TreatPacketSinkAsStreamSink(PacketSink& output) : output_(output) {}
    int process_bytes(const uint8_t* buffer, size_t length) {
        // Loop to ensure all bytes get sent
        while (length) {
            size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
            if (output_.process_packet(buffer, length) != 0)
                return -1;
            buffer += chunk;
            length -= chunk;
        }
        return 0;
    }
    size_t get_free_space() { return SIZE_MAX; }
private:
    PacketSink& output_;
} usb_stream_output(usb_packet_output);
#endif

#if defined(USB_PROTOCOL_NATIVE)
BidirectionalPacketBasedChannel usb_channel(usb_packet_output);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
PacketToStreamConverter usb_packetized_output(usb_stream_output);
BidirectionalPacketBasedChannel usb_channel(usb_packetized_output);
#endif

#if defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
StreamToPacketConverter usb_native_stream_input(usb_channel);
#endif


static void usb_server_thread(void * ctx) {
    (void) ctx;
    
    for (;;) {
        const uint32_t usb_check_timeout = 1; // ms
        osStatus sem_stat = osSemaphoreWait(sem_usb_rx, usb_check_timeout);
        if (sem_stat == osOK) {
            deadline_ms = timeout_to_deadline(PROTOCOL_SERVER_TIMEOUT_MS);
#if defined(USB_PROTOCOL_NATIVE)
            usb_channel.process_packet(usb_buf, usb_len);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
            usb_native_stream_input.process_bytes(usb_buf, usb_len);
#elif defined(USB_PROTOCOL_ASCII)
            ASCII_protocol_parse_stream(usb_buf, usb_len, usb_stream_output);
#endif
            USBD_CDC_ReceivePacket(&hUsbDeviceFS);  // Allow next packet
        }
    }
}

// Called from CDC_Receive_FS callback function, this allows the communication
// thread to handle the incoming data
void usb_process_packet(uint8_t *buf, uint32_t len) {
    usb_buf = buf;
    usb_len = len;
    osSemaphoreRelease(sem_usb_rx);
}

void serve_on_usb() {
    // Start USB communication thread
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(usb_server_thread_def), NULL);
}
