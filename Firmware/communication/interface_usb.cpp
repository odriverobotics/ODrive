
#include "interface_usb.h"
#include "ascii_protocol.hpp"

#include <MotorControl/utils.h>

#include <fibre/protocol.hpp>
#include <cmsis_os.h>
#include <freertos_vars.h>

#include <stm32_usb.hpp>
#include <usb_cdc.hpp>

USBStats_t usb_stats_ = {0};

int USBInterface::USBSender::process_packet(const uint8_t* buffer, size_t length) {
    if (!tx_endpoint_)
        return -1;
    // cannot send partial packets
    if (length > USB_TX_DATA_SIZE)
        return -1;
    // wait for USB interface to become ready
    if (osSemaphoreWait(sem_tx_, PROTOCOL_SERVER_TIMEOUT_MS) != osOK) {
        // If the host resets the device it might be that the TX-complete handler is never called
        // and the sem_tx_ semaphore is never released. To handle this we just override the
        // TX buffer if this wait times out. The implication is that the channel is no longer lossless.
        // TODO: handle endpoint reset properly
        usb_stats_.tx_overrun_cnt++;
    }
    // transmit packet
    memcpy(tx_buf_, buffer, length);
    USB_t::STATUS status = tx_endpoint_->start_tx(buffer, length);
    if (status != USB_t::OK) {
        osSemaphoreRelease(sem_tx_);
        return -1;
    }
    usb_stats_.tx_cnt++;
    return 0;
}

int USBInterface::TreatPacketSinkAsStreamSink::process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes) {
    // Loop to ensure all bytes get sent
    while (length) {
        size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
        if (output_.process_packet(buffer, length) != 0)
            return -1;
        buffer += chunk;
        length -= chunk;
        if (processed_bytes)
            *processed_bytes += chunk;
    }
    return 0;
}

bool USBInterface::start_server(bool enable_ascii_protocol) {
    if (!rx_endpoint_) {
        return false;
    }

    enable_ascii_protocol_ = enable_ascii_protocol;

    rx_endpoint_->on_rx_complete_.set<USBInterface, &USBInterface::handle_rx_packet>(*this);

    // Start RX by accepting the first packet
    rx_endpoint_->on_init_.set<USBInterface>(
        [](USBInterface* obj){
            if (obj) obj->rx_endpoint_->start_rx(obj->rx_buf_, sizeof(obj->rx_buf_));
        }, this);

    return true;
}

// Called when the RX operation started by start_rx() completes
void USBInterface::handle_rx_packet(size_t len) {
    usb_stats_.rx_cnt++;

    if (enable_ascii_protocol_) {
        ASCII_protocol_parse_stream(rx_buf_,
                len, stream_output);
    } else {
#if defined(USB_PROTOCOL_NATIVE)
        channel.process_packet(rx_buf_, len);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
        native_stream_input.process_bytes(
                rx_buf_, len, nullptr);
#endif
    }

    // Accept next packet
    if (rx_endpoint_) {
        rx_endpoint_->start_rx(rx_buf_, sizeof(rx_buf_));
    }
}
