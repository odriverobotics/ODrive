#ifndef __INTERFACE_USB_HPP
#define __INTERFACE_USB_HPP

#include "fibre/protocol.hpp"

#include <cmsis_os.h>
#include <stdint.h>

#include <usb.hpp>

#define USB_TX_BUFFER_SIZE  64
#define USB_RX_BUFFER_SIZE  64

extern StreamSink* usb_stream_output_ptr;

typedef struct {
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t tx_overrun_cnt;
} USBStats_t;

extern USBStats_t usb_stats_;

class USBInterface {
public:
    class USBSender : public PacketSink {
    public:
        USBSender(USBTxEndpoint_t* tx_endpoint)
                : tx_endpoint_(tx_endpoint) {
            osSemaphoreDef(sem_tx_def);
            sem_tx_ = osSemaphoreCreate(osSemaphore(sem_tx_def), 1);
            osSemaphoreWait(sem_tx_, 0); // remove token (will be released once the endpoint inits)

            tx_endpoint_->on_tx_complete_.set<USBSender>(
                [](USBSender* obj) {
                    if (obj) osSemaphoreRelease(obj->sem_tx_);
                }, this);

            tx_endpoint_->on_init_.set<USBSender>(
                [](USBSender* obj) {
                    if (obj) osSemaphoreRelease(obj->sem_tx_);
                }, this);
        }

        int process_packet(const uint8_t* buffer, size_t length);
    private:
        USBTxEndpoint_t* tx_endpoint_;
        uint8_t tx_buf_[USB_TX_BUFFER_SIZE];
        osSemaphoreId sem_tx_;
    };

    class TreatPacketSinkAsStreamSink : public StreamSink {
    public:
        TreatPacketSinkAsStreamSink(PacketSink& output) : output_(output) {}
        int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes);
        size_t get_free_space() { return SIZE_MAX; }
    private:
        PacketSink& output_;
    };

    USBInterface(USBRxEndpoint_t* rx_endpoint, USBTxEndpoint_t* tx_endpoint) :
        rx_endpoint_(rx_endpoint),
        packet_output(tx_endpoint),
        stream_output(packet_output)
#if defined(USB_PROTOCOL_NATIVE)
        , channel(packet_output)
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
        , packetized_output(stream_output),
        channel(packetized_output),
        native_stream_input(channel)
#endif
    {}

    bool start_server(bool enable_ascii_protocol);

    USBRxEndpoint_t* rx_endpoint_;
    USBSender packet_output;
    TreatPacketSinkAsStreamSink stream_output;
#if defined(USB_PROTOCOL_NATIVE)
    BidirectionalPacketBasedChannel channel;
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
    StreamBasedPacketSink packetized_output;
    BidirectionalPacketBasedChannel channel;
    StreamToPacketSegmenter native_stream_input;
#endif
private:
    void handle_rx_packet(size_t len);

    uint8_t rx_buf_[USB_RX_BUFFER_SIZE];
    bool enable_ascii_protocol_ = false;
};


#endif // __INTERFACE_USB_HPP
