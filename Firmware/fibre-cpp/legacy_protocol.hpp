#ifndef __FIBRE_LEGACY_PROTOCOL_HPP
#define __FIBRE_LEGACY_PROTOCOL_HPP

#include <fibre/async_stream.hpp>

#ifdef FIBRE_ENABLE_CLIENT
#include "legacy_object_client.hpp"
#include <unordered_map>
#include <optional>
#include <queue>
#endif

namespace fibre {

// Default CRC-8 Polynomial: x^8 + x^5 + x^4 + x^2 + x + 1
// Can protect a 4 byte payload against toggling of up to 5 bits
//  source: https://users.ece.cmu.edu/~koopman/crc/index.html
constexpr uint8_t CANONICAL_CRC8_POLYNOMIAL = 0x37;
constexpr uint8_t CANONICAL_CRC8_INIT = 0x42;

// Default CRC-16 Polynomial: 0x9eb2 x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
// Can protect a 135 byte payload against toggling of up to 5 bits
//  source: https://users.ece.cmu.edu/~koopman/crc/index.html
// Also known as CRC-16-DNP
constexpr uint16_t CANONICAL_CRC16_POLYNOMIAL = 0x3d65;
constexpr uint16_t CANONICAL_CRC16_INIT = 0x1337;

constexpr uint8_t CANONICAL_PREFIX = 0xAA;

constexpr uint16_t PROTOCOL_VERSION = 1;


class PacketWrapper : public AsyncStreamSink {
public:
    PacketWrapper(AsyncStreamSink* tx_channel)
        : tx_channel_(tx_channel) {}

    void start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) final;
    void cancel_write(TransferHandle transfer_handle) final;

private:
    void complete(WriteResult result);

    AsyncStreamSink* tx_channel_;
    TransferHandle inner_transfer_handle_;
    uint8_t header_buf_[3];
    uint8_t trailer_buf_[2];
    const uint8_t* expected_tx_end_;
    cbufptr_t payload_buf_ = {nullptr, nullptr};
    Callback<void, WriteResult> completer_;

    enum {
        kStateIdle,
        kStateCancelling,
        kStateSendingHeader,
        kStateSendingPayload,
        kStateSendingTrailer
    } state_ = kStateIdle;
};


class PacketUnwrapper : public AsyncStreamSource {
public:
    PacketUnwrapper(AsyncStreamSource* rx_channel)
        : rx_channel_(rx_channel) {}

    void start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) final;
    void cancel_read(TransferHandle transfer_handle) final;

private:
    void complete(ReadResult result);

    AsyncStreamSource* rx_channel_;
    TransferHandle inner_transfer_handle_;
    uint8_t rx_buf_[3];
    uint8_t* expected_rx_end_;
    size_t payload_length_ = 0;
    bufptr_t payload_buf_ = {nullptr, nullptr};
    Callback<void, ReadResult> completer_;

    enum {
        kStateIdle,
        kStateCancelling,
        kStateReceivingHeader,
        kStateReceivingPayload,
        kStateReceivingTrailer
    } state_ = kStateIdle;
};


struct LegacyProtocolPacketBased {
public:
    LegacyProtocolPacketBased(AsyncStreamSource* rx_channel, AsyncStreamSink* tx_channel, size_t tx_mtu)
        : rx_channel_(rx_channel), tx_channel_(tx_channel), tx_mtu_(std::min(tx_mtu, sizeof(tx_buf_))) {}

    AsyncStreamSource* rx_channel_ = nullptr;
    AsyncStreamSink* tx_channel_ = nullptr;
    size_t tx_mtu_;
    uint8_t tx_buf_[128];
    uint8_t rx_buf_[128];

    TransferHandle tx_handle_ = 0; // non-zero while a TX operation is in progress
    uint8_t* rx_end_ = nullptr; // non-zero if an RX operation has finished but wasn't handled yet because the TX channel was busy
    StreamStatus rx_status_ = kStreamOk; // non-ok if the RX process was terminated permanently.
                                         // This signals to the TX process that it should close
                                         // the protocol instance at the next possible instant.
    
    Callback<void, LegacyProtocolPacketBased*, StreamStatus> on_stopped_ = nullptr;

#if FIBRE_ENABLE_CLIENT
    void start_endpoint_operation(uint16_t endpoint_id, cbufptr_t tx_buf, bufptr_t rx_buf, EndpointOperationHandle* handle, Callback<void, EndpointOperationResult> callback);
    void cancel_endpoint_operation(EndpointOperationHandle handle);

    LegacyObjectClient client_{this};
#endif

#if FIBRE_ENABLE_CLIENT
    void start(Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_found_root_object, Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_lost_root_object, Callback<void, LegacyProtocolPacketBased*, StreamStatus> on_stopped);
#else
    void start(Callback<void, LegacyProtocolPacketBased*, StreamStatus> on_stopped);
#endif

private:

#if FIBRE_ENABLE_CLIENT
    struct EndpointOperation {
        uint16_t seqno;
        uint16_t endpoint_id;
        cbufptr_t tx_buf;
        bool tx_done;
        bufptr_t rx_buf;
        bool rx_done;
        Callback<void, EndpointOperationResult> callback;
    };

    void start_endpoint_operation(EndpointOperation op);

    uint16_t outbound_seq_no_ = 0;
    std::vector<EndpointOperation> pending_operations_; // operations that are waiting for TX
    EndpointOperationHandle transmitting_op_ = 0; // operation that is in TX
    std::unordered_map<uint16_t, EndpointOperation> expected_acks_; // operations that are waiting for RX
#endif

    void on_write_finished(WriteResult result);
    void on_read_finished(ReadResult result);
    void on_rx_closed(StreamStatus status);
    void on_rx_tx_closed(StreamStatus status);
};


struct LegacyProtocolStreamBased {
public:
    LegacyProtocolStreamBased(AsyncStreamSource* rx_channel, AsyncStreamSink* tx_channel)
        : unwrapper_(rx_channel), wrapper_(tx_channel) {}


#if FIBRE_ENABLE_CLIENT
    void start(Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_found_root_object, Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_lost_root_object, Callback<void, LegacyProtocolPacketBased*, StreamStatus> on_stopped) {
        inner_protocol_.start(on_found_root_object, on_lost_root_object, on_stopped);
    }
#else
    void start(Callback<void, LegacyProtocolPacketBased*, StreamStatus> on_stopped) { inner_protocol_.start(on_stopped); }
#endif

private:
    PacketUnwrapper unwrapper_;
    PacketWrapper wrapper_;
    LegacyProtocolPacketBased inner_protocol_{&unwrapper_, &wrapper_, 127};
};

}

#endif // __FIBRE_LEGACY_PROTOCOL_HPP
