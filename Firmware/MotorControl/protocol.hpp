/*
* # ODrive Communication Protocol #
*
* Communicating with an ODrive consists of a series of endpoint operations.
* operations. An endpoint is usually a single number or string.
* The available endpoints can be enumerated by reading the JSON from endpoint 0
* and can theoretically be different for each channel (they are not in practice).
*
* Each endpoint operation can send bytes to one endpoint (referenced by it's ID)
* and at the same time receive bytes from the same endpoint. The semantics of
* these payloads are specific to each endpoint's type, the name of which is
* indicated in the JSON.
*
* For instance an int32 endpoint's input and output is a 4 byte little endian
* representation. In general the convention for combined read/write requests is
* _exchange_, i.e. the returned value is the old value. Custom endpoint handlers
* may be non-compliant.
*
* ## Stream format: ##
* (For instance UART)
*
*   1. sync-byte
*   2. packet-length (0-127, larger values are reserved)
*   3. crc8(sync-byte + packet-length)
*   4. packet
*
* ## Packet format: ##
* (For instance USB)
*
* __Request__
*
*   1. seq-no, MSB = 0
*   2. endpoint-id, MSB = "expect ack"
*   3. expected_response_size
*   4. payload (contains offset if required)
*   5. crc16(seq-no + endpoint-id + payload + crc16(protocol_version + JSON))
*      For endpoint 0 the JSON-CRC is not included
*
* __Response__
*
*   1. seq-no, MSB = 1
*   2. payload
*   3. crc16(seq-no + payload + protocol_version)
*
*/


#ifndef __PROTOCOL_HPP
#define __PROTOCOL_HPP

// TODO: resolve assert
#define assert(expr)

#include <functional>
#include <limits>
#include <cstring>
#include "crc.hpp"


constexpr uint8_t SYNC_BYTE = '$';
constexpr uint8_t CRC8_INIT = 0x42;
constexpr uint16_t CRC16_INIT = 0x1337;
constexpr uint16_t PROTOCOL_VERSION = 1;
constexpr uint16_t TX_BUF_SIZE = 32; // does not work with 64 for some reason


template<typename T>
inline size_t write_le(T value, uint8_t* buffer);

template<typename T>
inline size_t read_le(T* value, const uint8_t* buffer);

template<>
inline size_t write_le<uint16_t>(uint16_t value, uint8_t* buffer) {
    buffer[0] = (value >> 0) & 0xff;
    buffer[1] = (value >> 8) & 0xff;
    return 2;
}

template<>
inline size_t write_le<uint32_t>(uint32_t value, uint8_t* buffer) {
    buffer[0] = (value >> 0) & 0xff;
    buffer[1] = (value >> 8) & 0xff;
    buffer[2] = (value >> 16) & 0xff;
    buffer[3] = (value >> 24) & 0xff;
    return 4;
}

template<>
inline size_t write_le<float>(float value, uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");
    return write_le<uint32_t>(*reinterpret_cast<const uint32_t*>(&value), buffer);
}

template<>
inline size_t read_le<uint16_t>(uint16_t* value, const uint8_t* buffer) {
    *value = (static_cast<uint16_t>(buffer[0]) << 0) |
            (static_cast<uint16_t>(buffer[1]) << 8);
    return 2;
}

template<>
inline size_t read_le<uint32_t>(uint32_t* value, const uint8_t* buffer) {
    *value = (static_cast<uint32_t>(buffer[0]) << 0) |
            (static_cast<uint32_t>(buffer[1]) << 8) |
            (static_cast<uint32_t>(buffer[2]) << 16) |
            (static_cast<uint32_t>(buffer[3]) << 24);
    return 4;
}

template<>
inline size_t read_le<float>(float* value, const uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");
    return read_le(reinterpret_cast<uint32_t*>(value), buffer);
}


template<typename T>
static inline T read_le(const uint8_t** buffer, size_t* length) {
    T result;
    size_t cnt = read_le(&result, *buffer);
    *buffer += cnt;
    *length -= cnt;
    return result;
}


typedef enum {
    AS_JSON,
    AS_INT32_ARRAY,
    AS_FLOAT,
    AS_INT,
    AS_BOOL,
    AS_UINT16,
    BEGIN_TREE,
    END_TREE
} EndpointTypeID_t;


// @brief Endpoint request handler
//
// When passed a valid endpoint context, implementing functions shall handle an
// endpoint read/write request by reading the provided input data and filling in
// output data. The exact semantics of this function depends on the corresponding
// endpoint's specification.
//
// @param input: pointer to the input data
// @param input_length: number of available input bytes
// @param output: pointer to where the output data should go.
//        If *output_length is non-zero, this is guaranteed not to be NULL.
// @param output_length: pointer to the remaining length of the output buffer.
//        The handler must update this value to subtract the number of written bytes.
//        The pointer itself is guaranteed not to be NULL.
typedef std::function<void(void* ctx, const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length)> EndpointHandler;


template<typename T>
void default_read_endpoint_handler(void* ctx, const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
    const T* value = reinterpret_cast<const T*>(ctx);
    // If the old value was requested, call the corresponding little endian serialization function
    if (*output_length) {
        uint8_t buffer[8]; // TODO: make buffer size dependent on the type
        size_t cnt = write_le<T>(*value, buffer);
        if (cnt > *output_length)
            cnt = *output_length;
        memcpy(output, buffer, cnt);
        *output_length -= cnt;
    }
}

template<typename T>
void default_readwrite_endpoint_handler(void* ctx, const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
    T* value = reinterpret_cast<T*>(ctx);
    
    // Call the handler for the const version of the endpoint's type - i.e. read the endpoint value into output
    default_read_endpoint_handler<T>(ctx, input, input_length, output, output_length);
    
    // If a new value was passed, call the corresponding little endian deserialization function
    if (input_length) {
        uint8_t buffer[8] = { 0 }; // TODO: make buffer size dependent on the type
        if (input_length > sizeof(buffer))
            input_length = sizeof(buffer);
        memcpy(buffer, input, input_length); // TODO: abort if not enough bytes received
        read_le<T>(value, buffer);
    }
}

class Endpoint {
public:
    const char* const name_;

    Endpoint(const char* name, EndpointTypeID_t type_id, EndpointHandler handler, const char* json_modifier, void *ctx) :
        name_(name),
        type_id_(type_id),
        handler_(handler),
        json_modifier_(json_modifier),
        ctx_(ctx)
    {
    }

    template<typename T>
    Endpoint(const char* name, const T* ctx) :
        Endpoint(name, AS_FLOAT, default_read_endpoint_handler<T>, "\"access\":\"r\"",
        const_cast<T*>(ctx) /* it's safe to cast the const away here because we
        know that the default_read_endpoint_handler immediately adds it back */) {}

    template<typename T>
    Endpoint(const char* name, T* ctx) :
        Endpoint(name, AS_FLOAT, default_readwrite_endpoint_handler<T>, "\"access\":\"rw\"", ctx) {}


    void write_json(size_t id, size_t* skip, uint8_t** output, size_t* output_length, bool* need_comma);

    void handle(const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
        if (handler_)
            return handler_(ctx_, input, input_length, output, output_length);
    }

private:
    const EndpointTypeID_t type_id_;
    const EndpointHandler handler_;
    const char* json_modifier_;
    void* const ctx_;
};


class PacketWriter {
public:
    // @brief Processes a packet.
    // TODO: define what happens when the output is congested. We can either drop the data or block.
    // TODO: define what happens when the packet is larger than what the implementation can handle.
    virtual int write_packet(const uint8_t* buffer, size_t length) = 0;
};


class StreamWriter {
public:
    // @brief Processes a chunk of bytes that is part of a continuous stream.
    // TODO: define what happens when the output is congested. We can either drop the data or block.
    virtual int write_bytes(const uint8_t* buffer, size_t length) = 0;
};


class StreamToPacketConverter : public StreamWriter {
public:
    StreamToPacketConverter(PacketWriter& output) :
        output_(output)
    {
    };

    int write_bytes(const uint8_t *buffer, size_t length);

private:
    uint8_t header_buffer_[3];
    size_t header_index_ = 0;
    uint8_t packet_buffer_[128];
    size_t packet_index_ = 0;
    size_t packet_length_ = 0;
    PacketWriter& output_;
};


class PacketToStreamConverter : public PacketWriter {
public:
    PacketToStreamConverter(StreamWriter& output) :
        output_(output)
    {
    };
    
    int write_packet(const uint8_t *buffer, size_t length);

private:
    StreamWriter& output_;
};


class BidirectionalPacketBasedChannel : public PacketWriter {
public:
    BidirectionalPacketBasedChannel(Endpoint* endpoints, size_t n_endpoints, PacketWriter& output) :
        global_endpoints_(endpoints),
        n_endpoints_(NUM_CHANNEL_SPECIFIC_ENDPOINTS + n_endpoints),
        output_(output),
        json_crc_(calculate_json_crc16())
    {
    }

    int write_packet(const uint8_t* buffer, size_t length);

private:
    
    uint16_t calculate_json_crc16(void);
    void interface_query(const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length);

    static void interface_query_handler(void* ctx, const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
        reinterpret_cast<BidirectionalPacketBasedChannel*>(ctx)->interface_query(input, input_length, output, output_length);
    }
    
    static void subscription_handler(void* ctx, const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
        reinterpret_cast<BidirectionalPacketBasedChannel*>(ctx)->subscription(input, input_length, output, output_length);
    }

    //ReceiveCallback c = BidirectionalPacketBasedChannel::receive_fetch_request_ex;
    Endpoint channel_specific_endpoints_[2] = {
        Endpoint("", AS_JSON, BidirectionalPacketBasedChannel::interface_query_handler, nullptr, this),
        Endpoint("subscriptions", AS_INT32_ARRAY, BidirectionalPacketBasedChannel::subscription_handler, nullptr, this)
    };
    static constexpr size_t NUM_CHANNEL_SPECIFIC_ENDPOINTS = sizeof(channel_specific_endpoints_) / sizeof(channel_specific_endpoints_[0]);

    Endpoint* global_endpoints_;
    size_t n_endpoints_;
    PacketWriter& output_;

    Endpoint* get_endpoint(size_t index) {
        if (index < NUM_CHANNEL_SPECIFIC_ENDPOINTS){
            return &channel_specific_endpoints_[index];
        } else if (index < n_endpoints_) {
            return &global_endpoints_[index - NUM_CHANNEL_SPECIFIC_ENDPOINTS];
        } else {
            return nullptr;
        }
    }

    void subscription(const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
        // TODO: handle
        return;
    }

    size_t expected_seq_no_ = 0;
    uint8_t tx_buf_[TX_BUF_SIZE];
    const uint16_t json_crc_;
};

#endif
