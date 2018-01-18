/*
see protocol.md for the protocol specification
*/

#ifndef __PROTOCOL_HPP
#define __PROTOCOL_HPP

// TODO: resolve assert
#define assert(expr)

#include <functional>
#include <limits>
#include <cstring>
#include "crc.hpp"


constexpr uint8_t SYNC_BYTE = 0xAA;
constexpr uint8_t CRC8_INIT = 0x42;
constexpr uint16_t CRC16_INIT = 0x1337;
constexpr uint16_t PROTOCOL_VERSION = 1;

// This value must not be larger than USB_TX_DATA_SIZE defined in usbd_cdc_if.h
//Oskar: What's the error? What values work? Does 63 work? Ideally we figure out how to get 64 to work, but if not let's find something better than 32.
constexpr uint16_t TX_BUF_SIZE = 32; // does not work with 64 for some reason
constexpr uint16_t RX_BUF_SIZE = 128; // larger values than 128 have currently no effect because of protocol limitations

// Maximum time we allocate for processing and responding to a request
constexpr uint32_t PROTOCOL_SERVER_TIMEOUT_MS = 10;

template<typename T>
inline size_t write_le(T value, uint8_t* buffer);

template<typename T>
inline size_t read_le(T* value, const uint8_t* buffer);

template<>
inline size_t write_le<uint8_t>(uint8_t value, uint8_t* buffer) {
    buffer[0] = value;
    return 2;
}

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
inline size_t write_le<int32_t>(int32_t value, uint8_t* buffer) {
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
inline size_t read_le<uint8_t>(uint8_t* value, const uint8_t* buffer) {
    *value = buffer[0];
    return 2;
}

template<>
inline size_t read_le<uint16_t>(uint16_t* value, const uint8_t* buffer) {
    *value = (static_cast<uint16_t>(buffer[0]) << 0) |
             (static_cast<uint16_t>(buffer[1]) << 8);
    return 2;
}

template<>
inline size_t read_le<int32_t>(int32_t* value, const uint8_t* buffer) {
    *value = (static_cast<int32_t>(buffer[0]) << 0) |
             (static_cast<int32_t>(buffer[1]) << 8) |
             (static_cast<int32_t>(buffer[2]) << 16) |
             (static_cast<int32_t>(buffer[3]) << 24);
    return 4;
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

// @brief Reads a value of type T from the buffer.
// @param buffer    Pointer to the buffer to be read. The pointer is updated by the number of bytes that were read.
// @param length    The number of available bytes in buffer. This value is updated to subtract the bytes that were read.
template<typename T>
static inline T read_le(const uint8_t** buffer, size_t* length) {
    T result;
    size_t cnt = read_le(&result, *buffer);
    *buffer += cnt;
    *length -= cnt;
    return result;
}

class PacketSink {
public:
    // @brief Processes a packet.
    // The blocking behavior shall depend on the thread-local deadline_ms variable.
    // @return: 0 on success, otherwise a non-zero error code
    // TODO: define what happens when the packet is larger than what the implementation can handle.
    virtual int process_packet(const uint8_t* buffer, size_t length) = 0;
};

class StreamSink {
public:
    // @brief Processes a chunk of bytes that is part of a continuous stream.
    // The blocking behavior shall depend on the thread-local deadline_ms variable.
    // @return: 0 on success, otherwise a non-zero error code
    virtual int process_bytes(const uint8_t* buffer, size_t length) = 0;

    // @brief Returns the number of bytes that can still be written to the stream.
    // Shall return SIZE_MAX if the stream has unlimited lenght.
    virtual size_t get_free_space() = 0;
};


class StreamToPacketConverter : public StreamSink {
public:
    StreamToPacketConverter(PacketSink& output) :
        output_(output)
    {
    };

    int process_bytes(const uint8_t *buffer, size_t length);
    
    size_t get_free_space() { return SIZE_MAX; }

private:
    uint8_t header_buffer_[3];
    size_t header_index_ = 0;
    uint8_t packet_buffer_[RX_BUF_SIZE];
    size_t packet_index_ = 0;
    size_t packet_length_ = 0;
    PacketSink& output_;
};


class PacketToStreamConverter : public PacketSink {
public:
    PacketToStreamConverter(StreamSink& output) :
        output_(output)
    {
    };
    
    int process_packet(const uint8_t *buffer, size_t length);

private:
    StreamSink& output_;
};


// Implements the StreamSink interface by writing into a fixed size
// memory buffer.
class MemoryStreamSink : public StreamSink {
public:
    MemoryStreamSink(uint8_t *buffer, size_t length) :
        buffer_(buffer),
        buffer_length_(length) {}

    // Returns 0 on success and -1 if the buffer could not accept everything because it became full
    int process_bytes(const uint8_t* buffer, size_t length) {
        int status = 0;
        if (length > buffer_length_) {
            length = buffer_length_;
            status = -1;
        }
        memcpy(buffer_, buffer, length);
        buffer_ += length;
        buffer_length_ -= length;
        return status;
    }

    size_t get_free_space() { return buffer_length_; }

private:
    uint8_t * buffer_;
    size_t buffer_length_;
};

// Implements the StreamSink interface by discarding the first couple of bytes
// and then forwarding the rest to another stream.
class NullStreamSink : public StreamSink {
public:
    NullStreamSink(size_t skip, StreamSink& follow_up_stream) :
        skip_(skip),
        follow_up_stream_(follow_up_stream) {}

    // Returns 0 on success and -1 if the buffer could not accept everything because it became full
    int process_bytes(const uint8_t* buffer, size_t length) {
        if (skip_ < length) {
            buffer += skip_;
            length -= skip_;
            skip_ = 0;
            return follow_up_stream_.process_bytes(buffer, length);
        } else {
            skip_ -= length;
            return 0;
        }
    }

    size_t get_free_space() { return skip_ + follow_up_stream_.get_free_space(); }

private:
    size_t skip_;
    StreamSink& follow_up_stream_;
};



// Implements the StreamSink interface by calculating the CRC16 checksum
// on the data that is sent to it.
class CRC16Calculator : public StreamSink {
public:
    CRC16Calculator(uint16_t crc16_init) :
        crc16_(crc16_init) {}

    int process_bytes(const uint8_t* buffer, size_t length) {
        crc16_ = calc_crc16(crc16_, buffer, length);
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }

    uint16_t get_crc16() { return crc16_; }
private:
    uint16_t crc16_;
};



typedef enum {
    PROPERTY,
    BEGIN_OBJECT,
    BEGIN_FUNCTION,
    CLOSE_TREE
} EndpointType_t;


// @brief Endpoint request handler
//
// When passed a valid endpoint context, implementing functions shall handle an
// endpoint read/write request by reading the provided input data and filling in
// output data. The exact semantics of this function depends on the corresponding
// endpoint's specification.
//
// @param input: pointer to the input data
// @param input_length: number of available input bytes
// @param output: The stream where to write the output to. Can be null.
//                The handler shall abort as soon as the stream returns
//                a non-zero error code on write.
typedef std::function<void(void* ctx, const uint8_t* input, size_t input_length, StreamSink* output)> EndpointHandler;


template<typename T>
void default_read_endpoint_handler(void* ctx, const uint8_t* input, size_t input_length, StreamSink* output) {
    const T* value = reinterpret_cast<const T*>(ctx);
    // If the old value was requested, call the corresponding little endian serialization function
    if (output) {
        // TODO: make buffer size dependent on the type
        uint8_t buffer[sizeof(T)];
        size_t cnt = write_le<T>(*value, buffer);
        if (cnt <= output->get_free_space())
            output->process_bytes(buffer, cnt);
    }
}

template<typename T>
void default_readwrite_endpoint_handler(void* ctx, const uint8_t* input, size_t input_length, StreamSink* output) {
    T* value = reinterpret_cast<T*>(ctx);
    
    // Read the endpoint value into output
    default_read_endpoint_handler<T>(ctx, input, input_length, output);
    
    // If a new value was passed, call the corresponding little endian deserialization function
    uint8_t buffer[sizeof(T)] = { 0 }; // TODO: make buffer size dependent on the type
    if (input_length >= sizeof(buffer))
        read_le<T>(value, input);
}

static void trigger_endpoint_handler(void* ctx, const uint8_t* input, size_t input_length, StreamSink* output) {
    (void) input;
    (void) input_length;
    (void) output;
    std::function<void(void)> function = reinterpret_cast<void(*)()>(ctx);
    function();
}


template<typename T>
static inline const char* get_default_json_modifier();

template<>
inline const char* get_default_json_modifier<const float>() {
    return "\"type\":\"float\",\"access\":\"r\"";
}
template<>
inline const char* get_default_json_modifier<float>() {
    return "\"type\":\"float\",\"access\":\"rw\"";
}
template<>
inline const char* get_default_json_modifier<const int32_t>() {
    return "\"type\":\"int32\",\"access\":\"r\"";
}
template<>
inline const char* get_default_json_modifier<int32_t>() {
    return "\"type\":\"int32\",\"access\":\"rw\"";
}
template<>
inline const char* get_default_json_modifier<const uint32_t>() {
    return "\"type\":\"uint32\",\"access\":\"r\"";
}
template<>
inline const char* get_default_json_modifier<const uint16_t>() {
    return "\"type\":\"uint16\",\"access\":\"r\"";
}
template<>
inline const char* get_default_json_modifier<uint16_t>() {
    return "\"type\":\"uint16\",\"access\":\"rw\"";
}
template<>
inline const char* get_default_json_modifier<const uint8_t>() {
    return "\"type\":\"uint8\",\"access\":\"r\"";
}
template<>
inline const char* get_default_json_modifier<uint8_t>() {
    return "\"type\":\"uint8\",\"access\":\"rw\"";
}

class Endpoint {
public:
    const char* const name_;

    Endpoint(const char* name, EndpointType_t type, EndpointHandler handler, const char* json_modifier, void *ctx) :
        name_(name),
        type_(type),
        handler_(handler),
        json_modifier_(json_modifier),
        ctx_(ctx)
    {
    }

    template<typename T>
    static Endpoint make_property(const char* name, const T* ctx) {
        return Endpoint(name, PROPERTY,
            default_read_endpoint_handler<T>,
            get_default_json_modifier<const T>(),
            const_cast<T*>(ctx) /* it's safe to cast the const away here because we
            know that the default_read_endpoint_handler immediately adds it back */);
    }

    template<typename T>
    static Endpoint make_property(const char* name, T* ctx) {
        return Endpoint(name, PROPERTY,
            default_readwrite_endpoint_handler<T>,
            get_default_json_modifier<T>(), ctx);
    }
    
    static Endpoint make_object(const char* name) {
        return Endpoint(name, BEGIN_OBJECT, nullptr,
            "\"type\":\"object\"", nullptr);
    }

    static Endpoint make_function(const char* name, void(*function)(void)) {
        return Endpoint(name, BEGIN_FUNCTION, trigger_endpoint_handler,
            "\"type\":\"function\"", reinterpret_cast<void*>(function));
    }

    static Endpoint close_tree() {
        return Endpoint(nullptr, CLOSE_TREE, nullptr, nullptr, nullptr);
    }

    void write_json(size_t id, bool* need_comma, StreamSink* output) const;

    void handle(const uint8_t* input, size_t input_length, StreamSink* output) const {
        if (handler_)
            return handler_(ctx_, input, input_length, output);
    }

private:
    const EndpointType_t type_;
    const EndpointHandler handler_;
    const char* json_modifier_;
    void* const ctx_;
};


/* @brief Handles the communication protocol on one channel.
*
* When instantiated with a list of endpoints and an output packet sink,
* objects of this class will handle packets passed into process_packet,
* pass the relevant data to the corresponding endpoints and dispatch response
* packets on the output.
*/
class BidirectionalPacketBasedChannel : public PacketSink {
public:
    BidirectionalPacketBasedChannel(const Endpoint* endpoints, size_t n_endpoints, PacketSink& output) :
        global_endpoints_(endpoints),
        n_endpoints_(NUM_CHANNEL_SPECIFIC_ENDPOINTS + n_endpoints),
        output_(output),
        json_crc_(calculate_json_crc16())
    {
    }

    int process_packet(const uint8_t* buffer, size_t length);

private:
    
    uint16_t calculate_json_crc16(void);
    void interface_query(const uint8_t* input, size_t input_length, StreamSink* output);

    static void interface_query_handler(void* ctx, const uint8_t* input, size_t input_length, StreamSink* output) {
        reinterpret_cast<BidirectionalPacketBasedChannel*>(ctx)->interface_query(input, input_length, output);
    }
    
    static void subscription_handler(void* ctx, const uint8_t* input, size_t input_length, StreamSink* output) {
        reinterpret_cast<BidirectionalPacketBasedChannel*>(ctx)->subscription(input, input_length, output);
    }

    const Endpoint channel_specific_endpoints_[1] = {
        Endpoint("", PROPERTY, BidirectionalPacketBasedChannel::interface_query_handler, "\"type\":\"json\",\"access\":\"rw\"", this),
        //Endpoint("subscriptions", PROPERTY, BidirectionalPacketBasedChannel::subscription_handler, nullptr, this)
    };
    static constexpr size_t NUM_CHANNEL_SPECIFIC_ENDPOINTS = sizeof(channel_specific_endpoints_) / sizeof(channel_specific_endpoints_[0]);
    
    const Endpoint* get_endpoint(size_t index) {
        if (index < NUM_CHANNEL_SPECIFIC_ENDPOINTS){
            return &channel_specific_endpoints_[index];
        } else if (index < n_endpoints_) {
            return &global_endpoints_[index - NUM_CHANNEL_SPECIFIC_ENDPOINTS];
        } else {
            return nullptr;
        }
    }

    void subscription(const uint8_t* input, size_t input_length, StreamSink* output) {
        // TODO: handle
        return;
    }

    const Endpoint * const global_endpoints_;
    size_t n_endpoints_;
    PacketSink& output_;
    uint8_t tx_buf_[TX_BUF_SIZE];
    const uint16_t json_crc_;
};

#endif
