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

// Note that this option cannot be used to debug UART because it prints on UART
//#define DEBUG_PROTOCOL
#ifdef DEBUG_PROTOCOL
#define LOG_PROTO(...)  do { printf(__VA_ARGS__); osDelay(10); } while (0)
#else
#define LOG_PROTO(...)  ((void) 0)
#endif


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
inline size_t write_le<bool>(bool value, uint8_t* buffer) {
    buffer[0] = value ? 1 : 0;
    return 1;
}

template<>
inline size_t write_le<uint8_t>(uint8_t value, uint8_t* buffer) {
    buffer[0] = value;
    return 1;
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
inline size_t write_le<uint64_t>(uint64_t value, uint8_t* buffer) {
    buffer[0] = (value >> 0) & 0xff;
    buffer[1] = (value >> 8) & 0xff;
    buffer[2] = (value >> 16) & 0xff;
    buffer[3] = (value >> 24) & 0xff;
    buffer[4] = (value >> 32) & 0xff;
    buffer[5] = (value >> 40) & 0xff;
    buffer[6] = (value >> 48) & 0xff;
    buffer[7] = (value >> 56) & 0xff;
    return 8;
}

template<>
inline size_t write_le<float>(float value, uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");
    const uint32_t * value_as_uint32 = reinterpret_cast<const uint32_t*>(&value);
    return write_le<uint32_t>(*value_as_uint32, buffer);
}

template<>
inline size_t read_le<bool>(bool* value, const uint8_t* buffer) {
    *value = buffer[0];
    return 1;
}

template<>
inline size_t read_le<uint8_t>(uint8_t* value, const uint8_t* buffer) {
    *value = buffer[0];
    return 1;
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
inline size_t read_le<uint64_t>(uint64_t* value, const uint8_t* buffer) {
    *value = (static_cast<uint64_t>(buffer[0]) << 0) |
             (static_cast<uint64_t>(buffer[1]) << 8) |
             (static_cast<uint64_t>(buffer[2]) << 16) |
             (static_cast<uint64_t>(buffer[3]) << 24) |
             (static_cast<uint64_t>(buffer[4]) << 32) |
             (static_cast<uint64_t>(buffer[5]) << 40) |
             (static_cast<uint64_t>(buffer[6]) << 48) |
             (static_cast<uint64_t>(buffer[7]) << 56);
    return 8;
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
void default_readwrite_endpoint_handler(const T* value, const uint8_t* input, size_t input_length, StreamSink* output) {
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
void default_readwrite_endpoint_handler(T* value, const uint8_t* input, size_t input_length, StreamSink* output) {
    // Read the endpoint value into output
    default_readwrite_endpoint_handler<T>(const_cast<const T*>(value), input, input_length, output);
    
    // If a new value was passed, call the corresponding little endian deserialization function
    uint8_t buffer[sizeof(T)] = { 0 }; // TODO: make buffer size dependent on the type
    if (input_length >= sizeof(buffer))
        read_le<T>(value, input);
}



template<typename T>
static inline const char* get_default_json_modifier();

template<>
inline constexpr const char* get_default_json_modifier<const float>() {
    return "\"type\":\"float\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<float>() {
    return "\"type\":\"float\",\"access\":\"rw\"";
}
template<>
inline constexpr const char* get_default_json_modifier<const uint64_t>() {
    return "\"type\":\"uint64\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<uint64_t>() {
    return "\"type\":\"uint64\",\"access\":\"rw\"";
}
template<>
inline constexpr const char* get_default_json_modifier<const int32_t>() {
    return "\"type\":\"int32\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<int32_t>() {
    return "\"type\":\"int32\",\"access\":\"rw\"";
}
template<>
inline constexpr const char* get_default_json_modifier<const uint32_t>() {
    return "\"type\":\"uint32\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<uint32_t>() {
    return "\"type\":\"uint32\",\"access\":\"rw\"";
}
template<>
inline constexpr const char* get_default_json_modifier<const uint16_t>() {
    return "\"type\":\"uint16\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<uint16_t>() {
    return "\"type\":\"uint16\",\"access\":\"rw\"";
}
template<>
inline constexpr const char* get_default_json_modifier<const uint8_t>() {
    return "\"type\":\"uint8\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<uint8_t>() {
    return "\"type\":\"uint8\",\"access\":\"rw\"";
}
template<>
inline constexpr const char* get_default_json_modifier<const bool>() {
    return "\"type\":\"bool\",\"access\":\"r\"";
}
template<>
inline constexpr const char* get_default_json_modifier<bool>() {
    return "\"type\":\"bool\",\"access\":\"rw\"";
}

class Endpoint {
public:
    //const char* const name_;
    virtual void handle(const uint8_t* input, size_t input_length, StreamSink* output) = 0;
    virtual bool get_string(char * output, size_t length) { return false; };
    virtual bool set_string(char * buffer, size_t length) { return false; }
};

class EndpointProvider {
public:
    virtual size_t get_endpoint_count() = 0;
    virtual void write_json(size_t id, StreamSink* output) = 0;
    virtual Endpoint* get_by_name(char * name, size_t length) = 0;
    virtual void register_endpoints(Endpoint** list, size_t id, size_t length) = 0;
};


static inline int write_string(const char* str, StreamSink* output) {
    return output->process_bytes(reinterpret_cast<const uint8_t*>(str), strlen(str));
}


/* @brief Handles the communication protocol on one channel.
*
* When instantiated with a list of endpoints and an output packet sink,
* objects of this class will handle packets passed into process_packet,
* pass the relevant data to the corresponding endpoints and dispatch response
* packets on the output.
*/
class BidirectionalPacketBasedChannel : public PacketSink {
public:
    BidirectionalPacketBasedChannel(PacketSink& output) :
        output_(output)
    { }

    int process_packet(const uint8_t* buffer, size_t length);
private:
    PacketSink& output_;
    uint8_t tx_buf_[TX_BUF_SIZE];
};


template<typename ... TMembers>
struct MemberList;

template<>
struct MemberList<> {
public:
    static constexpr size_t endpoint_count = 0;
    static constexpr bool is_empty = true;
    void write_json(size_t id, StreamSink* output) {
        // no action
    }
    void register_endpoints(Endpoint** list, size_t id, size_t length) {
        // no action
    }
    Endpoint* get_by_name(const char * name, size_t length) {
        return nullptr;
    }
    std::tuple<> get_names_as_tuple() const { return std::tuple<>(); }
};

template<typename TMember, typename ... TMembers>
struct MemberList<TMember, TMembers...> {
public:
    static constexpr size_t endpoint_count = TMember::endpoint_count + MemberList<TMembers...>::endpoint_count;
    static constexpr bool is_empty = false;

    MemberList(TMember&& this_member, TMembers&&... subsequent_members) :
        this_member_(std::forward<TMember>(this_member)),
        subsequent_members_(std::forward<TMembers>(subsequent_members)...) {}

    MemberList(TMember&& this_member, MemberList<TMembers...>&& subsequent_members) :
        this_member_(std::forward<TMember>(this_member)),
        subsequent_members_(std::forward<MemberList<TMembers...>>(subsequent_members)) {}

    // @brief Move constructor
/*    MemberList(MemberList&& other) :
        this_member_(std::move(other.this_member_)),
        subsequent_members_(std::move(other.subsequent_members_)) {}*/

    void write_json(size_t id, StreamSink* output) /*final*/ {
        this_member_.write_json(id, output);
        if (!MemberList<TMembers...>::is_empty)
            write_string(",", output);
        subsequent_members_.write_json(id + TMember::endpoint_count, output);
    }

    Endpoint* get_by_name(const char * name, size_t length) {
        Endpoint* result = this_member_.get_by_name(name, length);
        if (result) return result;
        else return subsequent_members_.get_by_name(name, length);
    }

    void register_endpoints(Endpoint** list, size_t id, size_t length) /*final*/ {
        this_member_.register_endpoints(list, id, length);
        subsequent_members_.register_endpoints(list, id + TMember::endpoint_count, length);
    }

    TMember this_member_;
    MemberList<TMembers...> subsequent_members_;
};

template<typename ... TMembers>
MemberList<TMembers...> make_protocol_member_list(TMembers&&... member_list) {
    return MemberList<TMembers...>(std::forward<TMembers>(member_list)...);
}

template<typename ... TMembers>
class ProtocolObject {
public:
    ProtocolObject(const char * name, TMembers&&... member_list) :
        name_(name),
        member_list_(std::forward<TMembers>(member_list)...) {}

    static constexpr size_t endpoint_count = MemberList<TMembers...>::endpoint_count;

    void write_json(size_t id, StreamSink* output) {
        write_string("{\"name\":\"", output);
        write_string(name_, output);
        write_string("\",\"type\":\"object\",\"members\":[", output);
        member_list_.write_json(id, output),
        write_string("]}", output);
    }

    Endpoint* get_by_name(const char * name, size_t length) {
        size_t segment_length = strlen(name);
        if (!strncmp(name, name_, length))
            return member_list_.get_by_name(name + segment_length + 1, length - segment_length - 1);
        else
            return nullptr;
    }

    void register_endpoints(Endpoint** list, size_t id, size_t length) {
        member_list_.register_endpoints(list, id, length);
    }
    
    const char * name_;
    MemberList<TMembers...> member_list_;
};

template<typename ... TMembers>
ProtocolObject<TMembers...> make_protocol_object(const char * name, TMembers&&... member_list) {
    return ProtocolObject<TMembers...>(name, std::forward<TMembers>(member_list)...);
}


// TODO: move to cpp_utils
#define ENABLE_IF_SAME(a, b, type) \
    template<typename T = a> typename std::enable_if_t<std::is_same<T, b>::value, bool>

template<typename TProperty>
class ProtocolProperty : public Endpoint {
public:
    static constexpr const char * json_modifier = get_default_json_modifier<TProperty>();
    static constexpr size_t endpoint_count = 1;

    ProtocolProperty(const char * name, TProperty* property)
        : name_(name), property_(property)
    {}

/*  TODO: find out why the move constructor is not used when it could be
    ProtocolProperty(const ProtocolProperty&) = delete;
    // @brief Move constructor
    ProtocolProperty(ProtocolProperty&& other) :
        Endpoint(std::move(other)),
        name_(std::move(other.name_)),
        property_(other.property_)
    {}
    constexpr ProtocolProperty& operator=(const ProtocolProperty& other) = delete;
    constexpr ProtocolProperty& operator=(const ProtocolProperty& other) {
        //Endpoint(std::move(other)),
        //name_(std::move(other.name_)),
        //property_(other.property_)
        name_ = other.name_;
        property_ = other.property_;
        return *this;
    }
    ProtocolProperty& operator=(ProtocolProperty&& other)
        : name_(other.name_), property_(other.property_)
    {}
    ProtocolProperty& operator=(const ProtocolProperty& other)
        : name_(other.name_), property_(other.property_)
    {}*/

    void write_json(size_t id, StreamSink* output) {
        // write name
        write_string("{\"name\":\"", output);
        LOG_PROTO("json: this at %x, name at %x is s\r\n", (uintptr_t)this, (uintptr_t)name_);
        //LOG_PROTO("json\r\n");
        write_string(name_, output);

        // write endpoint ID
        write_string("\",\"id\":", output);
        char id_buf[10];
        snprintf(id_buf, sizeof(id_buf), "%u", id); // TODO: get rid of printf
        write_string(id_buf, output);

        // write additional JSON data
        if (json_modifier && json_modifier[0]) {
            write_string(",", output);
            write_string(json_modifier, output);
        }

        write_string("}", output);
    }

    Endpoint* get_by_name(const char * name, size_t length) {
        if (!strncmp(name, name_, length))
            return this;
        else
            return nullptr;
    }


    // *** ASCII protocol handlers ***

    ENABLE_IF_SAME(std::decay_t<TProperty>, float, bool)
    get_string_ex(char * buffer, size_t length, int) {
        snprintf(buffer, length, "%f", *property_);
        return true;
    }
    ENABLE_IF_SAME(std::decay_t<TProperty>, int32_t, bool)
    get_string_ex(char * buffer, size_t length, int) {
        snprintf(buffer, length, "%ld", *property_);
        return true;
    }
    ENABLE_IF_SAME(std::decay_t<TProperty>, uint32_t, bool)
    get_string_ex(char * buffer, size_t length, int) {
        snprintf(buffer, length, "%lu", *property_);
        return true;
    }
    ENABLE_IF_SAME(std::decay_t<TProperty>, bool, bool)
    get_string_ex(char * buffer, size_t length, int) {
        buffer[0] = (*property_) ? '1' : '0';
        buffer[1] = 0;
        return true;
    }
    bool get_string_ex(char * buffer, size_t length, ...) {
        return false;
    }
    bool get_string(char * buffer, size_t length) final {
        return get_string_ex(buffer, length, 0);
    }
    ENABLE_IF_SAME(TProperty, float, bool)
    set_string_ex(char * buffer, size_t length, int) {
        return sscanf(buffer, "%f", property_) == 1;
    }
    ENABLE_IF_SAME(TProperty, int32_t, bool)
    set_string_ex(char * buffer, size_t length, int) {
        return sscanf(buffer, "%ld", property_) == 1;
    }
    ENABLE_IF_SAME(TProperty, uint32_t, bool)
    set_string_ex(char * buffer, size_t length, int) {
        return sscanf(buffer, "%lu", property_) == 1;
    }
    ENABLE_IF_SAME(TProperty, bool, bool)
    set_string_ex(char * buffer, size_t length, int) {
        int val;
        if (sscanf(buffer, "%d", &val) != 1)
            return false;
        *property_ = val;
        return true;
    }
    bool set_string_ex(char * buffer, size_t length, ...) {
        return false;
    }
    bool set_string(char * buffer, size_t length) final {
        //__asm ("bkpt");
        return set_string_ex(buffer, length, 0);
    }

    void register_endpoints(Endpoint** list, size_t id, size_t length) {
        if (id < length)
            list[id] = this;
    }
    void handle(const uint8_t* input, size_t input_length, StreamSink* output) {
        default_readwrite_endpoint_handler(property_, input, input_length, output);
    }
    /*void handle(const uint8_t* input, size_t input_length, StreamSink* output) {
        handle(input, input_length, output);
    }*/

    const char * name_;
    TProperty* property_;
};

// Non-const non-enum types
template<typename TProperty, typename = std::enable_if_t<!std::is_enum<TProperty>::value>>
ProtocolProperty<TProperty> make_protocol_property(const char * name, TProperty* property) {
    return ProtocolProperty<TProperty>(name, property);
};

// Const non-enum types
template<typename TProperty, typename = std::enable_if_t<!std::is_enum<TProperty>::value>>
ProtocolProperty<const TProperty> make_protocol_ro_property(const char * name, const TProperty* property) {
    return ProtocolProperty<const TProperty>(name, property);
};

// Non-const enum types
template<typename TProperty, typename = std::enable_if_t<std::is_enum<TProperty>::value>>
ProtocolProperty<std::underlying_type_t<TProperty>> make_protocol_property(const char * name, TProperty* property) {
    return ProtocolProperty<std::underlying_type_t<TProperty>>(name, reinterpret_cast<std::underlying_type_t<TProperty>*>(property));
};

// Const enum types
template<typename TProperty, typename = std::enable_if_t<std::is_enum<TProperty>::value>>
ProtocolProperty<const std::underlying_type_t<TProperty>> make_protocol_ro_property(const char * name, const TProperty* property) {
    return ProtocolProperty<const std::underlying_type_t<TProperty>>(name, reinterpret_cast<const std::underlying_type_t<TProperty>*>(property));
};



template<typename TObj, typename TRet, typename ... TArgs>
class FunctionTraits {
public:
    template<unsigned IUnpacked, typename ... TUnpackedArgs, typename = std::enable_if_t<IUnpacked != sizeof...(TArgs)>>
    static TRet invoke(TObj& obj, TRet(TObj::*func_ptr)(TArgs...), std::tuple<TArgs...> packed_args, TUnpackedArgs ... args) {
        return invoke<IUnpacked+1>(obj, func_ptr, packed_args, args..., std::get<IUnpacked>(packed_args));
    }

    template<unsigned IUnpacked>
    static TRet invoke(TObj& obj, TRet(TObj::*func_ptr)(TArgs...), std::tuple<TArgs...> packed_args, TArgs ... args) {
        return (obj.*func_ptr)(args...);
    }
};

/* @brief Invoke a class member function with a variable number of arguments that are supplied as a tuple

Example usage:

class MyClass {
public:
    int MyFunction(int a, int b) {
        return 0;
    }
};

MyClass my_object;
std::tuple<int, int> my_args(3, 4); // arguments are supplied as a tuple
int result = invoke_function_with_tuple(my_object, &MyClass::MyFunction, my_args);
*/
template<typename TObj, typename TRet, typename ... TArgs>
TRet invoke_function_with_tuple(TObj& obj, TRet(TObj::*func_ptr)(TArgs...), std::tuple<TArgs...> packed_args) {
    return FunctionTraits<TObj, TRet, TArgs...>::template invoke<0>(obj, func_ptr, packed_args);
}


template<typename ... TArgs>
struct PropertyListFactory;

template<>
struct PropertyListFactory<> {
    template<unsigned IPos, typename ... TAllProperties>
    static MemberList<> make_property_list(std::array<const char *, sizeof...(TAllProperties)> names, std::tuple<TAllProperties...>& values) {
        return MemberList<>();
    }
};

template<typename TProperty, typename ... TProperties>
struct PropertyListFactory<TProperty, TProperties...> {
    template<unsigned IPos, typename ... TAllProperties>
    static MemberList<ProtocolProperty<TProperty>, ProtocolProperty<TProperties>...>
    make_property_list(std::array<const char *, sizeof...(TAllProperties)> names, std::tuple<TAllProperties...>& values) {
        return MemberList<ProtocolProperty<TProperty>, ProtocolProperty<TProperties>...>(
            make_protocol_property(std::get<IPos>(names), &std::get<IPos>(values)),
            PropertyListFactory<TProperties...>::template make_property_list<IPos+1>(names, values)
        );
    }
};


template<typename TObj, typename TRet, typename ... TArgs>
class ProtocolFunction : public Endpoint {
public:
    static constexpr size_t endpoint_count = 1 + MemberList<ProtocolProperty<TArgs>...>::endpoint_count;
    template<typename ... TNames>
    ProtocolFunction(const char * name, TObj& obj, TRet(TObj::*func_ptr)(TArgs...), TNames ... names) :
        name_(name), all_arg_names_{names...}, obj_(obj), func_ptr_(func_ptr),
        input_properties_(PropertyListFactory<TArgs...>::template make_property_list<0>(all_arg_names_, in_args_))
    {
        LOG_PROTO("my tuple is at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
    }

    ProtocolFunction(const ProtocolFunction& other) :
        name_(other.name_), all_arg_names_(other.all_arg_names_), obj_(other.obj_), func_ptr_(other.func_ptr_),
        input_properties_(PropertyListFactory<TArgs...>::template make_property_list<0>(
            all_arg_names_, in_args_))
    {
        LOG_PROTO("COPIED! my tuple is at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
    }

    void write_json(size_t id, StreamSink* output) {
        // write name
        write_string("{\"name\":\"", output);
        write_string(name_, output);

        // write endpoint ID
        write_string("\",\"id\":", output);
        char id_buf[10];
        snprintf(id_buf, sizeof(id_buf), "%u", id); // TODO: get rid of printf
        write_string(id_buf, output);
        
        // write arguments
        write_string(",\"type\":\"function\",\"arguments\":[", output);
        input_properties_.write_json(id + 1, output),
        write_string("]}", output);
    }

    Endpoint* get_by_name(const char * name, size_t length) {
        return nullptr; // can't address functions by name
    }

    void register_endpoints(Endpoint** list, size_t id, size_t length) {
        if (id < length)
            list[id] = this;
        input_properties_.register_endpoints(list, id + 1, length);
    }

    void handle(const uint8_t* input, size_t input_length, StreamSink* output) {
        (void) input;
        (void) input_length;
        (void) output;
        LOG_PROTO("tuple still at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
        LOG_PROTO("invoke function using %d and %.3f\r\n", std::get<0>(in_args_), std::get<1>(in_args_));
        invoke_function_with_tuple(obj_, func_ptr_, in_args_);
    }

    const char * name_;
    std::array<const char *, sizeof...(TArgs)> all_arg_names_; // TODO: remove
    TObj& obj_;
    TRet(TObj::*func_ptr_)(TArgs...);
    std::tuple<TArgs...> in_args_;
    MemberList<ProtocolProperty<TArgs>...> input_properties_;
};

template<typename TObj, typename TRet, typename ... TArgs>
class ProtocolFunctionWithRet : Endpoint {
public:
    static constexpr size_t endpoint_count = 1 + MemberList<ProtocolProperty<TRet>>::endpoint_count + MemberList<ProtocolProperty<TArgs>...>::endpoint_count;
    template<typename ... TNames>
    ProtocolFunctionWithRet(const char * name, TObj& obj, TRet(TObj::*func_ptr)(TArgs...), TNames ... names) :
        name_(name), out_arg_names_{"out"}, all_arg_names_{names...}, obj_(obj), func_ptr_(func_ptr),
        output_properties_(PropertyListFactory<TRet>::template make_property_list<0>(out_arg_names_, out_args_)),
        input_properties_(PropertyListFactory<TArgs...>::template make_property_list<0>(all_arg_names_, in_args_))
    {
        LOG_PROTO("my tuple is at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
    }

    ProtocolFunctionWithRet(const ProtocolFunctionWithRet& other) :
        name_(other.name_), all_arg_names_(other.all_arg_names_), obj_(other.obj_), func_ptr_(other.func_ptr_),
        output_properties_(PropertyListFactory<TRet>::template make_property_list<0>(
            out_arg_names_, out_args_)),
        input_properties_(PropertyListFactory<TArgs...>::template make_property_list<0>(
            all_arg_names_, in_args_))
    {
        LOG_PROTO("COPIED! my tuple is at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
    }

    void write_json(size_t id, StreamSink* output) {
        // write name
        write_string("{\"name\":\"", output);
        write_string(name_, output);

        // write endpoint ID
        write_string("\",\"id\":", output);
        char id_buf[10];
        snprintf(id_buf, sizeof(id_buf), "%u", id); // TODO: get rid of printf
        write_string(id_buf, output);
        
        // write arguments
        write_string(",\"type\":\"function\",\"inputs\":[", output);
        input_properties_.write_json(id + 1, output),
        write_string("],\"outputs\":[", output);
        output_properties_.write_json(id + 1 + decltype(input_properties_)::endpoint_count, output),
        write_string("]}", output);
    }

    void register_endpoints(Endpoint** list, size_t id, size_t length) {
        if (id < length)
            list[id] = this;
        input_properties_.register_endpoints(list, id + 1, length);
        output_properties_.register_endpoints(list, id + 1 + decltype(input_properties_)::endpoint_count, length);
    }

    void handle(const uint8_t* input, size_t input_length, StreamSink* output) {
        (void) input;
        (void) input_length;
        (void) output;
        LOG_PROTO("tuple still at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
        LOG_PROTO("invoke function using %d and %.3f\r\n", std::get<0>(in_args_), std::get<1>(in_args_));
        std::get<0>(out_args_) = invoke_function_with_tuple(obj_, func_ptr_, in_args_);
    }

    const char * name_;
    std::array<const char *, 1> out_arg_names_; // TODO: remove
    std::array<const char *, sizeof...(TArgs)> all_arg_names_; // TODO: remove
    TObj& obj_;
    TRet(TObj::*func_ptr_)(TArgs...);
    //TRet ret_val_;
    std::tuple<TRet> out_args_;
    std::tuple<TArgs...> in_args_;
    MemberList<ProtocolProperty<TRet>> output_properties_;
    MemberList<ProtocolProperty<TArgs>...> input_properties_;
};

//template<typename TObj, typename TRet = void, typename ... TArgs, typename ... TNames, typename = std::enable_if_t<sizeof...(TArgs) == sizeof...(TNames)>>
//ProtocolFunction<TObj, TRet, TArgs...> make_protocol_function(const char * name, TObj& obj, TRet(TObj::*func_ptr)(TArgs...), TNames ... names) {
//    return ProtocolFunction<TObj, int, TArgs...>(name, obj, func_ptr, names...);
//}

template<typename TObj, typename TRet, typename ... TArgs, typename ... TNames, typename = std::enable_if_t<sizeof...(TArgs) == sizeof...(TNames)>>
ProtocolFunction<TObj, TRet, TArgs...> make_protocol_function(const char * name, TObj& obj, TRet(TObj::*func_ptr)(TArgs...), TNames ... names) {
    return ProtocolFunction<TObj, TRet, TArgs...>(name, obj, func_ptr, names...);
}

template<typename TObj, typename TRet, typename ... TArgs, typename ... TNames, typename = std::enable_if_t<sizeof...(TArgs) == sizeof...(TNames)>>
ProtocolFunctionWithRet<TObj, TRet, TArgs...> make_protocol_function_with_ret(const char * name, TObj& obj, TRet(TObj::*func_ptr)(TArgs...), TNames ... names) {
    return ProtocolFunctionWithRet<TObj, TRet, TArgs...>(name, obj, func_ptr, names...);
}



template<typename T>
class EndpointProvider_from_MemberList : public EndpointProvider {
public:
    EndpointProvider_from_MemberList(T& member_list) : member_list_(member_list) {}
    size_t get_endpoint_count() final {
        return T::endpoint_count;
    }
    void write_json(size_t id, StreamSink* output) final {
        return member_list_.write_json(id, output);
    }
    void register_endpoints(Endpoint** list, size_t id, size_t length) final {
        return member_list_.register_endpoints(list, id, length);
    }
    Endpoint* get_by_name(char * name, size_t length) final {
        for (size_t i = 0; i < length; i++) {
            if (name[i] == '.')
                name[i] = 0;
        }
        name[length-1] = 0;
        return member_list_.get_by_name(name, length);
    }
    T& member_list_;
};

void set_application_endpoints(EndpointProvider* endpoints);


// defined in communication.cpp
extern Endpoint* endpoints_[];
extern size_t n_endpoints_;
extern const size_t max_endpoints_;
extern EndpointProvider* application_endpoints;

#endif
