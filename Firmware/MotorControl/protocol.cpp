
/* Includes ------------------------------------------------------------------*/

#include "low_level.h"
#include "protocol.hpp"

#include <memory>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
//#define DEGUG_PROTOCOL
/* Private macros ------------------------------------------------------------*/

#ifdef DEGUG_PROTOCOL
#define LOG_PROTO(...)  do { printf(__VA_ARGS__); osDelay(10); } while (0)
#else
#define LOG_PROTO(...)  ((void) 0)
#endif

/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void hexdump(const uint8_t* buf, size_t len);
static inline int write_string(const char* str, StreamSink* output);

/* Function implementations --------------------------------------------------*/

#if 0
void hexdump(const uint8_t* buf, size_t len) {
    for (size_t pos = 0; pos < len; ++pos) {
        printf(" %02x", buf[pos]);
        if ((((pos + 1) % 16) == 0) || ((pos + 1) == len))
            printf("\r\n");
        osDelay(2);
    }
}
#else
void hexdump(const uint8_t* buf, size_t len) {
    (void) buf;
    (void) len;
}
#endif

static inline int write_string(const char* str, StreamSink* output) {
    return output->process_bytes(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

void Endpoint::write_json(size_t id, bool* need_comma, StreamSink* output) const {
    if (type_ == CLOSE_TREE) {
        write_string("]}", output);
        *need_comma = true;
    } else {
        if (*need_comma)
            write_string(",", output);

        // write name
        write_string("{\"name\":\"", output);
        if (name_)
            write_string(name_, output);

        // write endpoint ID
        write_string("\",\"id\":", output);
        char id_buf[10];
        snprintf(id_buf, sizeof(id_buf), "%u", id); // TODO: get rid of printf
        write_string(id_buf, output);

        // write additional JSON data
        if (json_modifier_ && json_modifier_[0]) {
            write_string(",", output);
            write_string(json_modifier_, output);
        }

        if (type_ == BEGIN_OBJECT) {
            write_string(",\"content\":[", output);
            *need_comma = false;
        } else if (type_ == BEGIN_FUNCTION) {
            write_string(",\"arguments\":[", output);
            *need_comma = false;
        } else if (type_ == PROPERTY) {
            write_string("}", output);
            *need_comma = true;
        }
    }
}



int StreamToPacketConverter::process_bytes(const uint8_t *buffer, size_t length) {
    int result = 0;

    while (length--) {
        if (header_index_ < sizeof(header_buffer_)) {
            // Process header byte
            header_buffer_[header_index_++] = *buffer;
            if (header_index_ == 1 && header_buffer_[0] != SYNC_BYTE) {
                header_index_ = 0;
            } else if (header_index_ == 2 && (header_buffer_[1] & 0x80)) {
                header_index_ = 0; // TODO: support packets larger than 128 bytes
            } else if (header_index_ == 3 && calc_crc8(CRC8_INIT, header_buffer_, 3)) {
                header_index_ = 0;
            } else if (header_index_ == 3) {
                packet_length_ = header_buffer_[1] + 2;
            }
        } else if (packet_index_ < sizeof(packet_buffer_)) {
            // Process payload byte
            packet_buffer_[packet_index_++] = *buffer;
        }

        // If both header and packet are fully received, hand it on to the packet processor
        if (header_index_ == 3 && packet_index_ == packet_length_) {
            if (calc_crc16(CRC16_INIT, packet_buffer_, packet_length_) == 0) {
                result |= output_.process_packet(packet_buffer_, packet_length_ - 2);
            }
            header_index_ = packet_index_ = packet_length_ = 0;
        }
        buffer++;
    }

    return result;
}

int PacketToStreamConverter::process_packet(const uint8_t *buffer, size_t length) {
    // TODO: support buffer size >= 128
    if (length >= 128)
        return -1;

    LOG_PROTO("send header\r\n");
    uint8_t header[] = {
        SYNC_BYTE,
        static_cast<uint8_t>(length),
        0
    };
    header[2] = calc_crc8(CRC8_INIT, header, 2);

    if (output_.process_bytes(header, sizeof(header)))
        return -1;
    LOG_PROTO("send payload:\r\n");
    hexdump(buffer, length);
    if (output_.process_bytes(buffer, length))
        return -1;

    LOG_PROTO("send crc16\r\n");
    uint16_t crc16 = calc_crc16(CRC16_INIT, buffer, length);
    uint8_t crc16_buffer[] = {
        (uint8_t)((crc16 >> 8) & 0xff),
        (uint8_t)((crc16 >> 0) & 0xff)
    };
    if (output_.process_bytes(crc16_buffer, 2))
        return -1;
    LOG_PROTO("sent!\r\n");
    return 0;
}


// Calculates the CRC16 of the JSON interface descriptor.
// The init value is the protocol version.
uint16_t BidirectionalPacketBasedChannel::calculate_json_crc16(void) {
    CRC16Calculator crc16_calculator(PROTOCOL_VERSION);

    uint8_t offset[4] = { 0 };
    interface_query(offset, sizeof(offset), &crc16_calculator);

    return crc16_calculator.get_crc16();
}

// Returns part of the JSON interface definition.
void BidirectionalPacketBasedChannel::interface_query(const uint8_t* input, size_t input_length, StreamSink* output) {
    // The request must contain a 32 bit integer to specify an offset
    if (input_length < 4)
        return;
    uint32_t offset = 0;
    read_le<uint32_t>(&offset, input);
    NullStreamSink output_with_offset = NullStreamSink(offset, *output);
    
    bool need_comma = false;
    write_string("[", &output_with_offset);
    for (size_t i = 0; i < n_endpoints_; ++i) {
        get_endpoint(i)->write_json(i, &need_comma, &output_with_offset);
        if (!output->get_free_space())
            return; // return early if the output cannot take more bytes
    }
    write_string("]", &output_with_offset);
}

//Oskar: Can you please make a google sheet which describes the packet layout/format
int BidirectionalPacketBasedChannel::process_packet(const uint8_t* buffer, size_t length) {
    LOG_PROTO("got packet of length %d: \r\n", length);
    hexdump(buffer, length);
    if (length < 4)
        return -1;

    uint16_t seq_no = read_le<uint16_t>(&buffer, &length);

    if (seq_no & 0x8000) {
        // TODO: ack handling
    } else {
        // TODO: think about some kind of ordering guarantees
        // currently the seq_no is just used to associate a response with a request

        uint16_t endpoint_id = read_le<uint16_t>(&buffer, &length);
        bool expect_response = endpoint_id & 0x8000;
        endpoint_id &= 0x7fff;

        const Endpoint* endpoint = get_endpoint(endpoint_id);
        if (!endpoint)
            return -1;

        // Verify packet footer. The expected footer value depends on the selected endpoint.
        // For endpoint 0 this is just the protocol version, for all other endpoints it's a
        // CRC over the entire JSON descriptor tree (this may change in future versions).
        uint16_t expected_footer = endpoint_id ? json_crc_ : PROTOCOL_VERSION;
        uint16_t actual_footer = buffer[length - 2] | (buffer[length - 1] << 8);
        if (expected_footer != actual_footer) {
            LOG_PROTO("footer mismatch for endpoint %d: expected %04x, got %04x\r\n", endpoint_id, expected_footer, actual_footer);
            return -1;
        }
        LOG_PROTO("footer ok\r\n");

        // TODO: if more bytes than the MTU were requested, should we abort or just return as much as possible?

        uint16_t expected_response_length = read_le<uint16_t>(&buffer, &length);

        // Limit response length according to our local TX buffer size
        if (expected_response_length > sizeof(tx_buf_) - 2)
            expected_response_length = sizeof(tx_buf_) - 2;

        MemoryStreamSink output(tx_buf_ + 2, expected_response_length);
        endpoint->handle(buffer, length - 2, &output);

        // Send response
        if (expect_response) {
            size_t actual_response_length = expected_response_length - output.get_free_space() + 2;
            write_le<uint16_t>(seq_no | 0x8000, tx_buf_);

            LOG_PROTO("send packet:\r\n");
            hexdump(tx_buf_, actual_response_length);
            output_.process_packet(tx_buf_, actual_response_length);
        }
    }

    return 0;
}
