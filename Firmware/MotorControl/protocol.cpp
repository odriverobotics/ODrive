
/* Includes ------------------------------------------------------------------*/

#include "low_level.h"
#include "protocol.hpp"

#include <memory>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// The order in this list must correspond to the order in EndpointTypeID_t
const char *type_names_[] = {
    "json",
    "int32[]",
    "float",
    "int",
    "bool",
    "uint16",
    "tree"
};

/* Private function prototypes -----------------------------------------------*/

static void write_buffer(const uint8_t* input, size_t input_length, size_t* skip, uint8_t** output, size_t* output_length);
static void write_string(const char* str, size_t* skip, uint8_t** output, size_t* output_length);

/* Function implementations --------------------------------------------------*/

// @brief Copies an input buffer to an output buffer, skipping a couple of bytes on the input buffer if required.
// @param input: input buffer
// @param input_length: number of bytes in the input buffer
// @param skip: number of bytes to skip in the input buffer - will be set to max{skip - input_length, 0}
// @param output: output buffer - will be increased by the number of bytes copied
// @param output_length: length of the output buffer - will be decreased by the number of bytes copied
void write_buffer(const uint8_t* input, size_t input_length, size_t* skip, uint8_t** output, size_t* output_length) {
    if (*skip >= input_length) {
        *skip -= input_length;
    } else {
        input_length -= *skip;
        input += *skip;
        *skip = 0;
        size_t length = input_length < *output_length ? input_length : *output_length;
        memcpy(*output, input, length);
        *output += length;
        *output_length -= length;
    }
}

static void write_string(const char* str, size_t* skip, uint8_t** output, size_t* output_length) {
    write_buffer(reinterpret_cast<const uint8_t*>(str), strlen(str), skip, output, output_length);
}

void Endpoint::write_json(size_t id, size_t* skip, uint8_t** output, size_t* output_length, bool* need_comma) {
    if (type_id_ == END_TREE) {
        write_string("]}", skip, output, output_length);
        *need_comma = true;
        return;
    } else if (type_id_ < END_TREE) {
        if (need_comma)
            write_string(",", skip, output, output_length);

        write_string("{\"name\":\"", skip, output, output_length);
        if (name_)
            write_string(name_, skip, output, output_length);
        write_string(",\"id\":", skip, output, output_length);
        char id_buf[10];
        snprintf(id_buf, sizeof(id_buf), "%u", id); // TODO: get rid of printf
        write_string(id_buf, skip, output, output_length);
        write_string("\"type\":\"", skip, output, output_length);
        if (type_names_[type_id_])
            write_string(type_names_[type_id_], skip, output, output_length);
        write_string("\"", skip, output, output_length);

        if (type_id_ == BEGIN_TREE) {
            write_string(",\"content\":[", skip, output, output_length);
            *need_comma = false;
        } else {
            if (json_modifier_ && json_modifier_[0]) {
                write_string(",", skip, output, output_length);
                write_string(json_modifier_, skip, output, output_length);
            }
            write_string("\"}", skip, output, output_length);
            *need_comma = true;
        }
    }
}



int StreamToPacketConverter::write_bytes(const uint8_t *buffer, size_t length) {
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
                packet_length_ = header_buffer_[1] + 2; // expect 2 more bytes than indicated (CRC16)
            }
        } else if (packet_index_ < sizeof(packet_buffer_)) {
            // Process payload byte
            packet_buffer_[packet_index_++] = *buffer;
        }

        // If both header and packet are fully received, hand it on to the packet processor
        if (header_index_ == 3 && packet_index_ == packet_length_) {
            result |= output_.write_packet(packet_buffer_, packet_length_ - 2);
            header_index_ = packet_index_ = packet_length_ = 0;
        }
        buffer++;
    }

    return result;
}

int PacketToStreamConverter::write_packet(const uint8_t *buffer, size_t length) {
    if (length >= 128)
        return -1;

    uint8_t header[] = {
        SYNC_BYTE,
        static_cast<uint8_t>(length),
        0
    };
    header[2] = calc_crc8(CRC8_INIT, header, 2);

    if (output_.write_bytes(header, sizeof(header)))
        return -1;
    if (output_.write_bytes(buffer, length))
        return -1;

    return 0;
}


// Calculates the CRC16 of the JSON interface descriptor.
// Make sure this stays consistent with what interface_query returns.
// The init value is the protocol version.
uint16_t BidirectionalPacketBasedChannel::calculate_json_crc16(void) {
    uint8_t buffer[64];
    size_t offset = 0;
    bool need_comma = false;

    uint8_t *buffer_ptr = buffer;
    size_t buffer_length = sizeof(buffer);
    write_string("[", &offset, &buffer_ptr, &buffer_length);
    uint16_t crc16 = calc_crc16(PROTOCOL_VERSION, buffer, sizeof(buffer) - buffer_length);

    for (size_t i = 0; i < n_endpoints_; ++i) {
        buffer_ptr = buffer;
        buffer_length = sizeof(buffer);
        get_endpoint(i)->write_json(i, &offset, &buffer_ptr, &buffer_length, &need_comma);
        crc16 = calc_crc16(crc16, buffer, sizeof(buffer) - buffer_length);
    }

    buffer_ptr = buffer;
    buffer_length = sizeof(buffer);
    write_string("]", &offset, &buffer_ptr, &buffer_length);
    crc16 = calc_crc16(crc16, buffer, sizeof(buffer) - buffer_length);

    return crc16;
}

// Returns part of the JSON interface definition.
// Make sure this stays consistent with what calculate_json_crc16 calculates.
// The init value is the protocol version.
void BidirectionalPacketBasedChannel::interface_query(const uint8_t* input, size_t input_length, uint8_t* output, size_t* output_length) {
    // The request must contain a 32 bit integer to specify an offset
    if (input_length < 4)
        return;
    uint32_t offset32 = 0;
    read_le<uint32_t>(offset32, input);
    size_t offset = offset32;
    
    bool need_comma = false;
    write_string("[", &offset, &output, output_length);
    for (size_t i = 0; i < n_endpoints_; ++i) {
        get_endpoint(i)->write_json(i, &offset, &output, output_length, &need_comma);
    }
    write_string("]", &offset, &output, output_length);
}

int BidirectionalPacketBasedChannel::write_packet(const uint8_t* buffer, size_t length) {
    if (length < 4)
        return -1;

    // calculate CRC for later validation
    uint16_t crc16 = calc_crc16(CRC16_INIT, buffer, length - 2);
    uint8_t crc16_termination[] = {
        (PROTOCOL_VERSION >> 8) & 0xff,
        (PROTOCOL_VERSION >> 0) & 0xff,
        buffer[length - 2],
        buffer[length - 1]
    };

    uint16_t seq_no = read_le<uint16_t>(&buffer, &length);

    if (seq_no & 0x8000) {
        // TODO: ack handling
    } else {
        // TODO: think about some kind of ordering guarantees
        // currently the seq_no is just used to associate a response with a request

        uint16_t endpoint_id = read_le<uint16_t>(&buffer, &length);
        bool expect_response = endpoint_id & 0x8000;
        endpoint_id &= 0x7fff;

        Endpoint* endpoint = get_endpoint(endpoint_id);
        if (!endpoint)
            return -1;

        // Verify packet CRC. The expected CRC termination value depends on the selected endpoint.
        // For endpoint 0 this is just the protocol version, for all other endpoints it's a
        // CRC over the entire JSON descriptor tree (this may change in future versions).
        if (endpoint_id) {
            crc16_termination[0] = (json_crc_ >> 8) & 0xff;
            crc16_termination[1] = (json_crc_ >> 0) & 0xff;
        }
        if (calc_crc16(crc16, crc16_termination, sizeof(crc16_termination)))
            return -1;

        // TODO: if more bytes than the MTU were requested, should we abort or just return as much as possible?
        uint16_t expected_response_length = read_le<uint16_t>(&buffer, &length);

        // Let the endpoint do the processing
        size_t requested_size = expected_response_length < (sizeof(tx_buf_) - 2) ? expected_response_length : (sizeof(tx_buf_) - 2);
        size_t remaining_size = requested_size;
        endpoint->handle(buffer, length, tx_buf_ + 2, &remaining_size);

        // Send response
        if (expect_response) {
            write_le<uint16_t>(seq_no | 0x8000, tx_buf_);
            output_.write_packet(tx_buf_, (requested_size - remaining_size) + 2);
        }
    }

    return 0;
}
