
/* Includes ------------------------------------------------------------------*/

#include <memory>
#include <stdlib.h>

#include <fibre/protocol.hpp>
#include <fibre/crc.hpp>

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void hexdump(const uint8_t* buf, size_t len);

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



int StreamToPacketSegmenter::process_bytes(const uint8_t *buffer, size_t length, size_t* processed_bytes) {
    int result = 0;

    while (length--) {
        if (header_index_ < sizeof(header_buffer_)) {
            // Process header byte
            header_buffer_[header_index_++] = *buffer;
            if (header_index_ == 1 && header_buffer_[0] != CANONICAL_PREFIX) {
                header_index_ = 0;
            } else if (header_index_ == 2 && (header_buffer_[1] & 0x80)) {
                header_index_ = 0; // TODO: support packets larger than 128 bytes
            } else if (header_index_ == 3 && calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, header_buffer_, 3)) {
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
            if (calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(CANONICAL_CRC16_INIT, packet_buffer_, packet_length_) == 0) {
                result |= output_.process_packet(packet_buffer_, packet_length_ - 2);
            }
            header_index_ = packet_index_ = packet_length_ = 0;
        }
        buffer++;
        if (processed_bytes)
            (*processed_bytes)++;
    }

    return result;
}

int StreamBasedPacketSink::process_packet(const uint8_t *buffer, size_t length) {
    // TODO: support buffer size >= 128
    if (length >= 128)
        return -1;

    LOG_FIBRE("send header\r\n");
    uint8_t header[] = {
        CANONICAL_PREFIX,
        static_cast<uint8_t>(length),
        0
    };
    header[2] = calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, header, 2);

    if (output_.process_bytes(header, sizeof(header), nullptr))
        return -1;
    LOG_FIBRE("send payload:\r\n");
    hexdump(buffer, length);
    if (output_.process_bytes(buffer, length, nullptr))
        return -1;

    LOG_FIBRE("send crc16\r\n");
    uint16_t crc16 = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(CANONICAL_CRC16_INIT, buffer, length);
    uint8_t crc16_buffer[] = {
        (uint8_t)((crc16 >> 8) & 0xff),
        (uint8_t)((crc16 >> 0) & 0xff)
    };
    if (output_.process_bytes(crc16_buffer, 2, nullptr))
        return -1;
    LOG_FIBRE("sent!\r\n");
    return 0;
}


// Returns part of the JSON interface definition.
bool fibre::endpoint0_handler(fibre::cbufptr_t* input_buffer, fibre::bufptr_t* output_buffer) {
    // The request must contain a 32 bit integer to specify an offset
    std::optional<uint32_t> offset = read_le<uint32_t>(input_buffer);
    
    if (!offset.has_value()) {
        // Didn't receive any offset
        return false;
    } else if (offset.value() == 0xffffffff) {
        // If the offset is special value 0xFFFFFFFF, send back the JSON version ID instead
        return write_le<uint32_t>(json_version_id_, output_buffer);
    } else if (offset.value() >= embedded_json_length) {
        // Attempt to read beyond the buffer end - return empty response
        return true;
    } else {
        // Return part of the json file
        size_t n_copy = std::min(output_buffer->size(), embedded_json_length - (size_t)offset.value());
        memcpy(output_buffer->begin(), embedded_json + offset.value(), n_copy);
        *output_buffer = output_buffer->skip(n_copy);
        return true;
    }
}

int BidirectionalPacketBasedChannel::process_packet(const uint8_t* buffer, size_t length) {
    LOG_FIBRE("got packet of length %d: \r\n", length);
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

        // Verify packet trailer. The expected trailer value depends on the selected endpoint.
        // For endpoint 0 this is just the protocol version, for all other endpoints it's a
        // CRC over the entire JSON descriptor tree (this may change in future versions).
        uint16_t expected_trailer = endpoint_id ? fibre::json_crc_ : PROTOCOL_VERSION;
        uint16_t actual_trailer = buffer[length - 2] | (buffer[length - 1] << 8);
        if (expected_trailer != actual_trailer) {
            LOG_FIBRE("trailer mismatch for endpoint %d: expected %04x, got %04x\r\n", endpoint_id, expected_trailer, actual_trailer);
            return -1;
        }
        LOG_FIBRE("trailer ok for endpoint %d\r\n", endpoint_id);

        // TODO: if more bytes than the MTU were requested, should we abort or just return as much as possible?

        uint16_t expected_response_length = read_le<uint16_t>(&buffer, &length);

        // Limit response length according to our local TX buffer size
        if (expected_response_length > sizeof(tx_buf_) - 2)
            expected_response_length = sizeof(tx_buf_) - 2;

        fibre::cbufptr_t input_buffer{buffer, length - 2};
        fibre::bufptr_t output_buffer{tx_buf_ + 2, expected_response_length};
        fibre::endpoint_handler(endpoint_id, &input_buffer, &output_buffer);

        // Send response
        if (expect_response) {
            size_t actual_response_length = expected_response_length - output_buffer.size() + 2;
            write_le<uint16_t>(seq_no | 0x8000, tx_buf_);

            LOG_FIBRE("send packet:\r\n");
            hexdump(tx_buf_, actual_response_length);
            output_.process_packet(tx_buf_, actual_response_length);
        }
    }

    return 0;
}
