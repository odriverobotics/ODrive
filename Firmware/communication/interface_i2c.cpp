
#include "interface_i2c.h"

#include <i2c.h>

#define I2C_RX_BUFFER_SIZE 128
#define I2C_RX_BUFFER_PREAMBLE_SIZE   4
#define I2C_TX_BUFFER_SIZE 128

I2CStats_t i2c_stats_;
/*
TODO: add support back

static uint8_t i2c_rx_buffer[I2C_RX_BUFFER_PREAMBLE_SIZE + I2C_RX_BUFFER_SIZE];
static uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];

class I2CSender : public PacketSink {
public:
    int process_packet(const uint8_t* buffer, size_t length) {
        if (length >= 2 && (length - 2) <= sizeof(i2c_tx_buffer))
            memcpy(i2c_tx_buffer, buffer + 2, length - 2);
        return 0;
    }
    size_t get_free_space() { return SIZE_MAX; }
} i2c1_packet_output;
BidirectionalPacketBasedChannel i2c1_channel(i2c1_packet_output);
*/
void start_i2c_server() {
    // CAN H = SDA
    // CAN L = SCL
    //HAL_I2C_EnableListen_IT(&hi2c1);
}
/*
void i2c_handle_packet(I2C_HandleTypeDef *hi2c) {
    size_t received = sizeof(i2c_rx_buffer) - hi2c->XferCount;
    if (received > I2C_RX_BUFFER_PREAMBLE_SIZE) {
        i2c_stats_.rx_cnt++;
        
        write_le<uint16_t>(0, i2c_rx_buffer); // hallucinate seq-no (not needed for I2C)
        i2c_rx_buffer[2] = i2c_rx_buffer[4]; // endpoint-id = I2C register address
        i2c_rx_buffer[3] = i2c_rx_buffer[5] | 0x80; // MSB must be 1
        size_t expected_bytes = (TX_BUF_SIZE - 2) < I2C_TX_BUFFER_SIZE ? (TX_BUF_SIZE - 2) : I2C_TX_BUFFER_SIZE;
        write_le<uint16_t>(expected_bytes, i2c_rx_buffer + 4); // hallucinate maximum number of expected response bytes

        i2c1_channel.process_packet(i2c_rx_buffer, received);

        // reset receive buffer
        hi2c->pBuffPtr = I2C_RX_BUFFER_PREAMBLE_SIZE + i2c_rx_buffer;
        hi2c->XferCount = sizeof(i2c_rx_buffer) - I2C_RX_BUFFER_PREAMBLE_SIZE;
    }


    if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
        hi2c->State = HAL_I2C_STATE_LISTEN;
}


void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    i2c_handle_packet(hi2c);
    // restart listening for address
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    i2c_stats_.addr_match_cnt += 1;
    
    i2c_handle_packet(hi2c);

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        HAL_I2C_Slave_Sequential_Receive_IT(hi2c,
            I2C_RX_BUFFER_PREAMBLE_SIZE + i2c_rx_buffer,
            sizeof(i2c_rx_buffer) - I2C_RX_BUFFER_PREAMBLE_SIZE, I2C_FIRST_AND_LAST_FRAME);
    } else {
        HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, i2c_tx_buffer, sizeof(i2c_tx_buffer), I2C_FIRST_AND_LAST_FRAME);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    // ignore NACK errors
    if (!(hi2c->ErrorCode & (~HAL_I2C_ERROR_AF)))
        return;

    i2c_stats_.error_cnt += 1;

    // Continue listening
    HAL_I2C_EnableListen_IT(hi2c);
}
*/
