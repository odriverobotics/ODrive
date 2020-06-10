#ifndef __INTERFACE_CAN_HPP
#define __INTERFACE_CAN_HPP

#include <cmsis_os.h>
#include <stm32f4xx_hal.h>
#include "fibre/protocol.hpp"
#include "odrive_main.h"
#include "can_helpers.hpp"

#define CAN_CLK_HZ (42000000)
#define CAN_CLK_MHZ (42)

// Anonymous enum for defining the most common CAN baud rates
enum {
    CAN_BAUD_125K   = 125000,
    CAN_BAUD_250K   = 250000,
    CAN_BAUD_500K   = 500000,
    CAN_BAUD_1000K  = 1000000,
    CAN_BAUD_1M     = 1000000
};

class ODriveCAN : public ODriveIntf::CanIntf {
   public:
    struct Config_t {
        uint32_t baud_rate = CAN_BAUD_250K;
        Protocol protocol = PROTOCOL_SIMPLE;
    };

    ODriveCAN(ODriveCAN::Config_t &config, CAN_HandleTypeDef *handle);

    // Thread Relevant Data
    osThreadId thread_id_;
    const uint32_t stack_size_ = 1024; // Bytes
    Error error_ = ERROR_NONE;

    volatile bool thread_id_valid_ = false;
    bool start_can_server();
    void can_server_thread();
    void send_heartbeat(Axis *axis);
    void reinit_can();

    void set_error(Error error);

    // I/O Functions
    uint32_t available();
    uint32_t write(can_Message_t &txmsg);
    bool read(can_Message_t &rxmsg);

    ODriveCAN::Config_t &config_;

private:
    CAN_HandleTypeDef *handle_ = nullptr;

    void set_baud_rate(uint32_t baudRate);
};

#endif  // __INTERFACE_CAN_HPP
