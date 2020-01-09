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

enum CAN_Protocol_t {
    CAN_PROTOCOL_SIMPLE
};

class ODriveCAN {
   public:
    struct Config_t {
        uint32_t baud = CAN_BAUD_250K;
        CAN_Protocol_t protocol = CAN_PROTOCOL_SIMPLE;
    };

    enum Error_t {
        ERROR_NONE = 0x00,
        ERROR_DUPLICATE_CAN_IDS = 0x01
    };

    ODriveCAN(CAN_HandleTypeDef *handle, ODriveCAN::Config_t &config);

    // Thread Relevant Data
    osThreadId thread_id_;
    const uint32_t stack_size_ = 1024; // Bytes
    Error_t error_ = ERROR_NONE;

    volatile bool thread_id_valid_ = false;
    bool start_can_server();
    void can_server_thread();
    void send_heartbeat(Axis *axis);
    void reinit_can();

    void set_error(Error_t error);

    // I/O Functions
    uint32_t available();
    uint32_t write(can_Message_t &txmsg);
    bool read(can_Message_t &rxmsg);

    // Communication Protocol Handling
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_object("config",
                                 make_protocol_ro_property("baud_rate", &config_.baud)),
            make_protocol_property("can_protocol", &config_.protocol),
            make_protocol_function("set_baud_rate", *this, &ODriveCAN::set_baud_rate, "baudRate"));
    }

   private:
    CAN_HandleTypeDef *handle_ = nullptr;
    ODriveCAN::Config_t &config_;

    void set_baud_rate(uint32_t baudRate);
};


DEFINE_ENUM_FLAG_OPERATORS(ODriveCAN::Error_t)

#endif  // __INTERFACE_CAN_HPP
