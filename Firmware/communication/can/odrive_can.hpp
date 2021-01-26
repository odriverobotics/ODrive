#ifndef __ODRIVE_CAN_HPP
#define __ODRIVE_CAN_HPP

//#include <cmsis_os.h>

#include <interfaces/canbus.hpp>
#include "can_simple.hpp"
#include <autogen/interfaces.hpp>
#include "cmsis_event_loop.hpp"

class ODriveCAN : public ODriveIntf::CanIntf {
public:
    struct Config_t {
        uint32_t baud_rate = 250000;
        Protocol protocol = PROTOCOL_SIMPLE;

        ODriveCAN* parent = nullptr; // set in apply_config()
        void set_baud_rate(uint32_t value) { parent->set_baud_rate(baud_rate); }
    };

    ODriveCAN(CanBusBase& canbus) : can_simple_{&canbus}, canbus_{canbus} {}

    bool apply_config();
    bool start_server();

    Error error_ = ERROR_NONE;

    Config_t config_;
    CANSimple can_simple_;
    osThreadId thread_id_;

    const uint32_t stack_size_ = 1024;  // Bytes

private:
    static const uint8_t kCanFifoNone = 0xff;

    void can_server_thread();
    bool set_baud_rate(uint32_t baud_rate);
    void start_canbus();
    void restart_canbus();
    void on_canbus_event(fibre::Callback<void> callback);
    void on_canbus_error(bool intf_down);

    CanBusBase& canbus_;
    CmsisEventLoop event_loop_;
};

#endif  // __ODRIVE_CAN_HPP
