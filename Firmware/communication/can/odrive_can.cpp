#include "odrive_can.hpp"

#include <cmsis_os.h>

#include "freertos_vars.h"
#include "utils.hpp"


bool ODriveCAN::apply_config() {
    config_.parent = this;
    set_baud_rate(config_.baud_rate);
    return true;
}

bool ODriveCAN::start_server() {
    event_loop_.init();

    auto wrapper = [](void* ctx) {
        ((ODriveCAN*)ctx)->can_server_thread();
    };
    osThreadDef(can_server_thread_def, wrapper, osPriorityNormal, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);

    return true;
}

void ODriveCAN::can_server_thread() {
    Protocol protocol = config_.protocol;

    if (protocol & PROTOCOL_SIMPLE) {
        can_simple_.init(MEMBER_CB(&event_loop_, call_later), 0, 5, 0);
    }

    start_canbus();

    event_loop_.run();

    // Event loop crashed
    for (;;) {
        osThreadSuspend(nullptr); // TODO: log error
    }
}

// Invoked on any fibre thread
bool ODriveCAN::set_baud_rate(uint32_t baud_rate) {
    if (canbus_.is_valid_baud_rate(baud_rate)) {
        if (config_.baud_rate != baud_rate) {
            config_.baud_rate = baud_rate;
            event_loop_.put(MEMBER_CB(this, restart_canbus));
        }
        return true;
    } else {
        // Probably not a compatible baudrate
        return false;
    }
}

// Invoked on the CAN thread
void ODriveCAN::start_canbus() {
    canbus_.start(config_.baud_rate, MEMBER_CB(this, on_canbus_event), MEMBER_CB(this, on_canbus_error));
}

// Invoked on the CAN thread
void ODriveCAN::restart_canbus() {
    canbus_.stop();
    start_canbus();
}

void ODriveCAN::on_canbus_event(fibre::Callback<void> callback) {
    if (!event_loop_.put(callback)) {
        // TODO: log error
    }
}

void ODriveCAN::on_canbus_error(bool intf_down) {
    if (intf_down) {
        start_canbus();
    }
}
