#include <odrive_main.h>


void Endstop::update() {
    debounceTimer_.update();
    last_state_ = endstop_state_;
    if (config_.enabled) {
        bool last_pin_state = pin_state_;

        pin_state_ = get_gpio(config_.gpio_num).read();

        // If the pin state has changed, reset the timer
        if (pin_state_ != last_pin_state)
            debounceTimer_.reset();

        if (debounceTimer_.expired())
            endstop_state_ = config_.is_active_high ? pin_state_ : !pin_state_;  // endstop_state is the logical state
    } else {
        endstop_state_ = false;
    }
}

bool Endstop::apply_config() {
    debounceTimer_.reset();
    if (config_.enabled) {
        debounceTimer_.start();
    } else {
        debounceTimer_.stop();
    }
    debounceTimer_.setIncrement(config_.debounce_ms * 0.001f);
    return true;
}
