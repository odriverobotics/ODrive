#ifndef __ENDSTOP_HPP
#define __ENDSTOP_HPP

#include "timer.hpp"
class Endstop {
   public:
    struct Config_t {
        float offset = 0;
        uint32_t debounce_ms = 50;
        uint16_t gpio_num = 0;
        bool enabled = false;
        bool is_active_high = false;
        bool pullup = true;
    };

    explicit Endstop(Endstop::Config_t& config);

    Endstop::Config_t& config_;
    Axis* axis_ = nullptr;

    void update_config();
    void set_enabled(bool enabled);

    void update();
    bool get_state();

    bool endstop_state_ = false;

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_ro_property("endstop_state", &endstop_state_),
            make_protocol_object("config",
                make_protocol_property("gpio_num", &config_.gpio_num,
                                    [](void* ctx) { static_cast<Endstop*>(ctx)->update_config(); }, this),
                make_protocol_property("enabled", &config_.enabled,
                                    [](void* ctx) { static_cast<Endstop*>(ctx)->update_config(); }, this),
                make_protocol_property("offset", &config_.offset),
                make_protocol_property("is_active_high", &config_.is_active_high),
                make_protocol_property("pullup", &config_.pullup),
                make_protocol_property("debounce_ms", &config_.debounce_ms,
                                    [](void* ctx) { static_cast<Endstop*>(ctx)->update_config(); }, this)));
    }

   private:
    bool pin_state_ = false;
    float pos_when_pressed_ = 0.0f;
    Timer<float> debounceTimer_;
};
#endif