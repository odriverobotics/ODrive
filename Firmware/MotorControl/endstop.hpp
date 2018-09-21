#ifndef __ENDSTOP_HPP
#define __ENDSTOP_HPP


struct EndstopConfig_t {
    uint16_t gpio_num;
    bool enabled = false;
    int32_t offset = 0;
    bool is_active_high = false;
    float debounce_ms = 100.0f;
};

class Endstop {
   public:
    Endstop(EndstopConfig_t& config);
    
    EndstopConfig_t& config_;
    Axis* axis_ = nullptr;

    void set_endstop_enabled(bool enable);
    void endstop_cb();
    void update();

    bool getEndstopState();

    auto make_protocol_definitions(){
        return make_protocol_member_list(
            make_protocol_object("config",
                                 make_protocol_property("gpio_num", &config_.gpio_num),
                                 make_protocol_property("enabled", &config_.enabled),
                                 make_protocol_property("offset", &config_.offset),
                                 make_protocol_property("is_active_high", &config_.is_active_high),
                                 make_protocol_property("debounce_ms", &config_.debounce_ms)
                                )
        );
    }

   private:
    bool endstop_state_ = false;
    bool pin_state_ = false;
    float debounce_timer_ = 0;
};
#endif