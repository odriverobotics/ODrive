#ifndef __ENDSTOP_HPP
#define __ENDSTOP_HPP


struct EndstopConfig_t {
    uint16_t gpio_num;
    bool enabled = false;
    int32_t offset = 0;
    bool is_active_high = false;
};

class Endstop {
   public:
    Endstop(EndstopConfig_t& config);
    EndstopConfig_t config_;
    Axis* axis_ = nullptr;

    bool endstop_state_ = false;

    void set_endstop_enabled(bool enable);
    void endstop_cb();

    auto make_protocol_definitions(){
        return make_protocol_member_list(
            make_protocol_object("config",
                                 make_protocol_property("gpio_num", &config_.gpio_num),
                                 make_protocol_property("enabled", &config_.enabled),
                                 make_protocol_property("offset", &config_.offset),
                                 make_protocol_property("is_active_high", &config_.is_active_high)
                                )
        );
    }

   private:
    uint16_t debounce_timer_ = 0;
};
#endif