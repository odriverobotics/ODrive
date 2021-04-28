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

        // custom setters
        Endstop* parent = nullptr;
        void set_gpio_num(uint16_t value) { gpio_num = value; parent->update_config(); }
        void set_enabled(uint32_t value) { enabled = value; parent->update_config(); }
        void set_debounce_ms(uint32_t value) { debounce_ms = value; parent->update_config(); }
    };

    explicit Endstop(Endstop::Config_t& config);

    Endstop::Config_t& config_;
    Axis* axis_ = nullptr;

    void update_config();
    void set_enabled(bool enabled);

    void update();
    bool get_state();

    bool endstop_state_ = false;

   private:
    bool pin_state_ = false;
    float pos_when_pressed_ = 0.0f;
    Timer<float> debounceTimer_;
};
#endif