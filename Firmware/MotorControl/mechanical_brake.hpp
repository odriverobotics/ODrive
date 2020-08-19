#ifndef __MECHANICAL_BRAKE_HPP
#define __MECHANICAL_BRAKE_HPP

class MechanicalBrake {
   public:
    struct Config_t {
        uint16_t gpio_num = 0;
        bool enabled = false;
        bool is_active_low = true;
        bool pulldown = true;

        // custom setters
        MechanicalBrake* parent = nullptr;
        void set_gpio_num(uint16_t value) { gpio_num = value; parent->update_config(); }
        void set_enabled(uint32_t value) { enabled = value; parent->update_config(); }
    };

    explicit MechanicalBrake(MechanicalBrake::Config_t& config);

    MechanicalBrake::Config_t& config_;
    Axis* axis_ = nullptr;

    void update_config();
    void set_enabled(bool enabled);

    void update();
    bool get_state();
    void release();
    void engage();

    bool mechanical_brake_state_ = true;

   private:
    bool pin_state_ = false;
};
#endif // __MECHANICAL_BRAKE_HPP