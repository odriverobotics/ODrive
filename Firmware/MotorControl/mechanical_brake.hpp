#ifndef __MECHANICAL_BRAKE_HPP
#define __MECHANICAL_BRAKE_HPP

#include <autogen/interfaces.hpp>

class MechanicalBrake : public ODriveIntf::MechanicalBrakeIntf  {
   public:
    struct Config_t {
        uint16_t gpio_num = 0;
        bool enabled = false;
        bool is_active_low = true;

        // custom setters
        MechanicalBrake* parent = nullptr;
        void set_gpio_num(uint16_t value) { gpio_num = value; parent->apply_config(); }
        void set_enabled(uint32_t value) { enabled = value; parent->apply_config(); }
    };

    MechanicalBrake() {}

    MechanicalBrake::Config_t config_;
    Axis* axis_ = nullptr;

    bool apply_config();
    void set_enabled(bool enabled);
    void release();
    void engage();
};
#endif // __MECHANICAL_BRAKE_HPP