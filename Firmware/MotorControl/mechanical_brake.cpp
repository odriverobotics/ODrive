#include <odrive_main.h>

bool MechanicalBrake::apply_config() {
    config_.parent = this;
    set_enabled(config_.enabled);
    return true;
}

void MechanicalBrake::engage() {
    get_gpio(config_.gpio_num).write(config_.is_active_low ? 0 : 1);
}

void MechanicalBrake::release() {
    get_gpio(config_.gpio_num).write(config_.is_active_low ? 1 : 0);
}

void MechanicalBrake::set_enabled(bool enable) { 
    if (enable) {
        // We need this flag to alert the system to the configuration on boot
	    odrv.config_.gpio_modes[config_.gpio_num] = ODriveIntf::GPIO_MODE_MECH_BRAKE;
    }
}