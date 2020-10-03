#include <odrive_main.h>

void MechanicalBrake::engage() {
	if (odrv.config_.gpio_modes[config_.gpio_num] == ODriveIntf::GPIO_MODE_MECH_BRAKE){
		get_gpio(config_.gpio_num).write(config_.is_active_low ? 0 : 1);
	}
}

void MechanicalBrake::release() {
	if (odrv.config_.gpio_modes[config_.gpio_num] == ODriveIntf::GPIO_MODE_MECH_BRAKE){
		get_gpio(config_.gpio_num).write(config_.is_active_low ? 1 : 0);
	}
}
