#include "hal.h"

#include <stdio.h>

void coolant_sync(uint8_t mode) {};
void spindle_sync(uint8_t state, float rpm) {};
// Initialize the limits module
void limits_init() {};
// Check for soft limit violations
void limits_soft_check(float *target) {};

// Prints system status messages.
void report_status_message(uint8_t status_code) {};

// Prints miscellaneous feedback messages.
void report_feedback_message(uint8_t message_code) {};

// Prints Grbl global settings
void report_grbl_settings() {};

void st_update_plan_block_parameters() {};



system_t sys;
int32_t sys_position[N_AXIS];   

void grbl_init() {
	memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = STATE_IDLE;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
    settings_init();
	gc_init();
	plan_reset();
}