
#ifndef hal_h
#define hal_h

#include "grbl.h"

// Methods for features not supported yet.
void coolant_sync(uint8_t mode);
void spindle_sync(uint8_t state, float rpm);
// Initialize the limits module
void limits_init();
// Check for soft limit violations
void limits_soft_check(float *target);




// Prints system status messages.
void report_status_message(uint8_t status_code);

// Prints miscellaneous feedback messages.
void report_feedback_message(uint8_t message_code);

// Prints Grbl global settings
void report_grbl_settings();



// Last motion block has been updated!
void st_update_plan_block_parameters();


// Motion output needs to be consumed before further processing can occur.
void consume_output();

void grbl_init();

#endif