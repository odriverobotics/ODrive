#include "grbl.h"

void protocol_execute_realtime() {};

// Executes the auto cycle feature, if enabled.
void protocol_auto_cycle_start() {
  consume_output();
};

// Block until all buffered steps are executed
void protocol_buffer_synchronize() {
  while (plan_get_current_block())
    consume_output();
};