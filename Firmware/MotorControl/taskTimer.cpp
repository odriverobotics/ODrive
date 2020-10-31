#include "taskTimer.hpp"

bool TaskTimer::sample_next = false;
volatile uint32_t adc_timestamp = 0;
