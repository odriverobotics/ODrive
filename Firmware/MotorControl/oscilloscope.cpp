
#include "oscilloscope.hpp"

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 4096

void Oscilloscope::update() {
    float trigger_data = trigger_src_ ? *trigger_src_ : 0.0f;
    float trigger_threshold = trigger_threshold_;
    float sample_data = data_src_ ? **data_src_ : 0.0f;

    if (trigger_data < trigger_threshold) {
        ready_ = true;
    }
    if (ready_ && trigger_data >= trigger_threshold) {
        capturing_ = true;
        ready_ = false;
    }
    if (capturing_) {
        if (pos_ < OSCILLOSCOPE_SIZE) {
            data_[pos_++] = sample_data;
        } else {
            pos_ = 0;
            capturing_ = false;
        }
    }
}
