
#include "oscilloscope.hpp"

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 4096

void Oscilloscope::update() {
    // Edit these to suit your capture needs
    float trigger_data = trigger_src_ ? *trigger_src_ : 0.0f;
    float trigger_threshold = trigger_threshold_;
    float sample_data = data_src_ ? *data_src_ : 0.0f;

    static bool ready = false;
    static bool capturing = false;
    if (trigger_data < trigger_threshold) {
        ready = true;
    }
    if (ready && trigger_data >= trigger_threshold) {
        capturing = true;
        ready = false;
    }
    if (capturing) {
        data_[pos_] = sample_data;
        if (++pos_ >= OSCILLOSCOPE_SIZE) {
            pos_ = 0;
            capturing = false;
        }
    }
}
