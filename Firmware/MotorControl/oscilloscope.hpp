#ifndef __OSCILLOSCOPE_HPP
#define __OSCILLOSCOPE_HPP

#include <autogen/interfaces.hpp>

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 4096

class Oscilloscope : public ODriveIntf::OscilloscopeIntf {
public:
    Oscilloscope(float* trigger_src, float trigger_threshold, float** data_src)
        : trigger_src_(trigger_src), trigger_threshold_(trigger_threshold), data_src_(data_src) {}

    float get_val(uint32_t index) override {
        return index < OSCILLOSCOPE_SIZE ? data_[index] : NAN;
    }

    void update();

    const uint32_t size_ = OSCILLOSCOPE_SIZE;
    const float* trigger_src_;
    const float trigger_threshold_;
    float* const * data_src_;

    float data_[OSCILLOSCOPE_SIZE] = {0};
    size_t pos_ = 0;
    bool ready_ = false;
    bool capturing_ = false;
};

#endif // __OSCILLOSCOPE_HPP