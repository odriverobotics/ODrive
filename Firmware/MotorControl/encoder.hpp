#ifndef __ENCODER_HPP
#define __ENCODER_HPP

struct EncoderConfig_t {
    bool use_index = false;
    bool calibrated = false;
    float idx_search_speed = 10.0f; // [rad/s electrical]
    int32_t cpr = (2048 * 4); // Default resolution of CUI-AMT102 encoder,
    int32_t offset = 0;
    float calib_range = 0.02;
};

class Encoder {
public:
    Encoder(const EncoderHardwareConfig_t& hw_config,
                     EncoderConfig_t& config);
    
    void setup();

    void enc_index_cb();

    void set_count(uint32_t count);
    bool calib_enc_offset(float voltage_magnitude);
    bool scan_for_enc_idx(float omega, float voltage_magnitude);

    bool update(float* pos_estimate, float* vel_estimate, float* phase);
    bool run_calibration();

    const EncoderHardwareConfig_t& hw_config;
    EncoderConfig_t& config;
    Axis* axis = nullptr; // set by Axis constructor

    volatile bool index_found = false;
    int32_t state = 0;
    float phase = 0.0f;    // [rad]
    float pll_pos = 0.0f;  // [rad]
    float pll_vel = 0.0f;  // [rad/s]
    float pll_kp = 0.0f;   // [rad/s / rad]
    float pll_ki = 0.0f;   // [(rad/s^2) / rad]
};

#endif // __ENCODER_HPP
