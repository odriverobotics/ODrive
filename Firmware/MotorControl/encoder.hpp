#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Encoder {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
        ERROR_CPR_OUT_OF_RANGE = 0x02,
        ERROR_RESPONSE = 0x04,
        ERROR_UNSUPPORTED_ENCODER_MODE = 0x08,
        ERROR_ILLEGAL_HALL_STATE = 0x10,
    };

    enum Mode_t {
        MODE_INCREMENTAL,
        MODE_HALL
    };

    struct Config_t {
        Encoder::Mode_t mode = Encoder::MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        float idx_search_speed = 10.0f; // [rad/s electrical]
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        int32_t offset = 0; // If pre_calibrated is true, this is copied into encoder.offset_ once
                            // index search succeeds
        float offset_float = 0.0f; // Sub-count phase alignment offset
        float calib_range = 0.02f;
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
                     Config_t& config);
    
    void setup();
    void set_error(Error_t error);
    bool do_checks();

    void enc_index_cb();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count);
    bool calib_enc_offset(float voltage_magnitude);
    bool scan_for_enc_idx(float omega, float voltage_magnitude);

    bool run_index_search();
    bool run_offset_calibration();
    bool update();

    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error_t error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    int32_t offset_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;    // [rad]
    float pos_estimate_ = 0.0f;  // [rad]
    float pos_cpr_ = 0.0f;  // [rad]
    float pll_vel_ = 0.0f;  // [rad/s]
    float pll_kp_ = 0.0f;   // [rad/s / rad]
    float pll_ki_ = 0.0f;   // [(rad/s^2) / rad]

    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_ro_property("is_ready", &is_ready_),
            make_protocol_ro_property("index_found", const_cast<bool*>(&index_found_)),
            make_protocol_property("shadow_count", &shadow_count_),
            make_protocol_property("count_in_cpr", &count_in_cpr_),
            make_protocol_property("offset", &offset_),
            make_protocol_property("interpolation", &interpolation_),
            make_protocol_property("phase", &phase_),
            make_protocol_property("pos_estimate", &pos_estimate_),
            make_protocol_property("pos_cpr", &pos_cpr_),
            make_protocol_property("hall_state", &hall_state_),
            make_protocol_property("pll_vel", &pll_vel_),
            make_protocol_property("pll_kp", &pll_kp_),
            make_protocol_property("pll_ki", &pll_ki_),
            make_protocol_object("config",
                make_protocol_property("mode", &config_.mode),
                make_protocol_property("use_index", &config_.use_index),
                make_protocol_property("pre_calibrated", &config_.pre_calibrated),
                make_protocol_property("idx_search_speed", &config_.idx_search_speed),
                make_protocol_property("cpr", &config_.cpr),
                make_protocol_property("offset", &config_.offset),
                make_protocol_property("offset_float", &config_.offset_float),
                make_protocol_property("calib_range", &config_.calib_range)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Encoder::Error_t)

#endif // __ENCODER_HPP
