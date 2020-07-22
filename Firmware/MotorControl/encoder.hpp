#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Encoder {
public:
    typedef enum {
        AS_FLAG_PARITY          = 0x8000,
        AS_FLAG_READ            = 0x4000,
    } As5047Flag;
    typedef enum {
        AS_CMD_NOP              = 0x0000,
        AS_CMD_ERROR            = 0x0001 | AS_FLAG_READ,   // Reads error register of sensor and clear error flags
        AS_CMD_DIAGNOSTICS      = 0x3FFD | AS_FLAG_READ,   // Reads automatic gain control and diagnostics info
        AS_CMD_MAGNITUDE        = 0x3FFE | AS_FLAG_READ,
        AS_CMD_ANGLE            = 0x3FFF | AS_FLAG_PARITY | AS_FLAG_READ,
    } As5047Command;
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
        ERROR_CPR_POLEPAIRS_MISMATCH = 0x02,
        ERROR_NO_RESPONSE = 0x04,
        ERROR_UNSUPPORTED_ENCODER_MODE = 0x08,
        ERROR_ILLEGAL_HALL_STATE = 0x10,
        ERROR_INDEX_NOT_FOUND_YET = 0x20,
        ERROR_ABS_SPI_TIMEOUT = 0x40,
        ERROR_ABS_SPI_COM_FAIL = 0x80,
        ERROR_ABS_SPI_NOT_READY = 0x100,
        ERROR_ABS_SPI_ERROR_BIT = 0x120,
    };

    enum Mode_t {
        MODE_INCREMENTAL,
        MODE_HALL,
        MODE_SINCOS,
        MODE_SPI_ABS_CUI = 0x100,   //!< compatible with CUI AMT23xx
        MODE_SPI_ABS_AMS = 0x101,   //!< compatible with AMS AS5047P, AS5048A/AS5048B (no daisy chain support)
        MODE_SPI_ABS_AEAT = 0x102,  //!< not yet implemented
    };
    const uint32_t MODE_FLAG_ABS = 0x100;

    struct Config_t {
        Encoder::Mode_t mode = Encoder::MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        int32_t offset = 0;        // Offset between encoder count and rotor electrical phase
        float offset_float = 0.0f; // Sub-count phase alignment offset
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool idx_search_unidirectional = false; // Only allow index search in known direction
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint16_t abs_spi_cs_gpio_pin = 1;
        uint16_t sincos_gpio_pin_sin = 3;
        uint16_t sincos_gpio_pin_cos = 4;
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
            Config_t& config, const Motor::Config_t& motor_config);
    
    void setup();
    void set_error(Error_t error);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    bool run_index_search();
    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool update();

    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor
    
    
    Error_t error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;    // [count]
    float pos_estimate_ = 0.0f;  // [count]
    float pos_cpr_ = 0.0f;  // [count]
    float vel_estimate_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    bool abs_spi_init();
    bool abs_spi_start_transaction();
    void abs_spi_cb();
    void abs_spi_cs_pin_init();
    uint16_t abs_spi_dma_tx_[1] = {AS_CMD_ANGLE}; //read and reset error code AS_CMD_ERROR
    uint16_t abs_spi_dma_rx_[1];
    bool abs_spi_pos_updated_ = false;
    Mode_t mode_ = MODE_INCREMENTAL;
    GPIO_TypeDef* abs_spi_cs_port_;
    uint16_t abs_spi_cs_pin_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;
    bool mWorkFirstTime_ = true;
    uint8_t mWorkErrorSPI_ = 0;
    uint16_t errorCodeFromAS_ = 0x0000;
    constexpr float getCoggingRatio(){
        return config_.cpr / 3600.0f;
    }

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_ro_property("is_ready", &is_ready_),
            make_protocol_ro_property("index_found", const_cast<bool*>(&index_found_)),
            make_protocol_ro_property("shadow_count", &shadow_count_),
            make_protocol_ro_property("count_in_cpr", &count_in_cpr_),
            make_protocol_ro_property("interpolation", &interpolation_),
            make_protocol_ro_property("phase", &phase_),
            make_protocol_ro_property("pos_estimate", &pos_estimate_),
            make_protocol_ro_property("pos_cpr", &pos_cpr_),
            make_protocol_ro_property("hall_state", &hall_state_),
            make_protocol_ro_property("vel_estimate", &vel_estimate_),
            make_protocol_ro_property("calib_scan_response", &calib_scan_response_),
            make_protocol_property("pos_abs", &pos_abs_),
            make_protocol_ro_property("spi_error_rate", &spi_error_rate_),
            make_protocol_ro_property("mWorkErrorSPI", &mWorkErrorSPI_),
            make_protocol_property("mWorkFirstTime", &mWorkFirstTime_),
            make_protocol_property("errorCodeFromAS", &errorCodeFromAS_),
            make_protocol_object("config",
                make_protocol_property("mode", &config_.mode),
                make_protocol_property("use_index", &config_.use_index,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->set_idx_subscribe(); }, this),
                make_protocol_property("find_idx_on_lockin_only", &config_.find_idx_on_lockin_only,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->set_idx_subscribe(); }, this),
                make_protocol_property("abs_spi_cs_gpio_pin", &config_.abs_spi_cs_gpio_pin,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->abs_spi_cs_pin_init(); }, this),
                make_protocol_property("zero_count_on_find_idx", &config_.zero_count_on_find_idx),
                make_protocol_property("cpr", &config_.cpr),
                make_protocol_property("offset", &config_.offset),
                make_protocol_property("pre_calibrated", &config_.pre_calibrated,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->check_pre_calibrated(); }, this),
                make_protocol_property("offset_float", &config_.offset_float),
                make_protocol_property("enable_phase_interpolation", &config_.enable_phase_interpolation),
                make_protocol_property("bandwidth", &config_.bandwidth,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->update_pll_gains(); }, this),
                make_protocol_property("calib_range", &config_.calib_range),
                make_protocol_property("calib_scan_distance", &config_.calib_scan_distance),
                make_protocol_property("calib_scan_omega", &config_.calib_scan_omega),
                make_protocol_property("idx_search_unidirectional", &config_.idx_search_unidirectional),
                make_protocol_property("ignore_illegal_hall_state", &config_.ignore_illegal_hall_state),
                make_protocol_property("sincos_gpio_pin_sin", &config_.sincos_gpio_pin_sin),
                make_protocol_property("sincos_gpio_pin_cos", &config_.sincos_gpio_pin_cos)
            ),
            make_protocol_function("set_linear_count", *this, &Encoder::set_linear_count, "count")
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Encoder::Error_t)

#endif // __ENCODER_HPP
