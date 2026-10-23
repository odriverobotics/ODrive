#ifndef __ENCODER_HPP
#define __ENCODER_HPP

class Encoder;

#include <board.h> // needed for arm_math.h
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include "utils.hpp"
#include <autogen/interfaces.hpp>
#include "component.hpp"


class Encoder : public ODriveIntf::EncoderIntf {
public:
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;
    static constexpr std::array<float, 6> hall_edge_defaults = 
        {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

    struct Config_t {
#ifdef ENCODER_USE_HALL
        Mode mode = MODE_HALL;
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 60.0f;//1000.0f;
        int32_t phase_offset = 45;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 1.4791250228881836; // Sub-count phase alignment offset
        int32_t cpr = 24;//(2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        float index_offset = 0.0f;
        bool use_index = false;
        bool pre_calibrated = true; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        int32_t direction = 1; // direction with respect to motor
        bool use_index_offset = true;
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint8_t hall_polarity = 0;
        bool hall_polarity_calibrated = true;
        std::array<float, 6> hall_edge_phcnt = hall_edge_defaults;
        uint16_t abs_spi_cs_gpio_pin = 7;
        uint16_t sincos_gpio_pin_sin = 1;
        uint16_t sincos_gpio_pin_cos = 2;
#endif
#ifdef ENCODER_USE_SPI_ABS_AMS
        Mode mode = MODE_SPI_ABS_AMS;
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 500.0f;//1000.0f;
        int32_t phase_offset = 0;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 0.5f; // Sub-count phase alignment offset
        int32_t cpr = 16384;//(2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        float index_offset = 0.0f;
        bool use_index = false;
        bool pre_calibrated = true; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        int32_t direction = -1; // direction with respect to motor
        bool use_index_offset = true;
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint8_t hall_polarity = 0;
        bool hall_polarity_calibrated = true;
        std::array<float, 6> hall_edge_phcnt = hall_edge_defaults;
        uint16_t abs_spi_cs_gpio_pin = 7;
        uint16_t sincos_gpio_pin_sin = 1;
        uint16_t sincos_gpio_pin_cos = 2;
#endif

        // custom setters
        Encoder* parent = nullptr;
        void set_use_index(bool value) { use_index = value; parent->set_idx_subscribe(); }
        void set_find_idx_on_lockin_only(bool value) { find_idx_on_lockin_only = value; parent->set_idx_subscribe(); }
        void set_abs_spi_cs_gpio_pin(uint16_t value) { abs_spi_cs_gpio_pin = value; parent->abs_spi_cs_pin_init(); }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(TIM_HandleTypeDef* timer, Stm32Gpio index_gpio,
            Stm32Gpio hallA_gpio, Stm32Gpio hallB_gpio, Stm32Gpio hallC_gpio,
            Stm32SpiArbiter* spi_arbiter);
    
    bool apply_config(ODriveIntf::MotorIntf::MotorType motor_type);
    void setup();
    void set_error(Error error);
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
    bool run_hall_polarity_calibration();
    bool run_hall_phase_calibration();
    bool run_offset_calibration();
    void sample_now();
    bool read_sampled_gpio(Stm32Gpio gpio);
    void decode_hall_samples();
    int32_t hall_model(float internal_pos);
    bool update();

    TIM_HandleTypeDef* timer_;
    Stm32Gpio index_gpio_;
    Stm32Gpio hallA_gpio_;
    Stm32Gpio hallB_gpio_;
    Stm32Gpio hallC_gpio_;
    Stm32SpiArbiter* spi_arbiter_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Config_t config_;

    Error error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    OutputPort<float> phase_ = 0.0f;     // [rad]
    OutputPort<float> phase_vel_ = 0.0f; // [rad/s]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s]
    OutputPort<float> pos_circular_ = 0.0f; // [turn]

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    static const constexpr GPIO_TypeDef* ports_to_sample[] = { GPIOA, GPIOB, GPIOC };
    uint16_t port_samples_[sizeof(ports_to_sample) / sizeof(ports_to_sample[0])];
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    std::optional<uint8_t> last_hall_cnt_ = std::nullopt; // Used to find hall edges for calibration
    bool calibrate_hall_phase_ = false;
    bool sample_hall_states_ = false;
    bool sample_hall_phase_ = false;
    std::array<int, 8> states_seen_count_; // for hall polarity calibration
    std::array<int, 6> hall_phase_calib_seen_count_;

    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    bool abs_spi_start_transaction();
    void abs_spi_cb(bool success);
    void abs_spi_cs_pin_init();
    bool abs_spi_pos_updated_ = false;
    Mode mode_ = MODE_INCREMENTAL;
    Stm32Gpio abs_spi_cs_gpio_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[1];
    Stm32SpiArbiter::SpiTask spi_task_;

    // ---- AS5047P 零位标定 ----
    // 实现要点：**不往 SPI 仲裁器里插任何新事务**，而是每个采样周期临时改写 spi_task_ 的
    // 命令字，搭本来就在跑的周期读的便车。这样周期读一步不落，abs_spi_pos_updated_ 不会
    // 出现长窗口为 false，也就不可能触发 ENCODER_ERROR_ABS_SPI_COM_FAIL。
    // A) 单帧流水线序列：可选地软写 ZPOSM/ZPOSL，然后读回 ZPOSM/ZPOSL/计数
    enum As5047Step : uint8_t {
        AS5047_IDLE = 0,
        AS5047_WR_M_CMD,   // 命令帧：写 ZPOSM
        AS5047_WR_M_DAT,   // 数据帧：ZPOSM = as5047_zpos_target_ >> 6
        AS5047_WR_L_CMD,   // 命令帧：写 ZPOSL
        AS5047_WR_L_DAT,   // 数据帧：ZPOSL = as5047_zpos_target_ & 0x3F
        AS5047_RD_ZM_CMD,  // 命令帧：读 ZPOSM(0x0016)
        AS5047_RD_ZM_DAT,
        AS5047_RD_ZL_CMD,  // 命令帧：读 ZPOSL(0x0017)
        AS5047_RD_ZL_DAT,
        AS5047_RD_CMD,     // 命令帧：读 ANGLECOM(0x3FFF) —— 周期读用的就是这个寄存器
        AS5047_RD_DAT,
        AS5047_RD_MAG_CMD, // 命令帧：读 MAG(0x3FFD) 磁场强度（CORDIC 幅值）
        AS5047_RD_MAG_DAT,
        AS5047_RD_DIAG_CMD,// 命令帧：读 DIAAGC(0x3FFC) 诊断与 AGC
        AS5047_RD_DIAG_DAT,
        AS5047_DONE,
    };
    volatile uint8_t as5047_step_ = AS5047_IDLE;
    volatile uint16_t as5047_zpos_target_ = 0;     // 要写入的 ZPOS
    volatile uint16_t as5047_zposm_rb_ = 0xffffu;  // ZPOSM 读回值（应 = zpos_target >> 6）
    volatile uint16_t as5047_zposl_rb_ = 0xffffu;  // ZPOSL 读回值（应 = zpos_target & 0x3F）
    volatile uint16_t as5047_rd_count_ = 0xffffu;  // 读回的 14bit 绝对计数
    volatile uint16_t as5047_mag_ = 0xffffu;       // MAG(0x3FFD) 读回值
    volatile uint16_t as5047_diag_ = 0xffffu;      // DIAAGC(0x3FFC) 读回值
    uint16_t as5047_seq_tx_ = 0;
    uint16_t as5047_seq_rx_ = 0;
    bool as5047_seq_prepare();
    void as5047_seq_advance();

    // B) OTP 烧写序列（不可逆！）严格按手册 Figure 35-38 的 19 步流程展开。
    // 每个采样周期一帧，整条序列约 40 帧 + 轮询，期间 abs_spi_pos_updated_ 不置位，
    // 所以必须 disarm 后执行。
    enum As5047OtpStep : uint8_t {
        AS5047_OTP_OFF = 0,
        AS5047_OTP_PRE_CMD,      // 烧前预检：读 0x0016/0x0017，非 0 说明 OTP 已经烧过
        AS5047_OTP_PRE_DAT,
        AS5047_OTP_WR_CMD,       // idx 0..3：写 0x0016+idx
        AS5047_OTP_WR_DAT,
        AS5047_OTP_RD1_CMD,      // idx 0..3：读回 0x0016+idx（step 6）
        AS5047_OTP_RD1_DAT,
        AS5047_OTP_PROGEN_CMD,   // PROG = 0x0001（step 8）
        AS5047_OTP_PROGEN_DAT,
        AS5047_OTP_BURN_CMD,     // PROG = 0x0008（step 9）
        AS5047_OTP_BURN_DAT,
        AS5047_OTP_POLL_CMD,     // 读 PROG 直到 0x0001（step 10）
        AS5047_OTP_POLL_DAT,
        AS5047_OTP_CLR_CMD,      // idx 0..3：清零易失内容（step 11）
        AS5047_OTP_CLR_DAT,
        AS5047_OTP_PROGVER_CMD,  // PROG = 0x0040（step 12）
        AS5047_OTP_PROGVER_DAT,
        AS5047_OTP_OTPREF_CMD,   // PROG = 0x0004（step 13）
        AS5047_OTP_OTPREF_DAT,
        AS5047_OTP_RD2_CMD,      // idx 0..3：读回 0x0016+idx（step 14）
        AS5047_OTP_RD2_DAT,
        AS5047_OTP_DONE,
        AS5047_OTP_FAILED,
    };
    volatile uint8_t as5047_otp_step_ = AS5047_OTP_OFF;
    volatile uint8_t as5047_otp_idx_ = 0;
    volatile uint32_t as5047_otp_polls_ = 0;
    volatile uint8_t as5047_otp_fail_ = 0;
    uint32_t as5047_otp_deadline_ = 0;  // 墙上时钟超时（毫秒）
    uint16_t as5047_otp_expected_[4] = {0, 0, 0, 0}; // 期望值：ZPOSM / ZPOSL / SETTINGS1 / SETTINGS2
    uint16_t as5047_otp_rd1_[4] = {0, 0, 0, 0};
    uint16_t as5047_otp_rd2_[4] = {0, 0, 0, 0};
    bool as5047_otp_prepare();
    void as5047_otp_advance();
    uint16_t as5047_otp_write_value(uint8_t idx) const;
    bool as5047_otp_verify(const uint16_t* rd) const;

    constexpr float getCoggingRatio(){
        return 1.0f / 3600.0f;
    }

    // AS5047P 零位标定：只能在 CMSIS 线程上下文调用；内部只轮询状态机（带超时），
    // 不做阻塞式 SPI。
    // 读回当前的 ZPOSM/ZPOSL（组合成 14bit zpos）和 14bit 绝对计数。**不动硬件任何状态**，
    // 上电后第一次调用读到的 ZPOS 就是 OTP 里的内容（未烧过则为 0）。
    bool as5047_read_zero_and_count(uint16_t* zpos, uint16_t* count, uint32_t timeout_ms = 50);
    // 软写 AS5047P 的 ZPOSM/ZPOSL = zpos（**易失寄存器，掉电丢失**），随后读回 ZPOSM 和计数。
    bool as5047_write_zpos(uint16_t zpos, uint16_t* count, uint16_t* zposm_rb,
                           uint32_t timeout_ms = 50);
    // ★ OTP 烧写，**不可逆，只能烧一次**。前置条件（调用方必须保证）：
    //     1) 轴已 disarm；2) 编码器供电 3.3~3.5V（5V 模式 4.5~5.5V），瞬时 100mA；3) 未软写过 ZPOS。
    //   拆成 start/poll 两半是刻意的：整条序列几十毫秒到 1 秒，不能阻塞 CAN 线程。
    bool as5047_burn_otp_start(uint16_t zpos, uint32_t timeout_ms = 3000);
    // 返回 true 表示已经结束（成功/失败/超时），out_zpos/fail_code 有效；false = 仍在进行
    bool as5047_burn_otp_poll(uint16_t* out_zpos, uint8_t* fail_code);
    // 当前 14bit 绝对计数（MODE_SPI_ABS_AMS 下就是 ANGLECOM 的原始值，已含 ZPOS）
    uint16_t as5047_current_count() const { return (uint16_t)(pos_abs_ & 0x3fffu); }
    // 读磁场强度 MAG(0x3FFD, 14bit) 和诊断信息 DIAAGC(0x3FFC)。纯读，无副作用。
    bool as5047_read_diagnostics(uint16_t* magnitude, uint16_t* diaagc, uint32_t timeout_ms = 50);
    // 等待轴静止（用周期读得到的 pos_abs_ 判定）。零位是位置量，轴在转时标定没有意义。
    bool as5047_wait_still(uint32_t timeout_ms = 200, uint32_t tol_counts = 4);

};

#endif // __ENCODER_HPP
