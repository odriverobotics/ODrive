
#include "odrive_main.h"
#include <Drivers/STM32/stm32_system.h>
#include <bitset>

Encoder::Encoder(TIM_HandleTypeDef* timer, Stm32Gpio index_gpio,
                 Stm32Gpio hallA_gpio, Stm32Gpio hallB_gpio, Stm32Gpio hallC_gpio,
                 Stm32SpiArbiter* spi_arbiter) :
        timer_(timer), index_gpio_(index_gpio),
        hallA_gpio_(hallA_gpio), hallB_gpio_(hallB_gpio), hallC_gpio_(hallC_gpio),
        spi_arbiter_(spi_arbiter)
{
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

bool Encoder::apply_config(ODriveIntf::MotorIntf::MotorType motor_type) {
    config_.parent = this;

    update_pll_gains();

    if (config_.pre_calibrated) {
        if (config_.mode == Encoder::MODE_HALL && config_.hall_polarity_calibrated)
            is_ready_ = true;
        if (config_.mode == Encoder::MODE_SINCOS)
            is_ready_ = true;
        if (motor_type == Motor::MOTOR_TYPE_ACIM)
            is_ready_ = true;
    }

    return true;
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
    set_idx_subscribe();

    mode_ = config_.mode;

    spi_task_.config = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = (mode_ == MODE_SPI_ABS_AEAT || mode_ == MODE_SPI_ABS_MA732) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
//        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32, //qiaoxu modify V0.5.1
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };

    if (mode_ == MODE_SPI_ABS_MA732) {
        abs_spi_dma_tx_[0] = 0x0000;
    }

    if(mode_ & MODE_FLAG_ABS){
        abs_spi_cs_pin_init();

        if (axis_->controller_.config_.anticogging.pre_calibrated) {
            axis_->controller_.anticogging_valid_ = true;
        }
    }
}

void Encoder::set_error(Error error) {
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// (maybe by attaching the interrupt on start search, synergistic with following)
void Encoder::enc_index_cb() {
    if (config_.use_index) {
        set_circular_count(0, false);
        if (config_.use_index_offset)
            set_linear_count((int32_t)(config_.index_offset * config_.cpr));
        if (config_.pre_calibrated) {
            is_ready_ = true;
            if(axis_->controller_.config_.anticogging.pre_calibrated){
                axis_->controller_.anticogging_valid_ = true;
            }
        } else {
            // We can't use the update_offset facility in set_circular_count because
            // we also set the linear count before there is a chance to update. Therefore:
            // Invalidate offset calibration that may have happened before idx search
            is_ready_ = false;
        }
        index_found_ = true;
    }

    // Disable interrupt
    index_gpio_.unsubscribe();
}

void Encoder::set_idx_subscribe(bool override_enable) {
    if (config_.use_index && (override_enable || !config_.find_idx_on_lockin_only)) {
        if (!index_gpio_.subscribe(true, false, enc_index_cb_wrapper, this)) {
            odrv.misconfigured_ = true;
        }
    } else if (!config_.use_index || config_.find_idx_on_lockin_only) {
        index_gpio_.unsubscribe();
    }
}

void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    // TODO: restoring config from python backup is fragile here (ACIM motor type must be set first)
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_ACIM) {
        if (!is_ready_)
            config_.pre_calibrated = false;
        if (mode_ == MODE_INCREMENTAL && !index_found_)
            config_.pre_calibrated = false;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;
    tim_cnt_sample_ = count;

    //Write hardware last
    timer_->Instance->CNT = count;

    cpu_exit_critical(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        config_.phase_offset += count - count_in_cpr_;
        config_.phase_offset = mod(config_.phase_offset, config_.cpr);
    }

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_counts_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

bool Encoder::run_index_search() {
    config_.use_index = true;
    index_found_ = false;
    set_idx_subscribe();

    bool success = axis_->run_lockin_spin(axis_->config_.calibration_lockin, false);
    return success;
}

bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;

    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;
    bool success = axis_->run_lockin_spin(lockin_config, false);

    if (success) {
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            config_.direction = 1;
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            config_.direction = -1;
        } else {
            config_.direction = 0;
        }
    }

    return success;
}


bool Encoder::run_hall_polarity_calibration() {
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;

    auto loop_cb = [this](bool const_vel) {
        if (const_vel)
            sample_hall_states_ = true;
        // No need to cancel early
        return true;
    };

    config_.hall_polarity_calibrated = false;
    states_seen_count_.fill(0);
    bool success = axis_->run_lockin_spin(lockin_config, false, loop_cb);
    sample_hall_states_ = false;

    if (success) {
        std::bitset<8> state_seen;
        std::bitset<8> state_confirmed;
        for (int i = 0; i < 8; i++) {
            if (states_seen_count_[i] > 0)
                state_seen[i] = true;
            if (states_seen_count_[i] > 50)
                state_confirmed[i] = true;
        }
        if (!(state_seen == state_confirmed)) {
            set_error(ERROR_ILLEGAL_HALL_STATE);
            return false;
        }

        // Hall effect sensors can be arranged at 60 or 120 electrical degrees.
        // Out of 8 possible states, 120 and 60 deg arrangements each miss 2 states.
        // ODrive assumes 120 deg separation - if a 60 deg setup is used, it can
        // be converted to 120 deg states by flipping the polarity of one sensor.
        uint8_t states = state_seen.to_ulong();
        uint8_t hall_polarity = 0;
        auto flip_detect = [](uint8_t states, unsigned int idx)->bool {
            return (~states & 0xFF) == (1<<(0+idx) | 1<<(7-idx));
        };
        if (flip_detect(states, 0)) {
            hall_polarity = 0b000;
        } else if (flip_detect(states, 1)) {
            hall_polarity = 0b001;
        } else if (flip_detect(states, 2)) {
            hall_polarity = 0b010;
        } else if (flip_detect(states, 3)) {
            hall_polarity = 0b100;
        } else {
            set_error(ERROR_ILLEGAL_HALL_STATE);
            return false;
        }
        config_.hall_polarity = hall_polarity;
        config_.hall_polarity_calibrated = true;
    }

    return success;
}

bool Encoder::run_hall_phase_calibration() {
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 30.0f; // run for 30 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;

    auto loop_cb = [this](bool const_vel) {
        if (const_vel)
            sample_hall_phase_ = true;
        // No need to cancel early
        return true;
    };

    // TODO: There is a race condition here with the execution in Encoder::update.
    // We should evaluate making thread execution synchronous with the control loops
    // at least optionally.
    // Perhaps the new loop_sync feature will give a loose timing guarantee that may be sufficient
    calibrate_hall_phase_ = true;
    config_.hall_edge_phcnt.fill(0.0f);
    hall_phase_calib_seen_count_.fill(0);
    bool success = axis_->run_lockin_spin(lockin_config, false, loop_cb);
    if (error_ & ERROR_ILLEGAL_HALL_STATE)
        success = false;

    if (success) {
        // Check deltas to dicern rotation direction
        float delta_phase = 0.0f;
        for (int i = 0; i < 6; i++) {
            int next_i = (i == 5) ? 0 : i+1;
            delta_phase += wrap_pm_pi(config_.hall_edge_phcnt[next_i] - config_.hall_edge_phcnt[i]);
        }
        // Correct reverse rotation
        if (delta_phase < 0.0f) {
            config_.direction = -1;
            for (int i = 0; i < 6; i++)
                config_.hall_edge_phcnt[i] = wrap_pm_pi(-config_.hall_edge_phcnt[i]);
        } else {
            config_.direction = 1;
        }
        // Normalize edge timing to 1st edge in sequence, and change units to counts
        float offset = config_.hall_edge_phcnt[0];
        for (int i = 0; i < 6; i++) {
            float& phcnt = config_.hall_edge_phcnt[i];
            phcnt = fmodf_pos((6.0f / (2.0f * M_PI)) * (phcnt - offset), 6.0f);
        }
    } else {
        config_.hall_edge_phcnt = hall_edge_defaults;
    }

    calibrate_hall_phase_ = false;
    return success;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;

    // Require index found if enabled
    if (config_.use_index && !index_found_) {
        set_error(ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }

    if (config_.mode == MODE_HALL && !config_.hall_polarity_calibrated) {
        set_error(ERROR_HALL_NOT_CALIBRATED_YET);
        return false;
    }

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    CRITICAL_SECTION() {
        // Reset state variables
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;

        float max_current_ramp = axis_->motor_.config_.calibration_current / start_lock_duration * 2.0f;
        axis_->open_loop_controller_.max_current_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_voltage_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;
        axis_->open_loop_controller_.target_current_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? axis_->motor_.config_.calibration_current : 0.0f;
        axis_->open_loop_controller_.target_voltage_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : axis_->motor_.config_.calibration_current;
        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(0 - config_.calib_scan_distance / 2.0f);

        axis_->motor_.current_control_.enable_current_control_src_ = (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);
        axis_->acim_estimator_.rotor_phase_src_.connect_to(&axis_->open_loop_controller_.phase_);

        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->acim_estimator_.rotor_phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
    }
    axis_->wait_for_control_iteration();

    axis_->motor_.arm(&axis_->motor_.current_control_);

    // go to start position of forward scan for start_lock_duration to get ready to scan
    for (size_t i = 0; i < (size_t)(start_lock_duration * 1000.0f); ++i) {
        if (!axis_->motor_.is_armed_) {
            return false; // TODO: return "disarmed" error code
        }
        if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
            axis_->motor_.disarm();
            return false; // TODO: return "aborted" error code
        }
        osDelay(1);
    }


    int32_t init_enc_val = shadow_count_;
    uint32_t num_steps = 0;
    int64_t encvaluesum = 0;

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = config_.calib_scan_omega;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
    }

    // scan forward
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(-INFINITY) >= config_.calib_scan_distance;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        axis_->motor_.disarm();
        return false;
    }

    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = std::abs(shadow_count_ - init_enc_val);
    if (std::abs(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        axis_->motor_.disarm();
        return false;
    }

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = -config_.calib_scan_omega;
    }

    // scan backwards
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(INFINITY) <= 0.0f;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Motor disarmed because of an error
    if (!axis_->motor_.is_armed_) {
        return false;
    }

    axis_->motor_.disarm();

    config_.phase_offset = encvaluesum / num_steps;
    int32_t residual = encvaluesum - ((int64_t)config_.phase_offset * (int64_t)num_steps);
    config_.phase_offset_float = (float)residual / (float)num_steps + 0.5f;  // add 0.5 to center-align state to phase

    is_ready_ = true;
    return true;
}

static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}

void Encoder::sample_now() {
    switch (mode_) {
        case MODE_INCREMENTAL: {
            tim_cnt_sample_ = (int16_t)timer_->Instance->CNT;
        } break;

        case MODE_HALL: {
            // do nothing: samples already captured in general GPIO capture
        } break;

        case MODE_SINCOS: {
            sincos_sample_s_ = get_adc_relative_voltage(get_gpio(config_.sincos_gpio_pin_sin)) - 0.5f;
            sincos_sample_c_ = get_adc_relative_voltage(get_gpio(config_.sincos_gpio_pin_cos)) - 0.5f;
        } break;

        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI:
        case MODE_SPI_ABS_AEAT:
        case MODE_SPI_ABS_RLS:
        case MODE_SPI_ABS_MA732:
        {
            abs_spi_start_transaction();
            // Do nothing
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }

    // Sample all GPIO digital input data registers, used for HALL sensors for example.
    for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); ++i) {
        port_samples_[i] = ports_to_sample[i]->IDR;
    }
}

bool Encoder::read_sampled_gpio(Stm32Gpio gpio) {
    for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); ++i) {
        if (ports_to_sample[i] == gpio.port_) {
            return port_samples_[i] & gpio.pin_mask_;
        }
    }
    return false;
}

void Encoder::decode_hall_samples() {
    hall_state_ = (read_sampled_gpio(hallA_gpio_) ? 1 : 0)
                | (read_sampled_gpio(hallB_gpio_) ? 2 : 0)
                | (read_sampled_gpio(hallC_gpio_) ? 4 : 0);
}

bool Encoder::abs_spi_start_transaction() {
    if (mode_ & MODE_FLAG_ABS){
        if (Stm32SpiArbiter::acquire_task(&spi_task_)) {
            spi_task_.ncs_gpio = abs_spi_cs_gpio_;
            spi_task_.length = 1; // AS5047P 逐帧流水线：所有事务都只有一帧
            spi_task_.on_complete = [](void* ctx, bool success) { ((Encoder*)ctx)->abs_spi_cb(success); };
            spi_task_.on_complete_ctx = this;
            spi_task_.next = nullptr;

            // ★ 零位标定 / OTP 烧写期间改写本周期要发的那一帧。注意：仍然是同一个
            // spi_task_、同一笔事务，仲裁器链表长度完全不变 —— 这一点是关键。若从线程
            // 额外插入阻塞事务，一旦某次 start() 失败该任务就会永久卡在表头，链表再也
            // 不推进，周期读连续 51ms 收不到样本 → ENCODER_ERROR_ABS_SPI_COM_FAIL。
            bool hijack = false;
            if (mode_ == MODE_SPI_ABS_AMS) {
                hijack = (as5047_otp_step_ != AS5047_OTP_OFF) ? as5047_otp_prepare()
                                                             : as5047_seq_prepare();
            }
            if (hijack) {
                spi_task_.tx_buf = (uint8_t*)&as5047_seq_tx_;
                spi_task_.rx_buf = (uint8_t*)&as5047_seq_rx_;
            } else {
                spi_task_.tx_buf = (uint8_t*)abs_spi_dma_tx_;
                spi_task_.rx_buf = (uint8_t*)abs_spi_dma_rx_;
            }

            spi_arbiter_->transfer_async(&spi_task_);
        } else {
            return false;
        }
    }
    return true;
}

uint8_t ams_parity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

uint8_t cui_parity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    return ~v & 3;
}

///////////////////////////////////////////////////////////////////////////
//  AS5047P 零位标定 —— 见 磁编AS5047P-ATSM.pdf
//
//  命令帧 (Figure 13)：bit15 = PARC(bit14:0 偶校验)，bit14 = R/W(0 写 / 1 读)，bit13:0 = 地址
//  读数据帧 (Figure 14)：bit15 = PARD(偶校验)，bit14 = EF(帧错误)，bit13:0 = 数据
//  写数据帧 (Figure 16)：bit15 = PARD(偶校验)，bit14 = 恒 0，bit13:0 = 数据
//
//  ★ 关键：AS5047P 是"逐帧流水线"协议，命令帧和数据帧各占一个 CS 低电平周期，而不是拼在
//    同一个 CS 段里发。手册原话：
//        "The SPI read is sampled on the rising edge of CSn and the data is transmitted
//         on MISO with the next read command."
//    本工程的周期读就是这么工作的（本周期发命令、下周期收数据），所以这里复用同样的
//    机制即可 —— 每个采样周期只发一帧，全部事务的 length 都是 1。
///////////////////////////////////////////////////////////////////////////
static uint16_t as5047_build_frame(uint16_t payload14, bool is_read) {
    uint16_t frame = payload14 & 0x3fffu;
    if (is_read) {
        frame |= (1u << 14);
    }
    // ams_parity() 返回 16bit 整体的偶校验；此时 bit15 = 0，等价于对 bit14:0 取偶校验
    return frame | ((uint16_t)ams_parity(frame) << 15);
}

// 按当前步骤填好本周期要发的帧；返回 false 表示无任务，走正常的周期读（0xFFFF）
bool Encoder::as5047_seq_prepare() {
    switch (as5047_step_) {
        // --- 可选：写 ZPOSM / ZPOSL = as5047_zpos_target_ ---
        case AS5047_WR_M_CMD: as5047_seq_tx_ = as5047_build_frame(0x0016u, false); return true;
        case AS5047_WR_M_DAT: as5047_seq_tx_ = as5047_build_frame((uint16_t)((as5047_zpos_target_ >> 6) & 0x00ffu), false); return true;
        case AS5047_WR_L_CMD: as5047_seq_tx_ = as5047_build_frame(0x0017u, false); return true;
        case AS5047_WR_L_DAT: as5047_seq_tx_ = as5047_build_frame((uint16_t)(as5047_zpos_target_ & 0x003fu), false); return true;

        // --- 读回 ZPOSM / ZPOSL / 计数 ---
        case AS5047_RD_ZM_CMD: as5047_seq_tx_ = as5047_build_frame(0x0016u, true); return true;
        case AS5047_RD_ZL_CMD: as5047_seq_tx_ = as5047_build_frame(0x0017u, true); return true;
        case AS5047_RD_CMD:    as5047_seq_tx_ = as5047_build_frame(0x3fffu, true); return true; // ANGLECOM

        // --- 读磁场强度 / 诊断信息 ---
        case AS5047_RD_MAG_CMD:  as5047_seq_tx_ = as5047_build_frame(0x3ffdu, true); return true; // MAG
        case AS5047_RD_DIAG_CMD: as5047_seq_tx_ = as5047_build_frame(0x3ffcu, true); return true; // DIAAGC

        case AS5047_RD_ZM_DAT:
        case AS5047_RD_ZL_DAT:
        case AS5047_RD_DAT:
        case AS5047_RD_MAG_DAT:
        case AS5047_RD_DIAG_DAT: as5047_seq_tx_ = as5047_build_frame(0x0000u, true); return true; // NOP

        default:
            return false; // AS5047_IDLE / AS5047_DONE
    }
}

// 标定帧不是角度，所以这里不设置 abs_spi_pos_updated_（update() 里会继续用上一笔 pos_abs_，
// 只跳过十几个采样周期，对 spi_error_rate_ 的贡献远低于 0.05 阈值）
void Encoder::as5047_seq_advance() {
    const uint16_t rx = as5047_seq_rx_;
    const bool rx_ok = !ams_parity(rx) && !(rx & (1u << 14)); // 偶校验正确且 EF 未置位

    switch (as5047_step_) {
        case AS5047_WR_M_CMD: as5047_step_ = AS5047_WR_M_DAT; break;
        case AS5047_WR_M_DAT: as5047_step_ = AS5047_WR_L_CMD; break;
        case AS5047_WR_L_CMD: as5047_step_ = AS5047_WR_L_DAT; break;
        case AS5047_WR_L_DAT: as5047_step_ = AS5047_RD_ZM_CMD; break;

        case AS5047_RD_ZM_CMD: as5047_step_ = AS5047_RD_ZM_DAT; break;
        case AS5047_RD_ZM_DAT:
            if (rx_ok) as5047_zposm_rb_ = rx & 0x3fffu;
            as5047_step_ = AS5047_RD_ZL_CMD;
            break;
        case AS5047_RD_ZL_CMD: as5047_step_ = AS5047_RD_ZL_DAT; break;
        case AS5047_RD_ZL_DAT:
            if (rx_ok) as5047_zposl_rb_ = rx & 0x3fffu;
            as5047_step_ = AS5047_RD_CMD;
            break;

        case AS5047_RD_CMD:   as5047_step_ = AS5047_RD_DAT; break; // 本帧是上一次访问的输出，丢弃
        case AS5047_RD_DAT:
            if (rx_ok) as5047_rd_count_ = rx & 0x3fffu;
            as5047_step_ = AS5047_DONE;
            break;

        // --- 读 MAG → 读 DIAAGC（入口由 as5047_read_diagnostics() 设为 RD_MAG_CMD）---
        case AS5047_RD_MAG_CMD:  as5047_step_ = AS5047_RD_MAG_DAT; break;
        case AS5047_RD_MAG_DAT:
            if (rx_ok) as5047_mag_ = rx & 0x3fffu;
            as5047_step_ = AS5047_RD_DIAG_CMD;
            break;
        case AS5047_RD_DIAG_CMD: as5047_step_ = AS5047_RD_DIAG_DAT; break;
        case AS5047_RD_DIAG_DAT:
            if (rx_ok) as5047_diag_ = rx & 0x3fffu;
            as5047_step_ = AS5047_DONE;
            break;

        default: break;
    }
}

// 等待轴静止。零位是一个位置量，而且序列里前后两次采样相隔若干采样周期，轴一转就会在
// 两次采样之间引入额外偏差，让结果无法判读 —— 所以标定前必须先确认轴没在动。
bool Encoder::as5047_wait_still(uint32_t timeout_ms, uint32_t tol_counts) {
    constexpr int32_t CPR = 16384; // AS5047P 14bit
    int32_t prev = pos_abs_;
    uint32_t waited = 0;

    while (waited < timeout_ms) {
        osDelay(2);
        waited += 2;
        int32_t cur = pos_abs_;
        int32_t d = cur - prev;
        if (d >  CPR / 2) d -= CPR;  // 处理 0/16383 回绕
        if (d < -CPR / 2) d += CPR;
        if (d < 0) d = -d;
        if ((uint32_t)d <= tol_counts) {
            return true;
        }
        prev = cur;
    }
    return false;
}

// 读回 ZPOSM/ZPOSL 和计数，不动硬件任何状态。上电后第一次调用读到的 ZPOS 就是 OTP 内容。
bool Encoder::as5047_read_zero_and_count(uint16_t* zpos, uint16_t* count, uint32_t timeout_ms) {
    if (mode_ != MODE_SPI_ABS_AMS) {
        return false;
    }

    as5047_rd_count_ = 0xffffu;
    as5047_zposm_rb_ = 0xffffu;
    as5047_zposl_rb_ = 0xffffu;
    as5047_step_ = AS5047_RD_ZM_CMD;

    uint32_t waited = 0;
    while (as5047_step_ != AS5047_DONE) {
        osDelay(1);
        if (++waited >= timeout_ms) {
            as5047_step_ = AS5047_IDLE;
            return false;
        }
    }
    as5047_step_ = AS5047_IDLE;

    // ZPOSM 是 8bit、ZPOSL 是 6bit，读回 0xFFFF 只可能是没读到
    if (as5047_zposm_rb_ == 0xffffu || as5047_zposl_rb_ == 0xffffu || as5047_rd_count_ == 0xffffu) {
        return false;
    }

    if (zpos)  *zpos  = (uint16_t)(((as5047_zposm_rb_ & 0x00ffu) << 6) | (as5047_zposl_rb_ & 0x003fu));
    if (count) *count = as5047_rd_count_;
    return true;
}

// 软写 ZPOSM/ZPOSL = zpos，然后读回 ZPOSM 和计数。
// 只轮询状态机，**不做任何阻塞式 SPI**，因此无论底层发生什么都能在 timeout_ms 内返回。
bool Encoder::as5047_write_zpos(uint16_t zpos, uint16_t* count, uint16_t* zposm_rb,
                                uint32_t timeout_ms) {
    if (mode_ != MODE_SPI_ABS_AMS) {
        return false;
    }

    as5047_zpos_target_ = (uint16_t)(zpos & 0x3fffu);
    as5047_rd_count_ = 0xffffu;
    as5047_zposm_rb_ = 0xffffu;
    as5047_zposl_rb_ = 0xffffu;
    as5047_step_ = AS5047_WR_M_CMD;

    uint32_t waited = 0;
    while (as5047_step_ != AS5047_DONE) {
        osDelay(1);
        if (++waited >= timeout_ms) {
            as5047_step_ = AS5047_IDLE;
            return false;
        }
    }
    as5047_step_ = AS5047_IDLE;

    if (count)    *count    = as5047_rd_count_;
    if (zposm_rb) *zposm_rb = as5047_zposm_rb_;
    return true;
}

// 读磁场强度 MAG(0x3FFD) 和诊断信息 DIAAGC(0x3FFC)。纯读，无副作用。
bool Encoder::as5047_read_diagnostics(uint16_t* magnitude, uint16_t* diaagc, uint32_t timeout_ms) {
    if (mode_ != MODE_SPI_ABS_AMS) {
        return false;
    }

    as5047_mag_ = 0xffffu;
    as5047_diag_ = 0xffffu;
    as5047_step_ = AS5047_RD_MAG_CMD;

    uint32_t waited = 0;
    while (as5047_step_ != AS5047_DONE) {
        osDelay(1);
        if (++waited >= timeout_ms) {
            as5047_step_ = AS5047_IDLE;
            return false;
        }
    }
    as5047_step_ = AS5047_IDLE;

    // MAG 是 14bit，读回 0xFFFF 只可能是没读到
    if (as5047_mag_ == 0xffffu || as5047_diag_ == 0xffffu) {
        return false;
    }

    if (magnitude) *magnitude = as5047_mag_;
    if (diaagc)    *diaagc    = as5047_diag_;
    return true;
}

///////////////////////////////////////////////////////////////////////////
//  AS5047P OTP 烧写（不可逆，只能烧一次）
//  严格按手册第 27-28 页的 19 步流程（Figure 35-38）展开；本段负责 step 2~15，
//  step 1/16 的"Power ON cycle"与最终 step 17~19 的复验必须由人工断电上电完成。
//  电气前提：烧写期间 VDD 必须 3.3~3.5V（5V 模式 4.5~5.5V），瞬时电流可达 100mA，
//  温度 25±20℃，且手册明确写了"A reprogramming is not allowed"。
///////////////////////////////////////////////////////////////////////////

// PROG 轮询上限：每次轮询占 2 个采样周期，4000 次约 1s
static constexpr uint32_t AS5047_OTP_POLL_LIMIT = 4000;

// 待写入 0x0016+idx 的值（idx 顺序与寄存器地址顺序一致，方便与读回值直接比对）
uint16_t Encoder::as5047_otp_write_value(uint8_t idx) const {
    return (idx < 4) ? as5047_otp_expected_[idx] : 0u;
}

bool Encoder::as5047_otp_prepare() {
    switch (as5047_otp_step_) {
        // 读 0x0016+idx（预检用 idx 0/1 读 ZPOSM/ZPOSL；step1/step2 用 idx 0..3）
        case AS5047_OTP_PRE_CMD:
        case AS5047_OTP_RD1_CMD:
        case AS5047_OTP_RD2_CMD:
            as5047_seq_tx_ = as5047_build_frame((uint16_t)(0x0016u + as5047_otp_idx_), true);
            return true;
        case AS5047_OTP_PRE_DAT:
        case AS5047_OTP_RD1_DAT:
        case AS5047_OTP_RD2_DAT:
            as5047_seq_tx_ = as5047_build_frame(0x0000u, true);
            return true;

        // 写 0x0016+idx
        case AS5047_OTP_WR_CMD:
        case AS5047_OTP_CLR_CMD:
            as5047_seq_tx_ = as5047_build_frame((uint16_t)(0x0016u + as5047_otp_idx_), false);
            return true;
        case AS5047_OTP_WR_DAT:
            as5047_seq_tx_ = as5047_build_frame(as5047_otp_write_value(as5047_otp_idx_), false);
            return true;
        case AS5047_OTP_CLR_DAT: // step 11：把整个非易失内容写 0
            as5047_seq_tx_ = as5047_build_frame(0x0000u, false);
            return true;

        // 写 PROG 寄存器
        case AS5047_OTP_PROGEN_CMD:
        case AS5047_OTP_BURN_CMD:
        case AS5047_OTP_PROGVER_CMD:
        case AS5047_OTP_OTPREF_CMD:
            as5047_seq_tx_ = as5047_build_frame(0x0003u, false);
            return true;
        case AS5047_OTP_PROGEN_DAT:  as5047_seq_tx_ = as5047_build_frame(0x0001u, false); return true; // PROGEN
        case AS5047_OTP_BURN_DAT:    as5047_seq_tx_ = as5047_build_frame(0x0008u, false); return true; // PROGOTP
        case AS5047_OTP_PROGVER_DAT: as5047_seq_tx_ = as5047_build_frame(0x0040u, false); return true; // PROGVER
        case AS5047_OTP_OTPREF_DAT:  as5047_seq_tx_ = as5047_build_frame(0x0004u, false); return true; // OTPREF

        // 轮询 PROG
        case AS5047_OTP_POLL_CMD: as5047_seq_tx_ = as5047_build_frame(0x0003u, true); return true;
        case AS5047_OTP_POLL_DAT: as5047_seq_tx_ = as5047_build_frame(0x0000u, true); return true;

        default:
            return false;
    }
}

// 比对 rd[0..3] = ZPOSM/ZPOSL/SETTINGS1/SETTINGS2 与期望值。
// SETTINGS1 的 bit0 是出厂预编程位，读回恒为 1，比对时必须屏蔽（手册 step 7/15/18 强制要求）
bool Encoder::as5047_otp_verify(const uint16_t* rd) const {
    if ((rd[0] & 0x00ffu) != as5047_otp_expected_[0]) return false;
    if ((rd[1] & 0x003fu) != as5047_otp_expected_[1]) return false;
    if ((rd[2] & 0xfffeu) != (as5047_otp_expected_[2] & 0xfffeu)) return false;
    if ((rd[3] & 0x3fffu) != as5047_otp_expected_[3]) return false;
    return true;
}

void Encoder::as5047_otp_advance() {
    const uint16_t rx = as5047_seq_rx_;
    const bool rx_ok = !ams_parity(rx) && !(rx & (1u << 14));

    switch (as5047_otp_step_) {
        // ---- 预检：ZPOSM/ZPOSL 都应读回 0。非 0 说明 OTP 已经烧过了 ----
        case AS5047_OTP_PRE_CMD: as5047_otp_step_ = AS5047_OTP_PRE_DAT; break;
        case AS5047_OTP_PRE_DAT:
            if (!rx_ok || (rx & 0x3fffu) != 0u) {
                as5047_otp_fail_ = 5;
                as5047_otp_step_ = AS5047_OTP_FAILED;
            } else if (++as5047_otp_idx_ < 2) {
                as5047_otp_step_ = AS5047_OTP_PRE_CMD;
            } else {
                as5047_otp_idx_ = 0;
                as5047_otp_step_ = AS5047_OTP_WR_CMD;
            }
            break;

        // ---- step 2/5：写 SETTINGS1/2 和 ZPOSM/ZPOSL ----
        case AS5047_OTP_WR_CMD: as5047_otp_step_ = AS5047_OTP_WR_DAT; break;
        case AS5047_OTP_WR_DAT:
            if (++as5047_otp_idx_ < 4) {
                as5047_otp_step_ = AS5047_OTP_WR_CMD;
            } else {
                as5047_otp_idx_ = 0;
                as5047_otp_step_ = AS5047_OTP_RD1_CMD;
            }
            break;

        // ---- step 6/7：读回 step1 并比对，不对就绝不往下烧 ----
        case AS5047_OTP_RD1_CMD: as5047_otp_step_ = AS5047_OTP_RD1_DAT; break;
        case AS5047_OTP_RD1_DAT:
            if (rx_ok) as5047_otp_rd1_[as5047_otp_idx_] = rx & 0x3fffu;
            if (++as5047_otp_idx_ < 4) {
                as5047_otp_step_ = AS5047_OTP_RD1_CMD;
            } else if (as5047_otp_verify(as5047_otp_rd1_)) {
                as5047_otp_step_ = AS5047_OTP_PROGEN_CMD;
            } else {
                as5047_otp_fail_ = 2;
                as5047_otp_step_ = AS5047_OTP_FAILED;
            }
            break;

        // ---- step 8/9：PROGEN=1 后过 2 帧（250us）再 PROGOTP=1，满足手册">=100us" ----
        case AS5047_OTP_PROGEN_CMD: as5047_otp_step_ = AS5047_OTP_PROGEN_DAT; break;
        case AS5047_OTP_PROGEN_DAT: as5047_otp_step_ = AS5047_OTP_BURN_CMD; break;
        case AS5047_OTP_BURN_CMD:   as5047_otp_step_ = AS5047_OTP_BURN_DAT; break;
        case AS5047_OTP_BURN_DAT:
            as5047_otp_polls_ = 0;
            as5047_otp_step_ = AS5047_OTP_POLL_CMD;
            break;

        // ---- step 10：轮询 PROG 直到读回 0x0001 ----
        case AS5047_OTP_POLL_CMD: as5047_otp_step_ = AS5047_OTP_POLL_DAT; break;
        case AS5047_OTP_POLL_DAT:
            if (rx_ok && (rx & 0x3fffu) == 0x0001u) {
                as5047_otp_idx_ = 0;
                as5047_otp_step_ = AS5047_OTP_CLR_CMD;
            } else if (++as5047_otp_polls_ > AS5047_OTP_POLL_LIMIT) {
                as5047_otp_fail_ = 3;
                as5047_otp_step_ = AS5047_OTP_FAILED;
            } else {
                as5047_otp_step_ = AS5047_OTP_POLL_CMD;
            }
            break;

        // ---- step 11：把非易失内容全部写 0（这样 step 13 刷新后读到的才是真 OTP 内容）----
        case AS5047_OTP_CLR_CMD: as5047_otp_step_ = AS5047_OTP_CLR_DAT; break;
        case AS5047_OTP_CLR_DAT:
            if (++as5047_otp_idx_ < 4) {
                as5047_otp_step_ = AS5047_OTP_CLR_CMD;
            } else {
                as5047_otp_step_ = AS5047_OTP_PROGVER_CMD;
            }
            break;

        // ---- step 12/13：PROGVER=1（guard band）→ OTPREF=1（用 OTP 刷新易失内容）----
        case AS5047_OTP_PROGVER_CMD: as5047_otp_step_ = AS5047_OTP_PROGVER_DAT; break;
        case AS5047_OTP_PROGVER_DAT: as5047_otp_step_ = AS5047_OTP_OTPREF_CMD; break;
        case AS5047_OTP_OTPREF_CMD:  as5047_otp_step_ = AS5047_OTP_OTPREF_DAT; break;
        case AS5047_OTP_OTPREF_DAT:
            as5047_otp_idx_ = 0;
            as5047_otp_step_ = AS5047_OTP_RD2_CMD;
            break;

        // ---- step 14/15：读回 step2 并比对，这才是"烧进去了"的真凭据 ----
        case AS5047_OTP_RD2_CMD: as5047_otp_step_ = AS5047_OTP_RD2_DAT; break;
        case AS5047_OTP_RD2_DAT:
            if (rx_ok) as5047_otp_rd2_[as5047_otp_idx_] = rx & 0x3fffu;
            if (++as5047_otp_idx_ < 4) {
                as5047_otp_step_ = AS5047_OTP_RD2_CMD;
            } else if (as5047_otp_verify(as5047_otp_rd2_)) {
                as5047_otp_step_ = AS5047_OTP_DONE;
            } else {
                as5047_otp_fail_ = 4;
                as5047_otp_step_ = AS5047_OTP_FAILED;
            }
            break;

        default: break;
    }
}

// 发起 OTP 烧写。立刻返回，不代表任何步骤已完成 —— 结果用 as5047_burn_otp_poll() 查。
bool Encoder::as5047_burn_otp_start(uint16_t zpos, uint32_t timeout_ms) {
    if (mode_ != MODE_SPI_ABS_AMS || as5047_otp_step_ != AS5047_OTP_OFF) {
        return false;
    }

    // 期望烧进 OTP 的内容，顺序与 0x0016..0x0019 一致
    as5047_otp_expected_[0] = (uint16_t)((zpos >> 6) & 0x00ffu); // ZPOSM
    as5047_otp_expected_[1] = (uint16_t)(zpos & 0x003fu);        // ZPOSL
    as5047_otp_expected_[2] = 0x0000u; // SETTINGS1：bit0 出厂位必须清 0，其余保持默认(ABZ/UVW 未使用)
    as5047_otp_expected_[3] = 0x0000u; // SETTINGS2

    for (uint8_t i = 0; i < 4; ++i) {
        as5047_otp_rd1_[i] = 0;
        as5047_otp_rd2_[i] = 0;
    }
    as5047_otp_fail_ = 0;
    as5047_otp_idx_ = 0;
    as5047_otp_polls_ = 0;
    as5047_step_ = AS5047_IDLE; // 确保没有残留的半截序列会和 OTP 序列抢帧
    // 整条序列期间周期读被占用，abs_spi_pos_updated_ 不置位，把 COM_FAIL 计数清零避免误报
    spi_error_rate_ = 0.0f;
    as5047_otp_deadline_ = HAL_GetTick() + timeout_ms;
    as5047_otp_step_ = AS5047_OTP_PRE_CMD;
    return true;
}

bool Encoder::as5047_burn_otp_poll(uint16_t* out_zpos, uint8_t* fail_code) {
    if (as5047_otp_step_ != AS5047_OTP_DONE && as5047_otp_step_ != AS5047_OTP_FAILED) {
        // 还没跑完。序列本身靠采样中断推进，这里只做墙上时钟超时保护
        if ((int32_t)(HAL_GetTick() - as5047_otp_deadline_) < 0) {
            return false;
        }
        as5047_otp_step_ = AS5047_OTP_OFF;
        if (fail_code) *fail_code = 1;
        if (out_zpos) *out_zpos = 0xffffu;
        return true;
    }

    const bool ok = (as5047_otp_step_ == AS5047_OTP_DONE);
    as5047_otp_step_ = AS5047_OTP_OFF;
    if (fail_code) *fail_code = ok ? 0u : as5047_otp_fail_;
    if (out_zpos) {
        // 从 step2 读回值重建实际烧进去的零位
        *out_zpos = ok ? (uint16_t)(((as5047_otp_rd2_[0] & 0x00ffu) << 6) | (as5047_otp_rd2_[1] & 0x003fu))
                       : 0xffffu;
    }
    return true;
}

void Encoder::abs_spi_cb(bool success) {
    uint16_t pos;

    if (!success) {
        goto done;
    }

    // OTP 烧写序列期间周期读被整段占用，位置本来就不更新；每周期把 COM_FAIL 计数器压住，
    // 免得长序列把 spi_error_rate_ 推过 0.05 阈值误报 ENCODER_ERROR_ABS_SPI_COM_FAIL
    if (as5047_otp_step_ != AS5047_OTP_OFF) {
        spi_error_rate_ = 0.0f;
        as5047_otp_advance();
        goto done;
    }

    // AS5047P 零位标定期间收回来的帧不是角度，交给序列机处理，不当作位置数据
    if (as5047_step_ != AS5047_IDLE) {
        as5047_seq_advance();
        goto done;
    }

    switch (mode_) {
        case MODE_SPI_ABS_AMS: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            // check if parity is correct (even) and error flag clear
            if (ams_parity(rawVal) || ((rawVal >> 14) & 1)) {
                goto done;
            }
            pos = rawVal & 0x3fff;
        } break;

        case MODE_SPI_ABS_CUI: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            // check if parity is correct
            if (cui_parity(rawVal)) {
                goto done;
            }
            pos = rawVal & 0x3fff;
        } break;

        case MODE_SPI_ABS_RLS: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            pos = (rawVal >> 2) & 0x3fff;
        } break;

        case MODE_SPI_ABS_MA732: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            pos = (rawVal >> 2) & 0x3fff;
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           goto done;
        } break;
    }

    pos_abs_ = pos;
    abs_spi_pos_updated_ = true;
    if (config_.pre_calibrated) {
        is_ready_ = true;
    }

done:
    Stm32SpiArbiter::release_task(&spi_task_);
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode and init cs pin
#if HW_VERSION_MAJOR == 4
    if (mode_ == MODE_SPI_ABS_MA732)
        abs_spi_cs_gpio_ = {GPIOA, GPIO_PIN_15};
    else
#else
    abs_spi_cs_gpio_ = get_gpio(config_.abs_spi_cs_gpio_pin);
#endif
    abs_spi_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

// Note that this may return counts +1 or -1 without any wrapping
int32_t Encoder::hall_model(float internal_pos) {
    int32_t base_cnt = (int32_t)std::floor(internal_pos);

    float pos_in_range = fmodf_pos(internal_pos, 6.0f);
    int pos_idx = (int)pos_in_range;
    if (pos_idx == 6) pos_idx = 5; // in case of rounding error
    int next_i = (pos_idx == 5) ? 0 : pos_idx+1;

    float below_edge = config_.hall_edge_phcnt[pos_idx];
    float above_edge = config_.hall_edge_phcnt[next_i];

    // if we are blow the "below" edge, we are the count under
    if (wrap_pm(pos_in_range - below_edge, 6.0f) < 0.0f)
        return base_cnt - 1;
    // if we are above the "above" edge, we are the count over
    else if (wrap_pm(pos_in_range - above_edge, 6.0f) > 0.0f)
        return base_cnt + 1;
    // otherwise we are in the nominal count (or completely lost)
    return base_cnt;
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; //LATCH

    switch (mode_) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;

        case MODE_HALL: {
            decode_hall_samples();
            if (sample_hall_states_) {
                states_seen_count_[hall_state_]++;
            }
            if (config_.hall_polarity_calibrated) {
                int32_t hall_cnt;
                if (decode_hall((hall_state_ ^ config_.hall_polarity), &hall_cnt)) {
                    if (calibrate_hall_phase_) {
                        if (sample_hall_phase_ && last_hall_cnt_.has_value()) {
                            int mod_hall_cnt = mod(hall_cnt - last_hall_cnt_.value(), 6);
                            size_t edge_idx;
                            if (mod_hall_cnt == 0) { goto skip; } // no count - do nothing
                            else if (mod_hall_cnt == 1) { // counted up
                                edge_idx = hall_cnt;
                            } else if (mod_hall_cnt == 5) { // counted down
                                edge_idx = last_hall_cnt_.value();
                            } else {
                                set_error(ERROR_ILLEGAL_HALL_STATE);
                                return false;
                            }

                            auto maybe_phase = axis_->open_loop_controller_.phase_.any();
                            if (maybe_phase) {
                                float phase = maybe_phase.value();
                                // Early increment to get the right divisor in recursive average
                                hall_phase_calib_seen_count_[edge_idx]++;
                                float& edge_phase = config_.hall_edge_phcnt[edge_idx];
                                if (hall_phase_calib_seen_count_[edge_idx] == 1)
                                    edge_phase = phase;
                                else {
                                    // circularly wrapped recursive average
                                    edge_phase += (phase - edge_phase) / hall_phase_calib_seen_count_[edge_idx];
                                    edge_phase = wrap_pm_pi(edge_phase);
                                }
                            }
                        }
                    skip:
                        last_hall_cnt_ = hall_cnt;

                        return true; // Skip all velocity and phase estimation
                    }

                    delta_enc = hall_cnt - count_in_cpr_;
                    delta_enc = mod(delta_enc, 6);
                    if (delta_enc > 3)
                        delta_enc -= 6;
                } else {
                    if (!config_.ignore_illegal_hall_state) {
                        set_error(ERROR_ILLEGAL_HALL_STATE);
                        return false;
                    }
                }
            }
        } break;

        case MODE_SINCOS: {
            float phase = fast_atan2(sincos_sample_s_, sincos_sample_c_);
            int fake_count = (int)(1000.0f * phase);
            //CPR = 6283 = 2pi * 1k

            delta_enc = fake_count - count_in_cpr_;
            delta_enc = mod(delta_enc, 6283);
            if (delta_enc > 6283/2)
                delta_enc -= 6283;
        } break;
        
        case MODE_SPI_ABS_RLS:
        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI: 
        case MODE_SPI_ABS_AEAT:
        case MODE_SPI_ABS_MA732: {
            if (abs_spi_pos_updated_ == false) {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.05f) {
                    set_error(ERROR_ABS_SPI_COM_FAIL);
                    return false;
                }
            } else {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }

            abs_spi_pos_updated_ = false;
            delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr/2) {
                delta_enc -= config_.cpr;
            }

        }break;
        default: {
            set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if(mode_ & MODE_FLAG_ABS)
        count_in_cpr_ = pos_abs_latched;

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
    // Encoder model
    auto encoder_model = [this](float internal_pos)->int32_t {
        if (config_.mode == MODE_HALL)
            return hall_model(internal_pos);
        else
            return (int32_t)std::floor(internal_pos);
    };
    // discrete phase detector
    float delta_pos_counts = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    // pll feedback
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;
    
    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_.any().value_or(0.0f);
    pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular = fmodf_pos(pos_circular, axis_->controller_.config_.circular_setpoint_range);
    pos_circular_ = pos_circular;

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.phase_offset_float);
    
    if (is_ready_) {
        phase_ = wrap_pm_pi(ph) * config_.direction;
        phase_vel_ = (2*M_PI) * *vel_estimate_.present() * axis_->motor_.config_.pole_pairs * config_.direction;
    }

    return true;
}
