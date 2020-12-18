
#include "ma732.hpp"
#include <cmsis_os.h>

static const SPI_InitTypeDef spi_config_ = {
    .Mode = SPI_MODE_MASTER,
    .Direction = SPI_DIRECTION_2LINES,
    .DataSize = SPI_DATASIZE_16BIT,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32,
    .FirstBit = SPI_FIRSTBIT_MSB,
    .TIMode = SPI_TIMODE_DISABLE,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .CRCPolynomial = 10,
};

bool Ma732::RegisterFile::operator==(const RegisterFile& other) {
    return (zero_pos_low == other.zero_pos_low)
        && (zero_pos_high == other.zero_pos_high)
        && (bias_current_trimming == other.bias_current_trimming)
        && (enable_trimming == other.enable_trimming)
        && (ppt_ilip == other.ppt_ilip)
        && (pulses_per_turn == other.pulses_per_turn)
        && (magnetic_threshold == other.magnetic_threshold)
        && (reverse_dir == other.reverse_dir)
        && (filter_window == other.filter_window)
        && (hysteresis == other.hysteresis);
}

Ma732::Ma732(Stm32SpiArbiter& spi_arbiter, Stm32Gpio ncs_gpio) :
    spi_arbiter_(spi_arbiter), ncs_gpio_(ncs_gpio),
    //spi_task_({
    //    .config = 
    //}),
    tx_buf_(0)
{
}

bool Ma732::init() {
    RegisterFile old_config;
    bool read_ok = read_reg(kRegNameZeroLow, &old_config.zero_pos_low)
                && read_reg(kRegNameZeroHigh, &old_config.zero_pos_high)
                && read_reg(kRegNameBiasCurrentTrimming, &old_config.bias_current_trimming)
                && read_reg(kRegNameEnableTrimming, &old_config.enable_trimming)
                && read_reg(kRegNamePptIlip, &old_config.ppt_ilip)
                && read_reg(kRegNamePulsesPerTurn, &old_config.pulses_per_turn)
                && read_reg(kRegNameMagneticThreshold, &old_config.magnetic_threshold)
                && read_reg(kRegNameReverseDir, &old_config.reverse_dir)
                && read_reg(kRegNameFilterWindow, &old_config.filter_window)
                && read_reg(kRegNameHysteresis, &old_config.hysteresis);

    if (!read_ok) {
        return false;
    }

    RegisterFile new_config;
    new_config.zero_pos_low = 0; // Z[7:0]: no position offset
    new_config.zero_pos_high = 0; // Z[15:8]: no position offset
    new_config.bias_current_trimming = 0; // BCT: no bias current trimming
    new_config.enable_trimming =
          (0b0 << 1) // ETX: no bias current trimming on X axis
        | (0b0 << 0); // ETY: no bias current trimming on Y axis
    new_config.ppt_ilip =
          (0b11 << 6) // PPT[1:0]: see below
        | (0b0000 << 2); // ILIP: index length/position (only relevant for the ABZ interface which we don't use)
    new_config.pulses_per_turn = 0b11111111; // PPT[9:2]: 1024 pulses per turn (only relevant for the ABZ interface which we don't use)
    new_config.magnetic_threshold = 
          (0b000 << 5) // MGLT: magnetic field low threshold: hysteresis between 20mT 26mT
        | (0b111 << 2); // MGHT: magnetic field high threshold: hysteresis between 120mT 126mT
    new_config.reverse_dir = (0b0 << 7); // RD: don't reverse direction
    new_config.filter_window = 119; // FW: 1024us filter time constant, 370Hz cutoff frequency
    new_config.hysteresis = 156; // HYS: 0.52° ABZ hysteresis (only relevant for the ABZ interface which we don't use) [see note below!]
    
    // The chip seems to ignore writes to the hysteresis register. So let's just ignore it.
    new_config.hysteresis = old_config.hysteresis;

    if (new_config == old_config) {
        // The encoder stores its configuration in non-volatile memory. This has
        // a guaranteed lifespan of only 1000 write cycles so to save on write
        // cycles we only rewrite the configuration if necessary.
        return true;
    }
    
    bool write_ok = write_reg(kRegNameZeroLow, new_config.zero_pos_low)
                 && write_reg(kRegNameZeroHigh, new_config.zero_pos_high)
                 && write_reg(kRegNameBiasCurrentTrimming, new_config.bias_current_trimming)
                 && write_reg(kRegNameEnableTrimming, new_config.enable_trimming)
                 && write_reg(kRegNamePptIlip, new_config.ppt_ilip)
                 && write_reg(kRegNamePulsesPerTurn, new_config.pulses_per_turn)
                 && write_reg(kRegNameMagneticThreshold, new_config.magnetic_threshold)
                 && write_reg(kRegNameReverseDir, new_config.reverse_dir)
                 && write_reg(kRegNameFilterWindow, new_config.filter_window)
                 && write_reg(kRegNameHysteresis, new_config.hysteresis);

    if (!write_ok) {
        return false;
    }

    initialized_ = true;
    return true;
}

std::optional<float> Ma732::get_field_strength() {
    // This function incrementally adjusts the lower and upper magnetic
    // thresholds of the encoder like this:
    //  [......]
    //   [....]
    //    [..]
    //     []
    // In each state, the threshold alarm bits are read. This allows us to
    // estimate the actual magnetic strength.

    // If this function fails midway the device is left in a state unsuitable
    // for operation
    bool was_initialized = initialized_;
    initialized_ = false;

    uint8_t responses = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        // set MGLT to i and MGHT to (7-i)
        uint8_t magnetic_threshold = (i << 5) | ((7 - i) << 2);
        if (!write_reg(kRegNameMagneticThreshold, magnetic_threshold)) {
            return false;
        }
        uint8_t threshold = 0;
        if (!read_reg(kRegNameThresholdStatus, &threshold)) {
            return false;
        }
        responses |= ((threshold & 0x80) >> i) | (((~threshold & 0x40) >> 6) << i);
    }

    // Reset to default
    if (!write_reg(kRegNameMagneticThreshold, (0b000 << 5) | (0b111 << 2))) {
        return false;
    }

    initialized_ = was_initialized;

    // The overlap between the intervals stems from the device's hysteresis.
    switch (responses) {
        case 0x00: return 0.0f; // below 26 mT
        case 0x01: return 30e-3; // [20 mT, 41 mT]
        case 0x03: return 45e-3; // [35 mT, 56 mT]
        case 0x07: return 60e-3; // [50 mT, 70 mT]
        case 0x0f: return 74e-3; // [64 mT, 84 mT]
        case 0x1f: return 88e-3; // [78 mT, 98 mT]
        case 0x3f: return 102e-3; // [92 mT, 112 mT]
        case 0x7f: return 116e-3; // [106 mT, 126 mT]
        case 0xff: return std::numeric_limits<float>::infinity(); // above 120 mT
        default: return std::nullopt; // inconsistent responses
    }
}

bool Ma732::read_reg(RegName reg_name, uint8_t* value) {
    // Do blocking write
    tx_buf_ = 0x4000 | ((uint16_t)reg_name << 8);
    if (!spi_arbiter_.transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }

    osDelay(1); // nCS must be deasserted for a short while. Can probably be shorter.

    tx_buf_ = 0x0000;
    rx_buf_ = 0xffff;
    if (!spi_arbiter_.transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000)) {
        return false;
    }

    if ((rx_buf_ & 0xff) != 0) { // lower byte always expected to be 0
        return false;
    }

    if (value) {
        *value = (uint8_t)(rx_buf_ >> 8);
    }

    return true;
}
/*
bool Ma732::write_reg(RegName reg_name, uint8_t value) {
    // Do blocking write
    buf2_[0] = 0x8000 | ((uint16_t)reg_name << 8) | value;
    buf2_[1] = 0;
    buf3_[0] = 0xffff;
    buf3_[1] = 0xffff;

    rx_buf_ = 0xffff;
    if (!spi_arbiter_.transfer(spi_config_, ncs_gpio_, (uint8_t *)(&buf2_), (uint8_t *)(&buf3_), nullptr, 1, 1000)) {
        return false;
    }

    osDelay(21); // 20ms wait required according to datasheet

    tx_buf_ = 0x0000;
    rx_buf_ = 0xffff;
    if (!spi_arbiter_.transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000)) {
        return false;
    }

    if (rx_buf_ != ((uint16_t)value << 8)) { // verify read-back
        return false;
    }

    return true;
}*/


bool Ma732::write_reg(RegName reg_name, uint8_t value) {
    tx_buf_ = 0x8000 | ((uint16_t)reg_name << 8) | value;
    if (!spi_arbiter_.transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }

    osDelay(21); // 20ms wait required according to datasheet

    tx_buf_ = 0x0000;
    rx_buf_ = 0xffff;
    if (!spi_arbiter_.transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000)) {
        return false;
    }

    if (rx_buf_ != ((uint16_t)value << 8)) { // verify read-back
        return false;
    }

    return true;
}
