#ifndef __MA732_HPP
#define __MA732_HPP

#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <optional>

class Ma732 {
public:
    Ma732(Stm32SpiArbiter& spi_arbiter, Stm32Gpio ncs_gpio);

    /**
     * @brief Initializes the MA732 encoder.
     *
     * This checks if the encoder is configured as desired and loads the
     * appropriate configuration otherwise.
     *
     * This function is blocking (on SPI I/O) and must be run in thread context.
     *
     * @returns false if the desired configuration could not be applied.
     */
    bool init();

    /**
     * @brief Measures the magnetic field strength seen by the device.
     *
     * This function is blocking, it takes around 90ms.
     *
     * @returns: The field strength in Tesla or std::nullopt if the measurement
     *           failed (e.g. because of an SPI communication error or because
     *           the value lies outside the measurable range or the algorithm
     *           detected inconsistent results).
     */
    std::optional<float> get_field_strength();

private:
    enum RegName {
        kRegNameZeroLow = 0x0,
        kRegNameZeroHigh = 0x1,
        kRegNameBiasCurrentTrimming = 0x2,
        kRegNameEnableTrimming = 0x3,
        kRegNamePptIlip = 0x4,
        kRegNamePulsesPerTurn = 0x5,
        kRegNameMagneticThreshold = 0x6,
        // might there be undocumented registers here?
        kRegNameReverseDir = 0x9,
        kRegNameFilterWindow = 0xE,
        kRegNameHysteresis = 0x10,
        kRegNameThresholdStatus = 0x1B,
    };

    struct RegisterFile {
        bool operator==(const RegisterFile& other);
        bool operator!=(const RegisterFile& other) { return !(*this == other); }

        uint8_t zero_pos_low;
        uint8_t zero_pos_high;
        uint8_t bias_current_trimming;
        uint8_t enable_trimming;
        uint8_t ppt_ilip;
        uint8_t pulses_per_turn;
        uint8_t magnetic_threshold;
        uint8_t reverse_dir;
        uint8_t filter_window;
        uint8_t hysteresis;
    };

    /**
     * @brief Writes data to a MA732 register.
     *
     * The result is read back and the function returns false if the result
     * doesn't match the written value.
     */
    bool write_reg(RegName reg_name, uint8_t value);

    /**
     * @brief Reads data from a MA732 register.
     * @returns: True on success, false otherwise.
     */
    bool read_reg(RegName reg_name, uint8_t* value);

    Stm32SpiArbiter& spi_arbiter_;
    Stm32Gpio ncs_gpio_;

    bool initialized_ = false;
    uint16_t tx_buf_;
    uint16_t rx_buf_;
};

#endif // __MA732_HPP
