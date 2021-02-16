#ifndef __BOARD_SUPPORT_PACKAGE_HPP
#define __BOARD_SUPPORT_PACKAGE_HPP

#include <interfaces/canbus.hpp>
#include <interfaces/file.hpp>

// TODO: move
template<typename T>
struct expanded {
    constexpr expanded(const T& val) : val_{val} {}
    T val_;

    template<std::size_t ... Is>
    constexpr std::array<T, sizeof...(Is)> expand(std::index_sequence<Is...>) {
        return {{(static_cast<void>(Is), val_)...}};
    }

    template<std::size_t N>
    constexpr operator std::array<T, N>() {
        return expand(std::make_index_sequence<N>());
    }
};

template<typename BoardTraits>
struct BoardSupportPackageBase {
    static constexpr const float kAdcMaxVoltage = 3.3f;
    static constexpr const unsigned _GPIO_COUNT = BoardTraits::_GPIO_COUNT;
    static constexpr const unsigned UART_COUNT = BoardTraits::UART_COUNT;
    static constexpr const unsigned CANBUS_COUNT = BoardTraits::CANBUS_COUNT;
    static constexpr const unsigned SPI_COUNT = BoardTraits::SPI_COUNT;
    static constexpr const unsigned INC_ENC_COUNT = BoardTraits::INC_ENC_COUNT;

    /**
     * @brief Array of user facing GPIOs.
     * 
     * A GPIO can be Stm32Gpio::none if it's not present or not connected on the
     * board.
     */
    static const std::array<Stm32Gpio, _GPIO_COUNT> gpios;

    /**
     * @brief Array of UART interfaces.
     */
    static const std::array<Stm32Usart*, UART_COUNT> uarts;

    /**
     * @brief Array of CAN bus interfaces.
     */
    static const std::array<CanBusBase*, CANBUS_COUNT> can_busses;

    /**
     * @brief Non volatile storage.
     * 
     * The maximum capacity of this file is board specific.
     */
    static File& nvm;

    /**
     * @brief PWM output to control the FAN speed.
     * 
     * null on 
     */
    static PwmOutputGroup<1>* fan_output;

    /**
     * @brief Continuously updated measurement of the DC link voltage in Volts.
     * 
     * If the value cannot be measured, it is set to 0.
     */
    float vbus_voltage = 0.0f;

    /**
     * @brief Array of most recent measured GPIO ADC values.
     * 
     * A value of 0.0f represents the lowest measurable voltage (0V).
     * A value of 1.0f represents the highest measurable voltage (
     * `kAdcMaxVoltage` V).
     * 
     * A value of NAN indicates that the ADC measurement is not available, for
     * example because the GPIO doesn't support it.
     * 
     * The sampling time and frequency is board specific.
     */
    std::array<float, _GPIO_COUNT> gpio_adc_values = expanded(NAN);

    /**
     * @brief Array of most recent measured GPIO PWM input values.
     * 
     * A value of 0.0f represents the lowest measurable duty cycle.
     * A value of 1.0f represents the highest measurable duty cycle.
     * 
     * A value of NAN indicates that the PWM measurement is not available,
     * for example because the GPIO doesn't support it.
     * 
     * The sampling time and frequency is board specific.
     */
    std::array<float, _GPIO_COUNT> gpio_pwm_values = expanded(NAN);

    /**
     * @brief An array of the most recent motor FET thermistor values.
     * 
     * Each value represents a temperature in °C.
     * The sampling time and frequency is board specific.
     * NAN is used to represent failed measurements.
     */
    std::array<float, AXIS_COUNT> motor_fet_temperatures = expanded(NAN);

    /**
     * @brief The most recent temperature of the brake FET thermistor in °C.
     * 
     * The sampling time and frequency is board specific.
     * NAN is used to represent failed measurements.
     */
    float brake_fet_thermistor = NAN;

    struct UartConfig {
        bool enabled = false;
        uint32_t baudrate = 0;
        int rx_gpio = -1;
        int tx_gpio = -1;
    };

    struct CanConfig {
        bool enabled = false;
        int r_gpio = -1;
        int d_gpio = -1;
    };

    struct IncEncConfig {
        bool enabled = false;
        int a_gpio = -1;
        int b_gpio = -1;
    };

    struct SpiConfig {
        bool enabled = false;
        int mosi_gpio = -1;
        int miso_gpio = -1;
        int sck_gpio = -1;
    };

    enum GpioMode {
        kAnalogInput, // must remain the first (default) item
        kPwmInput,
        kDigitalInputNoPull,
        kDigitalInputPullUp,
        kDigitalInputPullDown,
        kDigitalOutput,
        kAlternateFunction // UART, CAN, SPI, I2C, ...
    };

    struct BoardConfig {
        std::array<GpioMode, _GPIO_COUNT> gpio_modes = {};
        std::array<UartConfig, UART_COUNT> uart_config = {};
        std::array<CanConfig, CANBUS_COUNT> can_config = {};
        std::array<SpiConfig, SPI_COUNT> spi_config = {};
        std::array<IncEncConfig, INC_ENC_COUNT> inc_enc_config = {};
    };

    /**
     * @brief Initializes low level board functions such as clocks and ADCs.
     * 
     * Must be called early during startup.
     */
    bool init();

    /**
     * @brief Initializes the selected peripherals on the device.
     * 
     * Currently it is only allowed to call this function once after startup.
     * 
     * Functions that are hardwired to non-GPIO pins (e.g. SPI MISO/MOSI/SCK)
     * must be set to GPIO number -1.
     * 
     * On misconfiguration the function returns false and does not reconfigure
     * anything. This includes:
     *  - Use of the same GPIO by multiple peripherals.
     *  - Setting a GPIO to a mode that it does not support.
     *  - Assigning a function (e.g. UART_A TX) to a GPIO that does not support
     *    it or does not exist.
     *  - Disabling a peripheral that is needed for essential onboard features
     *    (e.g. SPI on ODrive v3.x).
     *  - Attempt to enable SPI on ODrive v4.x without enabling SCK (not
     *    supported by hardware).
     *  - Attempt to enable CAN on ODrive v3.x without enabling the
     *    corresponding CAN_R and CAN_D GPIOs.
     */
    bool config(const BoardConfig& config);

    static const int uart_rx_gpios[UART_COUNT];
    static const int uart_tx_gpios[UART_COUNT];
    static const int can_r_gpios[CANBUS_COUNT];
    static const int can_d_gpios[CANBUS_COUNT];
    static const int spi_mosi_gpios[SPI_COUNT];
    static const int spi_miso_gpios[SPI_COUNT];
    static const int spi_sck_gpios[SPI_COUNT];
    static const int inc_enc_a_gpios[INC_ENC_COUNT];
    static const int inc_enc_b_gpios[INC_ENC_COUNT];

private:
    bool validate_gpios(const BoardConfig& config) {
        std::array<int, _GPIO_COUNT> use_count = {};
        bool bad_use = false;

        auto inc_use_count = [&](bool enabled, int gpio, int legal_gpio) {
            if (enabled && (gpio >= 0)) {
                if (gpio == legal_gpio) {
                    use_count[gpio]++;
                } else {
                    bad_use = true;
                }
            }
        };

        for (size_t i = 0; i < _GPIO_COUNT; ++i) {
            inc_use_count(config.gpio_modes[i] != kAlternateFunction, i, i);
        }

        for (size_t i = 0; i < UART_COUNT; ++i) {
            auto& cfg = config.uart_config[i];
            inc_use_count(cfg.enabled, cfg.rx_gpio, uart_rx_gpios[i]);
            inc_use_count(cfg.enabled, cfg.tx_gpio, uart_tx_gpios[i]);
        }

        for (size_t i = 0; i < CANBUS_COUNT; ++i) {
            auto& cfg = config.can_config[i];
            inc_use_count(cfg.enabled, cfg.r_gpio, can_r_gpios[i]);
            inc_use_count(cfg.enabled, cfg.d_gpio, can_d_gpios[i]);
        }

        for (size_t i = 0; i < SPI_COUNT; ++i) {
            auto& cfg = config.spi_config[i];
            inc_use_count(cfg.enabled, cfg.miso_gpio, spi_miso_gpios[i]);
            inc_use_count(cfg.enabled, cfg.mosi_gpio, spi_mosi_gpios[i]);
            inc_use_count(cfg.enabled, cfg.sck_gpio, spi_sck_gpios[i]);
        }

        for (size_t i = 0; i < INC_ENC_COUNT; ++i) {
            auto& cfg = config.inc_enc_config[i];
            inc_use_count(cfg.enabled, cfg.a_gpio, inc_enc_a_gpios[i]);
            inc_use_count(cfg.enabled, cfg.b_gpio, inc_enc_b_gpios[i]);
        }

        for (auto& cnt: use_count) {
            if (cnt > 1) {
                return false; // at least one GPIO was used multiple times
            }
        }

        if (bad_use) {
            return false; // at least one GPIO number was out of range or didn't match the expected GPIO
        }

        return true;
    }
};

#endif // __BOARD_SUPPORT_PACKAGE_HPP
