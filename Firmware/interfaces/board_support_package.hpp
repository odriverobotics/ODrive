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

    /**
     * @brief Array of user facing GPIOs.
     * 
     * A GPIO can be Stm32Gpio::none if it's not present or not connected on the
     * board.
     */
    static const std::array<Stm32Gpio, BoardTraits::_GPIO_COUNT> gpios;

    /**
     * @brief Array of CAN bus interfaces.
     */
    static const std::array<CanBusBase*, BoardTraits::_CANBUS_COUNT> can_busses;

    /**
     * @brief Non volatile storage.
     * 
     * The maximum capacity of this file is board specific.
     */
    static File& nvm;

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
     * A value of -INFINITY indicates that the ADC measurement is not available, for
     * example because the GPIO doesn't support it.
     * 
     * The sampling time and frequency is board specific.
     */
    std::array<float, BoardTraits::_GPIO_COUNT> gpio_adc_values = expanded(-INFINITY);

    /**
     * @brief Array of most recent measured GPIO PWM input values.
     * 
     * A value of 0.0f represents the lowest measurable duty cycle.
     * A value of 1.0f represents the highest measurable duty cycle.
     * 
     * A value of -INFINITY indicates that the PWM measurement is not available,
     * for example because the GPIO doesn't support it.
     * 
     * The sampling time and frequency is board specific.
     */
    std::array<float, BoardTraits::_GPIO_COUNT> gpio_pwm_values = expanded(-INFINITY);

    /**
     * @brief An array of the most recent motor FET thermistor values.
     * 
     * Each value represents a temperature in °C.
     * The sampling time and frequency is board specific.
     * -INFINITY is used to represent failed measurements.
     */
    std::array<float, AXIS_COUNT> motor_fet_temperatures = expanded(-INFINITY);

    /**
     * @brief The most recent temperature of the brake FET thermistor in °C.
     * 
     * The sampling time and frequency is board specific.
     * -INFINITY is used to represent failed measurements.
     */
    float brake_fet_thermistor = -INFINITY;
};

#endif // __BOARD_SUPPORT_PACKAGE_HPP
