#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

// Hardware configuration
#include <board.h>

#ifdef __cplusplus
#include <fibre/protocol.hpp>
#include <communication/interface_usb.h>
#include <communication/interface_i2c.h>
#include <communication/interface_uart.h>
#include <task_timer.hpp>
extern "C" {
#endif

// OS includes
#include <cmsis_os.h>

// extern const float elec_rad_per_enc;
extern uint32_t _reboot_cookie;

extern uint64_t serial_number;
extern char serial_number_str[13];

#ifdef __cplusplus
}

typedef struct {
    bool fully_booted;
    uint32_t uptime; // [ms]
    uint32_t min_heap_space; // FreeRTOS heap [Bytes]
    uint32_t max_stack_usage_axis; // minimum remaining space since startup [Bytes]
    uint32_t max_stack_usage_usb;
    uint32_t max_stack_usage_uart;
    uint32_t max_stack_usage_startup;
    uint32_t max_stack_usage_can;

    uint32_t stack_size_axis;
    uint32_t stack_size_usb;
    uint32_t stack_size_uart;
    uint32_t stack_size_startup;
    uint32_t stack_size_can;

    int32_t prio_axis;
    int32_t prio_usb;
    int32_t prio_uart;
    int32_t prio_startup;
    int32_t prio_can;

    USBStats_t& usb = usb_stats_;
    I2CStats_t& i2c = i2c_stats_;
} SystemStats_t;

struct PWMMapping_t {
    endpoint_ref_t endpoint;
    float min = 0;
    float max = 0;
};

// @brief general user configurable board configuration
struct BoardConfig_t {
    ODriveIntf::GpioMode gpio_modes[GPIO_COUNT] = {
        DEFAULT_GPIO_MODES
    };

    bool enable_uart_a = true;
    bool enable_uart_b = false;
    bool enable_uart_c = false;
    uint32_t uart_a_baudrate = 115200;
    uint32_t uart_b_baudrate = 115200;
    uint32_t uart_c_baudrate = 115200;
    bool enable_can_a = true;
    bool enable_i2c_a = false;
    bool enable_ascii_protocol_on_usb = true;
    float max_regen_current = 0.0f;
    float brake_resistance = DEFAULT_BRAKE_RESISTANCE;
    float dc_bus_undervoltage_trip_level = 8.0f;                        //<! [V] minimum voltage below which the motor stops operating
    float dc_bus_overvoltage_trip_level = 1.07f * HW_VERSION_VOLTAGE;   //<! [V] maximum voltage above which the motor stops operating.
                                                                        //<! This protects against cases in which the power supply fails to dissipate
                                                                        //<! the brake power if the brake resistor is disabled.
                                                                        //<! The default is 26V for the 24V board version and 52V for the 48V board version.

    /**
     * If enabled, if the measured DC voltage exceeds `dc_bus_overvoltage_ramp_start`,
     * the ODrive will sink more power than usual into the the brake resistor
     * in an attempt to bring the voltage down again.
     * 
     * The brake duty cycle is increased by the following amount:
     *  vbus_voltage == dc_bus_overvoltage_ramp_start  =>  brake_duty_cycle += 0%
     *  vbus_voltage == dc_bus_overvoltage_ramp_end  =>  brake_duty_cycle += 100%
     * 
     * Remarks:
     *  - This feature is active even when all motors are disarmed.
     *  - This feature is disabled if `brake_resistance` is non-positive.
     */
    bool enable_dc_bus_overvoltage_ramp = false;
    float dc_bus_overvoltage_ramp_start = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                      //!< Do not set this lower than your usual vbus_voltage,
                                                                      //!< unless you like fried brake resistors.
    float dc_bus_overvoltage_ramp_end = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                    //!< Must be larger than `dc_bus_overvoltage_ramp_start`,
                                                                    //!< otherwise the ramp feature is disabled.

    float dc_max_positive_current = INFINITY; // Max current [A] the power supply can source
    float dc_max_negative_current = -0.000001f; // Max current [A] the power supply can sink. You most likely want a non-positive value here. Set to -INFINITY to disable.
    PWMMapping_t pwm_mappings[4];
    PWMMapping_t analog_mappings[GPIO_COUNT];
};

struct TaskTimes {
    TaskTimer sampling;
    TaskTimer control_loop_misc;
    TaskTimer control_loop_checks;
};


// Forward Declarations
class Axis;
class Motor;
class ODriveCAN;

extern ODriveCAN *odCAN;

// TODO: move
// this is technically not thread-safe but practically it might be
#define DEFINE_ENUM_FLAG_OPERATORS(ENUMTYPE) \
inline ENUMTYPE operator | (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) | static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator & (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) & static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator ^ (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) ^ static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator |= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) |= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator &= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) &= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator ^= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) ^= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator ~ (ENUMTYPE a) { return static_cast<ENUMTYPE>(~static_cast<std::underlying_type_t<ENUMTYPE>>(a)); }

#include "autogen/interfaces.hpp"

// ODrive specific includes
#include <utils.hpp>
#include <low_level.h>
#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <controller.hpp>
#include <current_limiter.hpp>
#include <thermistor.hpp>
#include <trapTraj.hpp>
#include <endstop.hpp>
#include <mechanical_brake.hpp>
#include <axis.hpp>
#include <oscilloscope.hpp>
#include <communication/communication.h>

// Defined in autogen/version.c based on git-derived version numbers
extern "C" {
extern const unsigned char fw_version_major_;
extern const unsigned char fw_version_minor_;
extern const unsigned char fw_version_revision_;
extern const unsigned char fw_version_unreleased_;
}

static Stm32Gpio get_gpio(size_t gpio_num) {
    return (gpio_num < GPIO_COUNT) ? gpios[gpio_num] : GPIO_COUNT ? gpios[0] : Stm32Gpio::none;
}

// general system functions defined in main.cpp
class ODrive : public ODriveIntf {
public:
    void save_configuration() override;
    void erase_configuration() override;
    void reboot() override { NVIC_SystemReset(); }
    void enter_dfu_mode() override;
    void clear_errors() override;

    float get_adc_voltage(uint32_t gpio) override {
        return ::get_adc_voltage(get_gpio(gpio));
    }

    int32_t test_function(int32_t delta) override {
        static int cnt = 0;
        return cnt += delta;
    }

    void do_fast_checks();
    void sampling_cb();
    void control_loop_cb(uint32_t timestamp);

    Axis& get_axis(int num) { return axes[num]; }
    ODriveCAN& get_can() { return *odCAN; }

    uint32_t get_interrupt_status(int32_t irqn);
    uint32_t get_dma_status(uint8_t stream_num);
    void disarm_with_error(Error error);

    Error error_ = ERROR_NONE;
    float& vbus_voltage_ = ::vbus_voltage; // TODO: make this the actual variable
    float& ibus_ = ::ibus_; // TODO: make this the actual variable
    float ibus_report_filter_k_ = 1.0f;

    const uint64_t& serial_number_ = ::serial_number;

#if defined(STM32F405xx)
    // Determine start address of the OTP struct:
    // The OTP is organized into 16-byte blocks.
    // If the first block starts with "0xfe" we use the first block.
    // If the first block starts with "0x00" and the second block starts with "0xfe",
    // we use the second block. This gives the user the chance to screw up once.
    // If none of the above is the case, we consider the OTP invalid (otp_ptr will be NULL).
    const uint8_t* otp_ptr =
        (*(uint8_t*)FLASH_OTP_BASE == 0xfe) ? (uint8_t*)FLASH_OTP_BASE :
        (*(uint8_t*)FLASH_OTP_BASE != 0x00) ? NULL :
            (*(uint8_t*)(FLASH_OTP_BASE + 0x10) != 0xfe) ? NULL :
                (uint8_t*)(FLASH_OTP_BASE + 0x10);

    // Read hardware version from OTP if available, otherwise fall back
    // to software defined version.
    const uint8_t hw_version_major_ = otp_ptr ? otp_ptr[3] : HW_VERSION_MAJOR;
    const uint8_t hw_version_minor_ = otp_ptr ? otp_ptr[4] : HW_VERSION_MINOR;
    const uint8_t hw_version_variant_ = otp_ptr ? otp_ptr[5] : HW_VERSION_VOLTAGE;
#else
#error "not implemented"
#endif

    // the corresponding macros are defined in the autogenerated version.h
    const uint8_t fw_version_major_ = ::fw_version_major_;
    const uint8_t fw_version_minor_ = ::fw_version_minor_;
    const uint8_t fw_version_revision_ = ::fw_version_revision_;
    const uint8_t fw_version_unreleased_ = ::fw_version_unreleased_; // 0 for official releases, 1 otherwise

    bool& brake_resistor_armed_ = ::brake_resistor_armed; // TODO: make this the actual variable
    bool& brake_resistor_saturated_ = ::brake_resistor_saturated; // TODO: make this the actual variable

    SystemStats_t system_stats_;
    Oscilloscope oscilloscope_{
        &axes[0].motor_.current_control_.v_current_control_integral_d_, // trigger_src
        0.5f, // trigger_threshold
        nullptr // &axes[0].motor_.current_control_.Ialpha_measured_ // data_src TODO: change data type
    };

    BoardConfig_t config_;
    uint32_t user_config_loaded_ = 0;
    bool misconfigured_ = false;

    uint32_t test_property_ = 0;

    uint32_t last_update_timestamp_ = 0;
    uint32_t n_evt_sampling_ = 0;
    uint32_t n_evt_control_loop_ = 0;
    bool task_timers_armed_ = false;
    TaskTimes task_times_;
};

extern ODrive odrv; // defined in main.cpp

#endif // __cplusplus

#endif /* __ODRIVE_MAIN_H */
