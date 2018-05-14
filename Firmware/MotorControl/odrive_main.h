#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// STM specific includes
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4 // TODO: might change in future board versions
#include <arm_math.h>

// OS includes
#include <cmsis_os.h>

// Hardware configuration
#if HW_VERSION_MAJOR == 3
#include "board_config_v3.h"
#else
#error "unknown board version"
#endif

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

static const float current_meas_period = CURRENT_MEAS_PERIOD;
static const int current_meas_hz = CURRENT_MEAS_HZ;
extern float vbus_voltage;
extern bool brake_resistor_armed_;
extern const float elec_rad_per_enc;
extern uint32_t _reboot_cookie;
extern bool user_config_loaded_;

extern uint64_t serial_number;
extern char serial_number_str[13];

typedef struct {
    bool fully_booted;
    uint32_t uptime; // [ms]
    uint32_t min_heap_space; // FreeRTOS heap [Bytes]
    uint32_t min_stack_space_axis0; // minimum remaining space since startup [Bytes]
    uint32_t min_stack_space_axis1;
    uint32_t min_stack_space_comms;
    uint32_t min_stack_space_usb;
    uint32_t min_stack_space_uart;
    uint32_t min_stack_space_usb_irq;
    uint32_t min_stack_space_startup;
} SystemStats_t;
extern SystemStats_t system_stats_;

#ifdef __cplusplus
}

// @brief general user configurable board configuration
struct BoardConfig_t {
    bool enable_uart = true;
    bool enable_ascii_protocol_on_usb = false;
    float brake_resistance = 0.47f;     // [ohm]
    float dc_bus_undervoltage_trip_level = 8.0f;                        //<! [V] minimum voltage below which the motor stops operating
    float dc_bus_overvoltage_trip_level = 1.08f * HW_VERSION_VOLTAGE;   //<! [V] maximum voltage above which the motor stops operating.
                                                                        //<! This protects against cases in which the power supply fails to dissipate
                                                                        //<! the brake power if the brake resistor is disabled.
                                                                        //<! The default is 26V for the 24V board version and 52V for the 48V board version.
};
extern BoardConfig_t board_config;
extern bool user_config_loaded_;

class Axis;
class Motor;

constexpr size_t AXIS_COUNT = 2;
extern Axis *axes[AXIS_COUNT];

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 128
extern float oscilloscope[OSCILLOSCOPE_SIZE];
extern size_t oscilloscope_pos;

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


// ODrive specific includes
#include <communication/protocol.hpp>
#include <utils.h>
#include <low_level.h>
#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <controller.hpp>
#include <motor.hpp>
#include <axis.hpp>
#include <communication/communication.h>

#endif // __cplusplus


// general system functions defined in main.cpp
void save_configuration(void);
void erase_configuration(void);
void enter_dfu_mode(void);

#endif /* __ODRIVE_MAIN_H */
