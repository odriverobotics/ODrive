/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include <stdbool.h>

// STM specific includes
#include <stm32f4xx_hal.h>
#include <tim.h>
#include <i2c.h>
#include <usbd_def.h>
#include <main.h>
#include "cmsis_os.h"

#include <arm_math.h>

#include <Drivers/STM32/stm32_system.h>

#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (500e-6f)
#endif

#define AXIS_COUNT (2)

// Total count of GPIOs, including encoder pins, CAN pins and a dummy GPIO0.
// ODrive v3.4 and earlier don't have GPIOs 6, 7 and 8 but to keep the numbering
// consistent we just leave a gap in the counting scheme.
#define GPIO_COUNT  (17)

#define CANBUS_COUNT (1)

#if HW_VERSION_MINOR >= 5 && HW_VERSION_VOLTAGE >= 48
#define DEFAULT_BRAKE_RESISTANCE (2.0f) // [ohm]
#else
#define DEFAULT_BRAKE_RESISTANCE (0.47f) // [ohm]
#endif

#define DEFAULT_ERROR_PIN 0
#define DEFAULT_MIN_DC_VOLTAGE 8.0f

#define DEFAULT_GPIO_MODES \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_UART_A, \
    ODriveIntf::GPIO_MODE_UART_A, \
    ODriveIntf::GPIO_MODE_ANALOG_IN, \
    ODriveIntf::GPIO_MODE_ANALOG_IN, \
    ODriveIntf::GPIO_MODE_ANALOG_IN, \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_DIGITAL, \
    ODriveIntf::GPIO_MODE_ENC0, \
    ODriveIntf::GPIO_MODE_ENC0, \
    ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN, \
    ODriveIntf::GPIO_MODE_ENC1, \
    ODriveIntf::GPIO_MODE_ENC1, \
    ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN, \
    ODriveIntf::GPIO_MODE_CAN_A, \
    ODriveIntf::GPIO_MODE_CAN_A,

#define TIM_TIME_BASE TIM14

// Run control loop at the same frequency as the current measurements.
#define CONTROL_TIMER_PERIOD_TICKS  (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1))

#define TIM1_INIT_COUNT (TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128) // TODO: explain why this offset

// The delta from the control loop timestamp to the current sense timestamp is
// exactly 0 for M0 and TIM1_INIT_COUNT for M1.
#define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA (TIM_1_8_PERIOD_CLOCKS / 2 + 1 * 128)

#ifdef __cplusplus
#include <Drivers/DRV8301/drv8301.hpp>
#include <Drivers/STM32/stm32_gpio.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <Drivers/STM32/stm32_usart.hpp>
#include <interfaces/canbus.hpp>
#include <interfaces/pwm_output_group.hpp>
#include <MotorControl/thermistor.hpp>

using TGateDriver = Drv8301;
using TOpAmp = Drv8301;

#include <MotorControl/motor.hpp>
#include <MotorControl/encoder.hpp>


#include <interfaces/board_support_package.hpp>

struct BoardTraits {
    constexpr static const unsigned _AXIS_COUNT = AXIS_COUNT;
    constexpr static const unsigned _GPIO_COUNT = GPIO_COUNT;
    constexpr static const unsigned _CANBUS_COUNT = CANBUS_COUNT;
};
using BoardSupportPackage = BoardSupportPackageBase<BoardTraits>;

extern BoardSupportPackage board;


extern std::array<Axis, AXIS_COUNT> axes;
extern Motor motors[AXIS_COUNT];
extern Encoder encoders[AXIS_COUNT];

struct GpioFunction { int mode = 0; uint8_t alternate_function = 0xff; };
extern std::array<GpioFunction, 3> alternate_functions[GPIO_COUNT];

extern USBD_HandleTypeDef usb_dev_handle;

extern Stm32SpiArbiter& ext_spi_arbiter;

extern Stm32Usart uart_a;
extern Stm32Usart uart_b;
extern Stm32Usart uart_c;

extern PwmOutputGroup<1>& brake_resistor_output;

// Points to a counter that resets at the main control loop frequency. The unit
// of this counter is an implementation detail. The period is indicated by
// `board_control_loop_counter_period`.
extern volatile uint32_t& board_control_loop_counter;
extern uint32_t board_control_loop_counter_period;
#endif

// Period in [s]
#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )
static const int current_meas_hz = CURRENT_MEAS_HZ;

// This board has no board-specific user configurations
static inline bool board_read_config() { return true; }
static inline bool board_write_config() { return true; }
static inline void board_clear_config() { }
static inline bool board_apply_config() { return true; }

/**
 * @brief Must be called very early during startup to start the clocks
 */
bool board_init_stage_0();

/**
 * @brief Must be called after configuration is loaded and GPIOs are initialized.
 */
bool board_init_stage_1();

void start_timers();

#endif // __BOARD_CONFIG_H
