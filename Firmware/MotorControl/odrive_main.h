#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

// Note on central include scheme by Samuel:
// there are circular dependencies between some of the header files,
// e.g. the Motor header needs a forward declaration of Axis and vice versa
// so I figured I'd make one main header that takes care of
// the forward declarations and right ordering
// btw this pattern is not so uncommon, for instance IIRC the stdlib uses it too

/* C includes ----------------------------------------------------------------*/

#include <stdbool.h>

// STM specific includes
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4 // TODO: might change in future board versions
#include <arm_math.h>

// OS includes
#include <cmsis_os.h>

#include <stm32_system.h>

/* C++ includes --------------------------------------------------------------*/

#ifdef __cplusplus

#include <fibre/protocol.hpp>
#include <stm32_usb.hpp> // TODO: replace with generic header
#include <stm32_usart.hpp> // TODO: replace with generic header

#endif // __cplusplus

/* Exported Declarations -----------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

// TODO: move to board definition file
#define TIM_APB1_CLOCK_HZ 84000000
#define TIM_APB1_PERIOD_CLOCKS 4096
#define TIM_APB1_DEADTIME_CLOCKS 40

#define MAX_MODULATION 0.80f // accounts for dead-time and the likes
#define V_DC_TO_V_PHASE_MAX (MAX_MODULATION * 0.57735026919f)

//#define NVIC_PRIO_M0                    configMAX_SYSCALL_INTERRUPT_PRIORITY - (1 << (8 - configPRIO_BITS))
//#define NVIC_PRIO_M1                    configMAX_SYSCALL_INTERRUPT_PRIORITY - (1 << (8 - configPRIO_BITS)) + 1
#define NVIC_PRIO_M0            (configMAX_SYSCALL_INTERRUPT_PRIORITY)
#define NVIC_PRIO_M1            (configMAX_SYSCALL_INTERRUPT_PRIORITY + (1 << (8 - configPRIO_BITS)))

#define NVIC_PRIO_UART          (configMAX_SYSCALL_INTERRUPT_PRIORITY + (2 << (8 - configPRIO_BITS)))
#define NVIC_PRIO_USB           (configMAX_SYSCALL_INTERRUPT_PRIORITY + (2 << (8 - configPRIO_BITS)))
#define NVIC_PRIO_SPI           (configMAX_SYSCALL_INTERRUPT_PRIORITY + (2 << (8 - configPRIO_BITS)))
#define NVIC_PRIO_I2C           (configMAX_SYSCALL_INTERRUPT_PRIORITY + (2 << (8 - configPRIO_BITS)))
#define NVIC_PRIO_CAN           (configMAX_SYSCALL_INTERRUPT_PRIORITY + (2 << (8 - configPRIO_BITS)))

// very low priority (it's ok for this one to be delayed up to ~25ms)
#define NVIC_PRIO_TICK_TIMER    (255)       // updates the tick count (TODO: use chained timers to do this without CPU)

//#define NVIC_PRIO_TASK_SWITCHER 255     //

#define RTOS_PRIO_USB osPriorityAboveNormal
#define RTOS_PRIO_COMM osPriorityNormal
#define RTOS_PRIO_UART osPriorityNormal
#define RTOS_PRIO_ANALOG_IN osPriorityLow
#define RTOS_PRIO_MAIN_TASK osPriorityIdle
#define RTOS_PRIO_AXIS0 (osPriority)(osPriorityHigh + (osPriority)1)
#define RTOS_PRIO_AXIS1 osPriorityHigh
//#define RTOS_PRIO_AXIS0 osPriorityRealtime
//#define RTOS_PRIO_AXIS1 osPriorityRealtime

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
    uint32_t boot_progress;
    float adc_irq_usage = 0.0f;
    float dma2_stream1_irq_usage = 0.0f;
    float dma2_stream2_irq_usage = 0.0f;
    float tim1_up_usage = 0.0f;
    float tim8_up_usage = 0.0f;
} SystemStats_t;
extern SystemStats_t system_stats_;

#ifdef __cplusplus
}

/* C++ declarations ----------------------------------------------------------*/

struct PWMMapping_t {
    endpoint_ref_t endpoint = { 0 };
    float min = 0;
    float max = 0;
};


#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR <= 4
#define GPIO_COUNT  5
#else
#define GPIO_COUNT  8
#endif

// @brief general user configurable board configuration
struct BoardConfig_t {
    bool enable_uart = true;
    bool enable_i2c_instead_of_can = false;
    bool enable_ascii_protocol_on_usb = true;
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5 && HW_VERSION_VOLTAGE >= 48
    float brake_resistance = 2.0f;     // [ohm]
#else
    float brake_resistance = 0.47f;     // [ohm]
#endif
    float dc_bus_undervoltage_trip_level = 8.0f;                        //<! [V] minimum voltage below which the motor stops operating
    float dc_bus_overvoltage_trip_level = 1.07f * HW_VERSION_VOLTAGE;   //<! [V] maximum voltage above which the motor stops operating.
                                                                        //<! This protects against cases in which the power supply fails to dissipate
                                                                        //<! the brake power if the brake resistor is disabled.
                                                                        //<! The default is 26V for the 24V board version and 52V for the 48V board version.
    PWMMapping_t pwm_mappings[GPIO_COUNT];
    PWMMapping_t analog_mappings[GPIO_COUNT];
};
extern BoardConfig_t board_config;
extern bool user_config_loaded_;

extern STM32_USART_t* comm_uart;
extern USB_t* comm_usb;
extern STM32_USBTxEndpoint_t cdc_tx_endpoint;
extern STM32_USBRxEndpoint_t cdc_rx_endpoint;
extern STM32_USBIntEndpoint_t cdc_cmd_endpoint;
extern STM32_USBTxEndpoint_t odrive_tx_endpoint;
extern STM32_USBRxEndpoint_t odrive_rx_endpoint;
#include "devices.hpp"
extern VoltageDivider_t vbus_sense;


extern STM32_GPIO_t* gpios[];
extern const size_t num_gpios;

// Sampling port A,B,C (coherent with current meas timing)
// TODO: make more flexible
static constexpr const GPIO_TypeDef* GPIOs_to_samp[] = { GPIOA, GPIOB, GPIOC };
static constexpr const int n_GPIO_samples = sizeof(GPIOs_to_samp) / sizeof(GPIOs_to_samp[0]);

class Axis;
class Motor;

extern Axis* axes;
extern const size_t n_axes;

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

#include <utils.h>
#include <low_level.h>
#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <async_estimator.hpp>
#include <controller.hpp>
#include <motor.hpp>
#include <trapTraj.hpp>
#include <axis.hpp>
#include <communication/communication.h>

//TODO clean this up
extern const float current_meas_period;
extern const int current_meas_hz;
extern bool user_config_loaded_;

extern uint64_t serial_number;
extern char serial_number_str[13];

#endif // __cplusplus


float get_adc_voltage(uint32_t gpio_num);

// general system functions defined in main.cpp
void save_configuration(void);
void erase_configuration(void);
void enter_dfu_mode(void);

#endif /* __ODRIVE_MAIN_H */
