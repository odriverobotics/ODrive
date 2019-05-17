#ifndef __BOARD_HPP
#define __BOARD_HPP

#if HW_VERSION_MAJOR != 3
#  error "This file is only for use with ODrive V3.x"
#endif

#include "stm32_system.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_dma.hpp"
#include "stm32_i2c.hpp"
#include "stm32_tim.hpp"
#include "stm32_usart.hpp"
#include "stm32_wwdg.hpp"
#include "drv8301.hpp"
#include "motor_impl.hpp"


struct PerChannelConfig_t {
    Encoder::Config_t encoder_config;
    SensorlessEstimator::Config_t sensorless_estimator_config;
    AsyncEstimator::Config_t async_estimator_config;
    Controller::Config_t controller_config;
    TrapezoidalTrajectory::Config_t trap_config;
    Axis::Config_t axis_config;
};


constexpr size_t AXIS_COUNT = 2; // sizeof(axes) / sizeof(axes[0]);
extern PerChannelConfig_t axis_configs[AXIS_COUNT]; // defined in main.cpp

const float thermistor_poly_coeffs[] =
    {363.93910201f, -462.15369634f, 307.55129571f, -27.72569531f};
const size_t thermistor_num_coeffs = sizeof(thermistor_poly_coeffs)/sizeof(thermistor_poly_coeffs[0]);

#if HW_VERSION_MINOR <= 3
#  define SHUNT_RESISTANCE (675e-6f)
#elif (HW_VERSION_MINOR >= 4) && (HW_VERSION_MINOR <= 6)
#  define SHUNT_RESISTANCE (500e-6f)
#endif

#if HW_VERSION_VOLTAGE >= 48
#  define VBUS_S_DIVIDER_RATIO 19.0f
#  define VBUS_OVERVOLTAGE_LEVEL 52.0f
#elif HW_VERSION_VOLTAGE == 24
#  define VBUS_S_DIVIDER_RATIO 11.0f
#  define VBUS_OVERVOLTAGE_LEVEL 26.0f
#else
#  error "unknown board voltage"
#endif

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5 && HW_VERSION_VOLTAGE >= 48
#  define DEFAULT_BRAKE_RESISTANCE 2.0f
#else
#  define DEFAULT_BRAKE_RESISTANCE 0.47f
#endif
#define DEFAULT_UNDERVOLTAGE_TRIP_LEVEL 8.0f
#define DEFAULT_OVERVOLTAGE_TRIP_LEVEL (1.07f * HW_VERSION_VOLTAGE)

// TODO: make dynamic
#define TIM_1_8_CLOCK_HZ 168000000
//#define TIM_1_8_PERIOD_CLOCKS (3500) // not 1kHz
#define TIM_1_8_PERIOD_CLOCKS (10000) // not 1kHz
#define TIM_1_8_DEADTIME_CLOCKS 20
#define TIM_1_8_RCR 0

#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )

#define SPI_CLOCK_POLARITY false


#if HW_VERSION_MINOR <= 2
STM32_ADCInjected_t::Channel_t vbus_sense_adc = adc1_injected.get_channel(&pa0);
#else
STM32_ADCInjected_t::Channel_t vbus_sense_adc = adc1_injected.get_channel(&pa6);
#endif
ADCVoltageSensor_t vbus_sense(&vbus_sense_adc, 3.3f * VBUS_S_DIVIDER_RATIO);


#if (HW_VERSION_MINOR == 1) || (HW_VERSION_MINOR == 2)
STM32_GPIO_t* gpios[] = { &pb2, &pa5, &pa4, &pa3 };
#elif (HW_VERSION_MINOR == 3) || (HW_VERSION_MINOR == 4)
STM32_GPIO_t* gpios[] = { &pa0, &pa1, &pa2, &pa3, &pb2 };
#elif (HW_VERSION_MINOR == 5) || (HW_VERSION_MINOR == 6)
STM32_GPIO_t* gpios[] = { &pa0, &pa1, &pa2, &pa3, &pc4, &pb2, &pa15, &pb3 };
#else
#error "unknown GPIOs"
#endif

const size_t num_gpios = sizeof(gpios) / sizeof(gpios[0]);

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
uint32_t pwm_in_gpios[4] = { 1, 2, 3, 4 };
#else
uint32_t pwm_in_gpios[4] = { 0, 0, 0, 4 }; // 0 means not in use
#endif

// TODO: adapt to GPIOs count
STM32_ADCRegular_t::Channel_t gpio1_adc = adc1_regular.get_channel(gpios[0]);
STM32_ADCRegular_t::Channel_t gpio2_adc = adc1_regular.get_channel(gpios[1]);
STM32_ADCRegular_t::Channel_t gpio3_adc = adc1_regular.get_channel(gpios[2]);
STM32_ADCRegular_t::Channel_t gpio4_adc = adc1_regular.get_channel(gpios[3]);
#if (HW_VERSION_MINOR >= 3)
STM32_ADCRegular_t::Channel_t gpio5_adc = adc1_regular.get_channel(gpios[4]);
#endif
#if (HW_VERSION_MINOR >= 5)
STM32_ADCRegular_t::Channel_t gpio6_adc = adc1_regular.get_channel(gpios[5]);
STM32_ADCRegular_t::Channel_t gpio7_adc = adc1_regular.get_channel(gpios[6]);
STM32_ADCRegular_t::Channel_t gpio8_adc = adc1_regular.get_channel(gpios[7]);
#endif

STM32_ADCRegular_t::Channel_t* gpio_adcs[num_gpios] = {
    &gpio1_adc, &gpio2_adc, &gpio3_adc, &gpio4_adc,
#if (HW_VERSION_MINOR >= 3)
    &gpio5_adc,
#endif
#if (HW_VERSION_MINOR >= 5)
    &gpio6_adc, &gpio7_adc, &gpio8_adc
#endif
};

DRV8301_t gate_driver_m0(
    &spi3,
    &pc13, // chip select
    &pb12, // enable (shared across both motors)
    &pd2 // nFault (shared across both motors)
);

DRV8301_t gate_driver_m1(
    &spi3,
    &pc14, // chip select
    &pb12, // enable (shared across both motors)
    &pd2 // nFault (shared across both motors)
);


STM32_ADCInjected_t::Channel_t current_sensor_m0_b_adc = adc2_injected.get_channel(&pc0);
STM32_ADCInjected_t::Channel_t current_sensor_m0_c_adc = adc3_injected.get_channel(&pc1);
STM32_ADCRegular_t::Channel_t current_sensor_m1_b_adc = adc2_regular.get_channel(&pc3);
STM32_ADCRegular_t::Channel_t current_sensor_m1_c_adc = adc3_regular.get_channel(&pc2);
LinearCurrentSensor_t<decltype(current_sensor_m0_b_adc), &current_sensor_m0_b_adc, decltype(gate_driver_m0), &gate_driver_m0> current_sensor_m0_b(1.0f / SHUNT_RESISTANCE * 3.3f, 0.5f);
LinearCurrentSensor_t<decltype(current_sensor_m0_c_adc), &current_sensor_m0_c_adc, decltype(gate_driver_m0), &gate_driver_m0> current_sensor_m0_c(1.0f / SHUNT_RESISTANCE * 3.3f, 0.5f);
LinearCurrentSensor_t<decltype(current_sensor_m1_b_adc), &current_sensor_m1_b_adc, decltype(gate_driver_m1), &gate_driver_m1> current_sensor_m1_b(1.0f / SHUNT_RESISTANCE * 3.3f, 0.5f);
LinearCurrentSensor_t<decltype(current_sensor_m1_c_adc), &current_sensor_m1_c_adc, decltype(gate_driver_m1), &gate_driver_m1> current_sensor_m1_c(1.0f / SHUNT_RESISTANCE * 3.3f, 0.5f);

STM32_ADCRegular_t::Channel_t temp_sensor_m0_inv_adc = adc1_regular.get_channel(&pc5);
#if HW_VERSION_MINOR <= 2
STM32_ADCRegular_t::Channel_t temp_sensor_m1_inv_adc = adc1_regular.get_channel(&pa1);
#else
STM32_ADCRegular_t::Channel_t temp_sensor_m1_inv_adc = adc1_regular.get_channel(&pa4);
#endif

Thermistor_t<decltype(temp_sensor_m0_inv_adc), temp_sensor_m0_inv_adc> temp_sensor_m0_inv(thermistor_poly_coeffs, thermistor_num_coeffs);
Thermistor_t<decltype(temp_sensor_m1_inv_adc), temp_sensor_m1_inv_adc> temp_sensor_m1_inv(thermistor_poly_coeffs, thermistor_num_coeffs);

ADCChannel_t* adc_sincos_s = &gpio3_adc;
ADCChannel_t* adc_sincos_c = &gpio4_adc;

STM32_GPIO_t* aux_l = &pb10;
STM32_GPIO_t* aux_h = &pb11;


#if (HW_VERSION_MINOR <= 2)
STM32_ADCRegular_t::Channel_t adc_aux_v = adc1_regular.get_channel(&pa6);
STM32_ADCRegular_t::Channel_t adc_aux_i = adc1_regular.get_channel(&pa2);
STM32_ADCRegular_t::Channel_t adc_aux_temp = adc1_regular.get_channel(&pc4);
#elif (HW_VERSION_MINOR <= 4)
STM32_ADCRegular_t::Channel_t adc_aux_i = adc1_regular.get_channel(&pa5);
STM32_ADCRegular_t::Channel_t adc_aux_temp = adc1_regular.get_channel(&pc4);
#endif

const size_t n_axes = AXIS_COUNT;

MultiWatchdog<decltype(wwdg), wwdg, AXIS_COUNT> motor_watchdog;

MotorImpl<
    STM32_Timer_t,
    &tim1,
    STM32_GPIO_t,
    &pb13, &pb14, &pb15, // pwm_{a,b,c}_l_gpio
    &pa8, &pa9, &pa10, // pwm_{a,b,c}_h_gpio
    DRV8301_t,
    &gate_driver_m0, // gate_driver_a
    &gate_driver_m0, // gate_driver_b
    &gate_driver_m0, // gate_driver_c
    CurrentSensor_t, nullptr, // current_sensor_a
    decltype(current_sensor_m0_b), &current_sensor_m0_b, // current_sensor_b
    decltype(current_sensor_m0_c), &current_sensor_m0_c, // current_sensor_c
    decltype(temp_sensor_m0_inv), &temp_sensor_m0_inv, // inverter_thermistor_a
    decltype(temp_sensor_m0_inv), &temp_sensor_m0_inv, // inverter_thermistor_b
    decltype(temp_sensor_m0_inv), &temp_sensor_m0_inv, // inverter_thermistor_c
    TempSensor_t, nullptr, // motor_thermistor_a
    TempSensor_t, nullptr, // motor_thermistor_b
    TempSensor_t, nullptr, // motor_thermistor_c
    ADCVoltageSensor_t, &vbus_sense, // vbus_sense
    decltype(motor_watchdog), &motor_watchdog, 0 // watchdog, watchdog_slot
> m0(
    TIM_1_8_PERIOD_CLOCKS, TIM_1_8_RCR, TIM_1_8_DEADTIME_CLOCKS,
    NVIC_PRIO_M0
);

MotorImpl<
    STM32_Timer_t,
    &tim8,
    STM32_GPIO_t,
    &pa7, &pb0, &pb1, // pwm_{a,b,c}_l_gpio
    &pc6, &pc7, &pc8, // pwm_{a,b,c}_h_gpio
    DRV8301_t,
    &gate_driver_m1, // gate_driver_a
    &gate_driver_m1, // gate_driver_b
    &gate_driver_m1, // gate_driver_c
    CurrentSensor_t, nullptr, // current_sensor_a
    decltype(current_sensor_m1_b), &current_sensor_m1_b, // current_sensor_b
    decltype(current_sensor_m1_c), &current_sensor_m1_c, // current_sensor_c
    decltype(temp_sensor_m1_inv), &temp_sensor_m1_inv, // inverter_thermistor_a
    decltype(temp_sensor_m1_inv), &temp_sensor_m1_inv, // inverter_thermistor_b
    decltype(temp_sensor_m1_inv), &temp_sensor_m1_inv, // inverter_thermistor_c
    TempSensor_t, nullptr, // motor_thermistor_a
    TempSensor_t, nullptr, // motor_thermistor_b
    TempSensor_t, nullptr, // motor_thermistor_c
    ADCVoltageSensor_t,
    &vbus_sense, // vbus_sense
    decltype(motor_watchdog),
    &motor_watchdog, 1 // watchdog, watchdog_slot
> m1(
    TIM_1_8_PERIOD_CLOCKS, TIM_1_8_RCR, TIM_1_8_DEADTIME_CLOCKS,
    NVIC_PRIO_M1
);

static inline bool board_init(Axis axes[AXIS_COUNT]) {
    new (&axes[0]) Axis(
        m0,
        Encoder(
            &tim3, // counter
#if (HW_VERSION_MINOR == 3) || (HW_VERSION_MINOR == 4)
            &pa15, // index_gpio
#elif (HW_VERSION_MINOR == 5) || (HW_VERSION_MINOR == 6)
            &pc9, // index_gpio
#endif
            &pb4, // hallA_gpio
            &pb5, // hallB_gpio
#if (HW_VERSION_MINOR == 3) || (HW_VERSION_MINOR == 4)
            &pa15, // hallC_gpio (same as index pin)
#elif (HW_VERSION_MINOR == 5) || (HW_VERSION_MINOR == 6)
            &pc9, // hallC_gpio (same as index pin)
#endif
            adc_sincos_s, // sincos_s (same for both encoders)
            adc_sincos_c, // sincos_c (same for both encoders)
            axis_configs[0].encoder_config
        ),
        SensorlessEstimator(axis_configs[0].sensorless_estimator_config),
        AsyncEstimator(axis_configs[0].async_estimator_config),
        Controller(axis_configs[0].controller_config),
        TrapezoidalTrajectory(axis_configs[0].trap_config),
        RTOS_PRIO_AXIS0, // thread_priority
        axis_configs[0].axis_config
    );
    new (&axes[1]) Axis(
        m1,
        Encoder(
            &tim4, // counter
#if (HW_VERSION_MINOR <= 4)
            &pb3, // index_gpio
#elif (HW_VERSION_MINOR == 5) || (HW_VERSION_MINOR == 6)
            &pc15, // index_gpio
#endif
            &pb6, // hallA_gpio
            &pb7, // hallB_gpio
#if (HW_VERSION_MINOR <= 4)
            &pb3, // hallC_gpio (same as index pin)
#elif (HW_VERSION_MINOR == 5) || (HW_VERSION_MINOR == 6)
            &pc15, // hallC_gpio (same as index pin)
#endif
            adc_sincos_s, // sincos_s (same for both encoders)
            adc_sincos_c, // sincos_c (same for both encoders)
            axis_configs[1].encoder_config
        ),
        SensorlessEstimator(axis_configs[1].sensorless_estimator_config),
        AsyncEstimator(axis_configs[1].async_estimator_config),
        Controller(axis_configs[1].controller_config),
        TrapezoidalTrajectory(axis_configs[1].trap_config),
        RTOS_PRIO_AXIS1, // thread_priority
        axis_configs[1].axis_config
    );

    sprintf(hw_version_str, "%d.%d-%dV", hw_version_major, hw_version_minor, hw_version_variant);
    sprintf(product_name_str, "ODrive v%s", hw_version_str);
    return true;
}

bool enqueue_adc_channels() {
    bool success = current_sensor_m0_b_adc.enqueue() &&
                   current_sensor_m0_c_adc.enqueue() &&
                   current_sensor_m1_b_adc.enqueue() &&
                   current_sensor_m1_c_adc.enqueue() &&
                   vbus_sense_adc.enqueue() &&
                   temp_sensor_m0_inv_adc.enqueue() &&
                   temp_sensor_m1_inv_adc.enqueue();
    return success;
}

GPIO_t* i2c_a0_gpio = gpios[2];
GPIO_t* i2c_a1_gpio = gpios[3];
GPIO_t* i2c_a2_gpio = gpios[4];


#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
uint32_t default_step_gpio_nums[AXIS_COUNT] = { 1, 7 };
uint32_t default_dir_gpio_nums[AXIS_COUNT] = { 2, 8 };
#else
uint32_t default_step_gpio_nums[AXIS_COUNT] = { 1, 3 };
uint32_t default_dir_gpio_nums[AXIS_COUNT] = { 2, 4 };
#endif





STM32_USBTxEndpoint_t cdc_tx_endpoint(&usb.hUsbDeviceFS, 0x81);    /* EP1 for data IN */
STM32_USBRxEndpoint_t cdc_rx_endpoint(&usb.hUsbDeviceFS, 0x01);    /* EP1 for data OUT */
STM32_USBIntEndpoint_t cdc_cmd_endpoint(&usb.hUsbDeviceFS, 0x82);  /* EP2 for CDC commands */
STM32_USBTxEndpoint_t odrive_tx_endpoint(&usb.hUsbDeviceFS, 0x83); /* EP3 IN: ODrive device TX endpoint */
STM32_USBRxEndpoint_t odrive_rx_endpoint(&usb.hUsbDeviceFS, 0x03); /* EP3 OUT: ODrive device RX endpoint */


STM32_USART_t* comm_uart = &uart4;

#if HW_VERSION_MAJOR == 3
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
const uint8_t hw_version_major = otp_ptr ? otp_ptr[3] : HW_VERSION_MAJOR;
const uint8_t hw_version_minor = otp_ptr ? otp_ptr[4] : HW_VERSION_MINOR;
const uint8_t hw_version_variant = otp_ptr ? otp_ptr[5] : HW_VERSION_VOLTAGE;
#else
#error "not implemented"
#endif


static inline bool board_init_devices(void) {
    return true; // nothing to do
}


#endif // __BOARD_HPP