

#include <stm32_system.h>
#include <stm32_adc.hpp>
#include <stm32_can.hpp>
#include <stm32_gpio.hpp>
#include <stm32_i2c.hpp>
#include <stm32_spi.hpp>
#include <stm32_usart.hpp>
#include <stm32_usb.hpp>

#include <usb_cdc.hpp>
#include <winusb_compat.hpp>

#include <devices.hpp>
#include <drv8301.hpp>

#include <odrive_main.h>

#include <freertos.hpp>

#include <inttypes.h>


//#define __MAIN_CPP__
#include "odrive_v3_x.hpp"





uint8_t axes_obj_bufs[sizeof(Axis[AXIS_COUNT])];
Axis* axes = reinterpret_cast<Axis*>(axes_obj_bufs);

#include "nvm_config.hpp"

#include "freertos_vars.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>
#include <communication/can_simple.hpp>

BoardConfig_t board_config;
PerChannelConfig_t axis_configs[AXIS_COUNT];
bool user_config_loaded_;

const float current_meas_period = CURRENT_MEAS_PERIOD;
const int current_meas_hz = (int)(CURRENT_MEAS_HZ);
SystemStats_t system_stats_ = { 0 };

uint64_t serial_number = 0;
char serial_number_str[13]; // 12 digits + null termination
char hw_version_str[16];
char product_name_str[64];


typedef Config<
    BoardConfig_t,
    CANSimple::Config_t,
    PerChannelConfig_t[AXIS_COUNT]> ConfigFormat;


// @brief Returns the ADC voltage associated with the specified pin.
// GPIO_set_to_analog() must be called first to put the Pin into
// analog mode.
// Returns NaN if the pin has no associated ADC1 channel.
//
// On ODrive 3.3 and 3.4 the following pins can be used with this function:
//  GPIO_1, GPIO_2, GPIO_3, GPIO_4 and some pins that are connected to
//  on-board sensors (M0_TEMP, M1_TEMP, AUX_TEMP)
//
// The ADC values are sampled in background at ~30kHz without
// any CPU involvement.
//
// Details: each of the 16 conversion takes (15+26) ADC clock
// cycles and the ADC, so the update rate of the entire sequence is:
//  21000kHz / (15+26) / 16 = 32kHz
// The true frequency is slightly lower because of the injected vbus
// measurements
float get_adc_voltage(uint32_t gpio_num) {
    float result = 0.0f / 0.0f;
    if (gpio_num < sizeof(gpio_adcs) / sizeof(gpio_adcs[0])) {
        gpio_adcs[gpio_num].get_normalized(&result);
        result *= 3.3f;
    }
    return result;
}

// TODO: make a BrakeResistor class
void start_brake_pwm() {
    // Start brake resistor PWM in floating output configuration
    tim2.htim.Instance->CCR3 = 0;
    tim2.htim.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    // Enable Channel3 P and Channel 4 P
    tim2.enable_pwm(false, false, false, false, true, false, true, false);
    safety_critical_arm_brake_resistor();
}


void save_configuration(void) {
    if (ConfigFormat::safe_store_config(
            &board_config,
            &can_config,
            &axis_configs)) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    } else {
        user_config_loaded_ = true;
    }
}

extern "C" int load_configuration(void) {
    // Try to load configs
    if (NVM_init() ||
        ConfigFormat::safe_load_config(
                &board_config,
                &can_config,
                &axis_configs)) {
        //If loading failed, restore defaults
        board_config = BoardConfig_t();
        can_config = CANSimple::Config_t();
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            axis_configs[i] = PerChannelConfig_t();

            // Load board-specific defaults
            axis_configs[i].axis_config.step_gpio_num = default_step_gpio_nums[i];
            axis_configs[i].axis_config.dir_gpio_num = default_dir_gpio_nums[i];
            axis_configs[i].axis_config.can_node_id = i;
        }

        // Load board-specific defaults
        board_config.brake_resistance = DEFAULT_BRAKE_RESISTANCE;
        board_config.dc_bus_undervoltage_trip_level = DEFAULT_UNDERVOLTAGE_TRIP_LEVEL;
        board_config.dc_bus_overvoltage_trip_level = DEFAULT_OVERVOLTAGE_TRIP_LEVEL;
    } else {
        user_config_loaded_ = true;
    }
    return user_config_loaded_;
}

void erase_configuration(void) {
    NVM_erase();
}

void enter_dfu_mode() {
    if ((hw_version_major == 3) && (hw_version_minor >= 5)) {
        __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        _reboot_cookie = 0xDEADBEEF;
        NVIC_SystemReset();
    } else {
        /*
        * DFU mode is only allowed on board version >= 3.5 because it can burn
        * the brake resistor FETs on older boards.
        * If you really want to use it on an older board, add 3.3k pull-down resistors
        * to the AUX_L and AUX_H signals and _only then_ uncomment these lines.
        */
        //__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        //_reboot_cookie = 0xDEADFE75;
        //NVIC_SystemReset();
    }
}

extern "C" {
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for (;;); // TODO: safe action
}
void vApplicationIdleHook(void) {
    // TODO: reconsider how to get stack sizes
    if (system_stats_.fully_booted) {
        system_stats_.uptime = xTaskGetTickCount();
        system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();
        system_stats_.min_stack_space_comms = uxTaskGetStackHighWaterMark(comm_thread) * sizeof(StackType_t);
//        system_stats_.min_stack_space_axis0 = uxTaskGetStackHighWaterMark(axes[0].thread_id_) * sizeof(StackType_t);
//        system_stats_.min_stack_space_axis1 = uxTaskGetStackHighWaterMark(axes[1].thread_id_) * sizeof(StackType_t);
//        system_stats_.min_stack_space_usb = uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
//        system_stats_.min_stack_space_uart = uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
//        system_stats_.min_stack_space_usb_irq = uxTaskGetStackHighWaterMark(usb.irq_thread) * sizeof(StackType_t);
        system_stats_.min_stack_space_startup = uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
//        system_stats_.min_stack_space_can = uxTaskGetStackHighWaterMark(odCAN->thread_id_) * sizeof(StackType_t);
    }
}
}

USBCompositeDevice composite_device;
USB_CDC_t cdc_class(&cdc_cmd_endpoint, &cdc_tx_endpoint, &cdc_rx_endpoint);
USB_ODriveNativeClass_t odrive_class(&odrive_tx_endpoint, &odrive_rx_endpoint);

int main_task(void);

/**
 * @brief Main entry point called from assembly startup code
 */
int main(void) {
    if (!system_init()) {
        for (;;); // TODO: define error action
    }

    if (!init_monotonic_clock(&tim14, NVIC_PRIO_TICK_TIMER)) {
        for (;;); // TODO: define error action
    }

    // Load persistent configuration (or defaults)
    load_configuration();

    // Load Axis objects
    board_init(reinterpret_cast<Axis*>(axes_obj_bufs));

    // Init timer for FreeRTOS scheduler

    /* Init FreeRTOS resources (in freertos.cpp) */
    freertos_init(&main_task, RTOS_PRIO_MAIN_TASK);

    /* Start scheduler */
    osKernelStart();

    // We should never get here as control is now taken by the scheduler
    // and all further initialization happens on main_task().
    for (;;);
}

/**
 * @brief Startup task that is started once FreeRTOS is set up.
 */
int main_task(void) {
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);
    uint32_t uuid_mixed_part = uuid0 + uuid2;
    serial_number = ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);

    uint64_t val = serial_number;
    for (size_t i = 0; i < 12; ++i) {
        serial_number_str[i] = "0123456789ABCDEF"[(val >> (48-4)) & 0xf];
        val <<= 4;
    }
    serial_number_str[12] = 0;

    char cdc_interface_str[64];
    sprintf(cdc_interface_str, "%s CDC Interface", hw_version_str);

    char native_interface_str[64];
    sprintf(native_interface_str, "%s Native Interface", hw_version_str);


    /* Setup Communication I/O -----------------------------------------------*/

    system_stats_.boot_progress = 0;

    // Set up USB device
    if (!composite_device.register_class(&cdc_class))
        goto fail;
    if (!composite_device.register_class(&odrive_class))
        goto fail;
    if (!usb.add_vendor_request_handler(&ms_request_handler)) // zero-config WinUSB support
        goto fail;
    if (!usb.add_int_endpoint(&cdc_cmd_endpoint))
        goto fail;
    if (!usb.add_rx_endpoint(&cdc_rx_endpoint))
        goto fail;
    if (!usb.add_tx_endpoint(&cdc_tx_endpoint))
        goto fail;
    if (!usb.add_rx_endpoint(&odrive_rx_endpoint))
        goto fail;
    if (!usb.add_tx_endpoint(&odrive_tx_endpoint))
        goto fail;
    if (!usb.init(
        0x1209, 0x0D32, 1033, // VID, PID, LangID
        "ODrive Robotics", cdc_interface_str, serial_number_str, "CDC Config", "CDC Interface", native_interface_str,
        &composite_device, RTOS_PRIO_USB
    ))
        goto fail;
    if (!usb.enable_interrupts(NVIC_PRIO_USB))
        goto fail;
    if (!usb.start())
        goto fail;

    system_stats_.boot_progress++;

#if 0
    // M0/M1 PWM input
    tim5.init(
        0xFFFFFFFF, // period
        STM32_Timer_t::UP
    );
    tim5.config_input_compare_mode(&pa2, &pa3);
    tim5.enable_cc_interrupt(pwm_in_cb, nullptr);

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_i2c) {
        // TODO: make
        if (i2c_a0_gpio)
            i2c_a0_gpio->init(GPIO_t::INPUT, GPIO_t::PULL_UP);
        if (i2c_a1_gpio)
            i2c_a1_gpio->init(GPIO_t::INPUT, GPIO_t::PULL_UP);
        if (i2c_a2_gpio)
            i2c_a2_gpio->init(GPIO_t::INPUT, GPIO_t::PULL_UP);

        osDelay(1);
        i2c_stats_.addr = (0xD << 3);
        i2c_stats_.addr |= i2c_a0_gpio->read() ? 0x1 : 0;
        i2c_stats_.addr |= i2c_a1_gpio->read() ? 0x2 : 0;
        i2c_stats_.addr |= i2c_a2_gpio->read() ? 0x4 : 0;
        if (!i2c1.setup_as_slave(100000, i2c_stats_.addr, &pb8, &pb9, &dma1_stream6, &dma1_stream5))
            goto fail;
        if (!i2c1.enable_interrupts(NVIC_PRIO_I2C))
            goto fail;
    }
#endif
#endif

    if (board_config.enable_can) {
        if (!can1.init(&pb8, &pb9))
            goto fail;
        if (!can1.enable_interrupts(NVIC_PRIO_CAN))
            goto fail;
        if (!can1.config(can_config.baud))
            goto fail;
        if (!can1.start())
            goto fail;
    }

#if 0
    // Init general user ADC on some GPIOs.
    for (size_t i = 0; i < sizeof(gpios) / sizeof(gpios[0]); ++i) {
        // todo: only set to analog if supported
        gpios[i]->setup_analog();
    }    
#endif

    // TODO: make dynamically reconfigurable
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_uart) {
        if (!uart4.init(115200, gpios[0], gpios[1], &dma1_stream4, &dma1_stream2)) // Provisionally this can be changed to 921600 for faster transfers, the low power Arduinos will not keep up. 
            goto fail;
        if (!uart4.enable_interrupts(NVIC_PRIO_UART))
            goto fail;
    }
#endif

    // TODO: check if clock polarity can be reconfigured on the fly. Datasheet
    // suggests SPI must be disabled first but it doesn't explicitly state that
    // it doesn't work while enabled when timing carefully.
    if (!spi3.init(&pc10, &pc11, &pc12, &dma1_stream7, &dma1_stream0, SPI_CLOCK_POLARITY))
        goto fail;
    if (!spi3.enable_interrupts(NVIC_PRIO_SPI))
        goto fail;

    // Diagnostics timer
    //tim13.init(
    //    (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR+1)) * ((float)TIM_APB1_CLOCK_HZ / (float)TIM_1_8_CLOCK_HZ) - 1, // period
    //    STM32_Timer_t::UP
    //);
    //tim13.enable_update_interrupt(); // todo: this was probably not used

    system_stats_.boot_progress++;

    /* Set up ADC ------------------------------------------------------------*/

    if (!vbus_sense.init()) {
        goto fail;
    }

    system_stats_.boot_progress++;

    // AUX PWM
    if (aux_l && aux_h) {
        tim2.init(
            TIM_APB1_PERIOD_CLOCKS, // period
            STM32_Timer_t::UP_DOWN
        );
        tim2.setup_pwm(3,
                aux_l, nullptr,
                true, true, // active high
                0 // initial value
        ); // AUX L
        tim2.setup_pwm(4,
                aux_h, nullptr,
                true, true, // active high
                TIM_APB1_PERIOD_CLOCKS + 1 // initial value
        ); // AUX H
    }

    system_stats_.boot_progress++;

    // TODO: find a better place to init the ADC sequence
    if (!adc1_injected.init(nullptr) ||
        !adc2_injected.init(nullptr) ||
        !adc3_injected.init(nullptr) ||
        !adc1_regular.init(&dma2_stream0) ||
        !adc2_regular.init(&dma2_stream2) ||
        !adc3_regular.init(&dma2_stream1)) {
        goto fail;
    }

    system_stats_.boot_progress++;

    if (!adc1_injected.set_trigger(&tim1) ||
        !adc2_injected.set_trigger(&tim1) ||
        !adc3_injected.set_trigger(&tim1) ||
        !adc1_regular.set_trigger(&tim8) ||
        !adc2_regular.set_trigger(&tim8) ||
        !adc3_regular.set_trigger(&tim8)) {
        goto fail;
    }

    system_stats_.boot_progress++;

    for (size_t i = 0; i < sizeof(adc_init_list) / sizeof(adc_init_list[0]); ++i) {
        if (!adc_init_list[i]->enqueue()) {
            goto fail;
        }
    }

    system_stats_.boot_progress = 10;

#if 0
    // TODO: think of something that this doesn't disable UART and such
    for (size_t i = 0; i < sizeof(gpio_adcs) / sizeof(gpio_adcs[0]); ++i) {
        printf("will init %d\r\n", i);
        osDelay(100);
        if (gpio_adcs[i].is_valid()) {
            if (!gpio_adcs[i].init() ||
                !adc1_regular.append(&gpio_adcs[i])) {
                goto fail;
            }
        }
    }
#endif

    system_stats_.boot_progress++;

    // TODO: set up the remaining channels

    osDelay(1);

    if (!adc1_injected.apply() ||
        !adc2_injected.apply() ||
        !adc3_injected.apply() ||
        !adc1_regular.apply() ||
        !adc2_regular.apply() ||
        !adc3_regular.apply()) {
        goto fail;
    }

    osDelay(1);

    system_stats_.boot_progress++;
    
    // Start ADC for temperature measurements and user measurements
    if (!adc1_regular.enable_updates() ||
        !adc2_regular.enable_updates() ||
        !adc3_regular.enable_updates() ||
        !adc1_injected.enable_updates() ||
        !adc2_injected.enable_updates() ||
        !adc3_injected.enable_updates()) {
        goto fail;
    }

#if 0
    // Start pwm-in compare modules
    // must happen after communication is initialized
    pwm_in_init();
#endif

    // start CPU cycle counter
    DWT->CTRL |= 0x1;

    system_stats_.boot_progress = 100;

    osDelay(100);

    // Setup hardware for all components
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!axes[i].init()) {
            goto fail;
        }
    }

    //osDelay(100);
    // Init communications
    init_communication();


#if 1
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].motor_.start_updates();
    }
    // Synchronize PWM of both motors up to a 90° PWM phase shift
    sync_timers(axes[0].motor_.timer_, axes[1].motor_.timer_, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128, nullptr);
#endif

#if 0
    // Start brake resistor PWM
    start_brake_pwm();
#endif

    // This delay serves two purposes:
    //  - Let the current sense calibration converge (the current
    //    sense interrupts are firing in background by now)
    //  - Allow a user to interrupt the code, e.g. by flashing a new code,
    //    before it does anything crazy
    // TODO make timing a function of calibration filter tau
    osDelay(1500);

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].start_thread();
    }



#if 0
    start_analog_thread();
#endif

    system_stats_.fully_booted = true;
    for (;;) {
        //uart4.start_tx((const uint8_t*)"a\r\n", 3, nullptr, nullptr);
//        printf("hi\r\n");
        printf("uptime: %" PRIu32 "\r\n", system_stats_.uptime);
//        uint32_t prev_ints = all_usb_ints;
//        all_usb_ints = 0;
//        printf("USB ints: %08" PRIx32 "\r\n", prev_ints);
//        //printf("uptime: %" PRIu32 "\r\n", system_stats_.uptime);
        osDelay(100);

        // Read and publish ISR CPU usage
        uint32_t mask = cpu_enter_critical();
        uint32_t total_cnt = DWT->CYCCNT;
        DWT->CYCCNT = 0;
        uint32_t adc_irq_ticks_copy = adc_irq_ticks; adc_irq_ticks = 0;
        uint32_t dma2_stream1_irq_ticks_copy = dma2_stream1_irq_ticks; dma2_stream1_irq_ticks = 0;
        uint32_t dma2_stream2_irq_ticks_copy = dma2_stream2_irq_ticks; dma2_stream2_irq_ticks = 0;
        uint32_t tim1_up_ticks_copy = tim1_up_tim10_irq_ticks; tim1_up_tim10_irq_ticks = 0;
        uint32_t tim8_up_ticks_copy = tim8_up_tim13_irq_ticks; tim8_up_tim13_irq_ticks = 0;
        cpu_exit_critical(mask);

        system_stats_.adc_irq_usage = (float)adc_irq_ticks_copy / (float)total_cnt;
        system_stats_.dma2_stream1_irq_usage = (float)dma2_stream1_irq_ticks_copy / (float)total_cnt;
        system_stats_.dma2_stream2_irq_usage = (float)dma2_stream2_irq_ticks_copy / (float)total_cnt;
        system_stats_.tim1_up_usage = (float)tim1_up_ticks_copy / (float)total_cnt;
        system_stats_.tim8_up_usage = (float)tim8_up_ticks_copy / (float)total_cnt;
    }
    return 0;
    goto fail;

fail:
    init_communication();
    for (;;) {
        printf("fail\r\n");
        osDelay(1000);
    }
    return -1;
}
