
#define __MAIN_CPP__
#include "odrive_main.h"
#include "nvm_config.hpp"

#include "usart.h"
#include "freertos_vars.h"
#include "usb_device.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>
#include <communication/interface_can.hpp>

osSemaphoreId sem_usb_irq;
osSemaphoreId sem_uart_dma;
osSemaphoreId sem_usb_rx;
osSemaphoreId sem_usb_tx;
osSemaphoreId sem_can;

osThreadId usb_irq_thread;
const uint32_t stack_size_usb_irq_thread = 2048; // Bytes

// Place FreeRTOS heap in core coupled memory for better performance
__attribute__((section(".ccmram")))
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char _estack; // provided by the linker script


ODriveCAN::Config_t can_config;
SensorlessEstimator::Config_t sensorless_configs[AXIS_COUNT];
Controller::Config_t controller_configs[AXIS_COUNT];
OffboardThermistorCurrentLimiter::Config_t motor_thermistor_configs[AXIS_COUNT];
Axis::Config_t axis_configs[AXIS_COUNT];
TrapezoidalTrajectory::Config_t trap_configs[AXIS_COUNT];
Endstop::Config_t min_endstop_configs[AXIS_COUNT];
Endstop::Config_t max_endstop_configs[AXIS_COUNT];

std::array<Axis*, AXIS_COUNT> axes;
ODriveCAN *odCAN = nullptr;
ODrive odrv{};


ConfigManager config_manager;

static bool config_pop_all() {
    bool success = board_pop_config() &&
           config_manager.pop(&odrv.config_) &&
           config_manager.pop(&can_config);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = config_manager.pop(&encoders[i].config_) &&
                  config_manager.pop(&sensorless_configs[i]) &&
                  config_manager.pop(&controller_configs[i]) &&
                  config_manager.pop(&trap_configs[i]) &&
                  config_manager.pop(&min_endstop_configs[i]) &&
                  config_manager.pop(&max_endstop_configs[i]) &&
                  config_manager.pop(&motors[i].config_) &&
                  config_manager.pop(&fet_thermistors[i].config_) &&
                  config_manager.pop(&motor_thermistor_configs[i]) &&
                  config_manager.pop(&axis_configs[i]);
    }
    return success;
}

static bool config_push_all() {
    bool success = board_push_config() &&
           config_manager.push(&odrv.config_) &&
           config_manager.push(&can_config);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = config_manager.push(&encoders[i].config_) &&
                  config_manager.push(&sensorless_configs[i]) &&
                  config_manager.push(&controller_configs[i]) &&
                  config_manager.push(&trap_configs[i]) &&
                  config_manager.push(&min_endstop_configs[i]) &&
                  config_manager.push(&max_endstop_configs[i]) &&
                  config_manager.push(&motors[i].config_) &&
                  config_manager.push(&fet_thermistors[i].config_) &&
                  config_manager.push(&motor_thermistor_configs[i]) &&
                  config_manager.push(&axis_configs[i]);
    }
    return success;
}

static void config_clear_all() {
    odrv.config_ = {};
    can_config = {};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        encoders[i].config_ = {};
        sensorless_configs[i] = {};
        controller_configs[i] = {};
        trap_configs[i] = {};
        motors[i].config_ = {};
        fet_thermistors[i].config_ = {};
        axis_configs[i] = {};
        // Default step/dir pins are different, so we need to explicitly load them
        Axis::load_default_step_dir_pin_config(hw_configs[i].axis_config, &axis_configs[i]);
        Axis::load_default_can_id(i, axis_configs[i]);
        min_endstop_configs[i] = {};
        max_endstop_configs[i] = {};
        controller_configs[i].load_encoder_axis = i;
    }
}

static bool config_apply_all() {
    bool success = true;
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = encoders[i].apply_config(motors[i].config_.motor_type)
               && motors[i].apply_config();
    }
    return success;
}

void ODrive::save_configuration(void) {
    bool success = config_manager.prepare_store()
                && config_push_all()
                && config_manager.start_store()
                && config_push_all()
                && config_manager.finish_store();
    if (success) {
        user_config_loaded_ = true;
    } else {
        printf("saving configuration failed\r\n");
        osDelay(5);
    }
}

void ODrive::erase_configuration(void) {
    NVM_erase();

    // FIXME: this reboot is a workaround because we don't want the next save_configuration
    // to write back the old configuration from RAM to NVM. The proper action would
    // be to reset the values in RAM to default. However right now that's not
    // practical because several startup actions depend on the config. The
    // other problem is that the stack overflows if we reset to default here.
    NVIC_SystemReset();
}

void ODrive::enter_dfu_mode() {
    if ((hw_version_major_ == 3) && (hw_version_minor_ >= 5)) {
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

static void usb_deferred_interrupt_thread(void * ctx) {
    (void) ctx; // unused parameter

    for (;;) {
        // Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
        osStatus semaphore_status = osSemaphoreWait(sem_usb_irq, osWaitForever);
        if (semaphore_status == osOK) {
            // We have a new incoming USB transmission: handle it
            HAL_PCD_IRQHandler(&usb_pcd_handle);
            // Let the irq (OTG_FS_IRQHandler) fire again.
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
}

extern "C" {

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for(auto& axis : axes){
        safety_critical_disarm_motor_pwm(axis->motor_);
    }
        safety_critical_disarm_brake_resistor();
    for (;;); // TODO: safe action
}

void vApplicationIdleHook(void) {
    if (odrv.system_stats_.fully_booted) {
        odrv.system_stats_.uptime = xTaskGetTickCount();
        odrv.system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();
        odrv.system_stats_.min_stack_space_comms = uxTaskGetStackHighWaterMark(comm_thread) * sizeof(StackType_t);
        uint32_t min_stack_space[AXIS_COUNT];
        std::transform(axes.begin(), axes.end(), std::begin(min_stack_space), [](auto& axis) { return uxTaskGetStackHighWaterMark(axes[1]->thread_id_) * sizeof(StackType_t); });
        odrv.system_stats_.min_stack_space_axis = *std::min_element(std::begin(min_stack_space), std::end(min_stack_space));
        odrv.system_stats_.min_stack_space_usb = uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_uart = uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_usb_irq = uxTaskGetStackHighWaterMark(usb_irq_thread) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_startup = uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        odrv.system_stats_.min_stack_space_can = uxTaskGetStackHighWaterMark(odCAN->thread_id_) * sizeof(StackType_t);

        // Actual usage, in bytes, so we don't have to math
        odrv.system_stats_.stack_usage_axis = axes[0]->stack_size_ - odrv.system_stats_.min_stack_space_axis;
        odrv.system_stats_.stack_usage_comms = stack_size_comm_thread - odrv.system_stats_.min_stack_space_comms;
        odrv.system_stats_.stack_usage_usb = stack_size_usb_thread - odrv.system_stats_.min_stack_space_usb;
        odrv.system_stats_.stack_usage_uart = stack_size_uart_thread - odrv.system_stats_.min_stack_space_uart;
        odrv.system_stats_.stack_usage_usb_irq = stack_size_usb_irq_thread - odrv.system_stats_.min_stack_space_usb_irq;
        odrv.system_stats_.stack_usage_startup = stack_size_default_task - odrv.system_stats_.min_stack_space_startup;
        odrv.system_stats_.stack_usage_can = odCAN->stack_size_ - odrv.system_stats_.min_stack_space_can;
    }
}

}


/**
 * @brief Main thread started from main().
 */
static void rtos_main(void*) {
    // Init USB device
    MX_USB_DEVICE_Init();


    // Start ADC for temperature measurements and user measurements
    start_general_purpose_adc();

    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed)
    init_communication();

    // Start pwm-in compare modules
    // must happen after communication is initialized
    pwm_in_init();

    // Setup hardware for all components
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!axes[i]->setup()) {
            for (;;); // TODO: proper error handling
        }
    }

    for(auto& axis : axes){
        axis->encoder_.setup();
    }

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

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
        axes[i]->start_thread();
    }

    start_analog_thread();

    odrv.system_stats_.fully_booted = true;

    // Main thread finished starting everything and can delete itself now (yes this is legal).
    vTaskDelete(defaultTaskHandle);
}

/**
 * @brief Carries out early startup tasks that need to run before any static
 * initializers.
 * This function gets called from the startup assembly code.
 */
extern "C" void early_start_checks(void) {
    if(_reboot_cookie == 0xDEADFE75) {
        /* The STM DFU bootloader enables internal pull-up resistors on PB10 (AUX_H)
        * and PB11 (AUX_L), thereby causing shoot-through on the brake resistor
        * FETs and obliterating them unless external 3.3k pull-down resistors are
        * present. Pull-downs are only present on ODrive 3.5 or newer.
        * On older boards we disable DFU by default but if the user insists
        * there's only one thing left that might save it: time.
        * The brake resistor gate driver needs a certain 10V supply (GVDD) to
        * make it work. This voltage is supplied by the motor gate drivers which get
        * disabled at system reset. So over time GVDD voltage _should_ below
        * dangerous levels. This is completely handwavy and should not be relied on
        * so you are on your own on if you ignore this warning.
        *
        * This loop takes 5 cycles per iteration and at this point the system runs
        * on the internal 16MHz RC oscillator so the delay is about 2 seconds.
        */
        for (size_t i = 0; i < (16000000UL / 5UL * 2UL); ++i) {
            __NOP();
        }
        _reboot_cookie = 0xDEADBEEF;
    }

    /* We could jump to the bootloader directly on demand without rebooting
    but that requires us to reset several peripherals and interrupts for it
    to function correctly. Therefore it's easier to just reset the entire chip. */
    if(_reboot_cookie == 0xDEADBEEF) {
        _reboot_cookie = 0xCAFEFEED;  //Reset bootloader trigger
        __set_MSP((uintptr_t)&_estack);
        // http://www.st.com/content/ccc/resource/technical/document/application_note/6a/17/92/02/58/98/45/0c/CD00264379.pdf/files/CD00264379.pdf
        void (*builtin_bootloader)(void) = (void (*)(void))(*((uint32_t *)0x1FFF0004));
        builtin_bootloader();
    }

    /* The bootloader might fail to properly clean up after itself,
    so if we're not sure that the system is in a clean state we
    just reset it again */
    if(_reboot_cookie != 42) {
        _reboot_cookie = 42;
        NVIC_SystemReset();
    }
}

/**
 * @brief Main entry point called from assembly startup code.
 */
extern "C" int main(void) {
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

    // Init low level system functions (clocks, flash interface)
    system_init();

    // Load configuration from NVM. This needs to happen after system_init()
    // since the flash interface must be initialized and before board_init()
    // since board initialization can depend on the config.
    odrv.user_config_loaded_ = config_manager.start_load()
            && config_pop_all()
            && config_manager.finish_load()
            && config_apply_all();
    if (!odrv.user_config_loaded_) {
        config_clear_all();
        config_apply_all();
    }

    // Init board-specific peripherals
    board_init();

    // Init GPIOs according to their configured mode
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        // Skip unavailable GPIOs
        if (!get_gpio(i)) {
            continue;
        }

        ODriveIntf::GpioMode mode = odrv.config_.gpio_modes[i];

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = get_gpio(i).pin_mask_;
        GPIO_InitStruct.Alternate = 0;

        switch (mode) {
            case ODriveIntf::GPIO_MODE_DIGITAL: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ANALOG_IN: {
                GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
            } break;
            case ODriveIntf::GPIO_MODE_UART0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                GPIO_InitStruct.Alternate = alternate_functions[i][0];
                if (!odrv.config_.enable_uart0) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_PWM0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = alternate_functions[i][1];
            } break;
            case ODriveIntf::GPIO_MODE_CAN0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                GPIO_InitStruct.Alternate = alternate_functions[i][2];
                if (!odrv.config_.enable_can0) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_I2C0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                GPIO_InitStruct.Alternate = alternate_functions[i][3];
                if (!odrv.config_.enable_i2c0) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_ENC0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = alternate_functions[i][4];
            } break;
            case ODriveIntf::GPIO_MODE_ENC1: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = alternate_functions[i][5];
            } break;
            default: {
                GPIO_InitStruct.Alternate = GPIO_AF_NONE;
            }
        }

        // The selected mode is invalid for this GPIO. Leave GPIO uninitialized.
        if (GPIO_InitStruct.Alternate == GPIO_AF_NONE) {
            odrv.misconfigured_ = true;
            continue;
        }

        HAL_GPIO_Init(get_gpio(i).port_, &GPIO_InitStruct);
    }

    // Init usb irq binary semaphore, and start with no tokens by removing the starting one.
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
    osSemaphoreWait(sem_usb_irq, 0);

    // Create a semaphore for UART DMA and remove a token
    osSemaphoreDef(sem_uart_dma);
    sem_uart_dma = osSemaphoreCreate(osSemaphore(sem_uart_dma), 1);

    // Create a semaphore for USB RX
    osSemaphoreDef(sem_usb_rx);
    sem_usb_rx = osSemaphoreCreate(osSemaphore(sem_usb_rx), 1);
    osSemaphoreWait(sem_usb_rx, 0);  // Remove a token.

    // Create a semaphore for USB TX
    osSemaphoreDef(sem_usb_tx);
    sem_usb_tx = osSemaphoreCreate(osSemaphore(sem_usb_tx), 1);

    osSemaphoreDef(sem_can);
    sem_can = osSemaphoreCreate(osSemaphore(sem_can), 1);
    osSemaphoreWait(sem_can, 0);

    // Start USB interrupt handler thread
    osThreadDef(task_usb_pump, usb_deferred_interrupt_thread, osPriorityAboveNormal, 0, stack_size_usb_irq_thread / sizeof(StackType_t));
    usb_irq_thread = osThreadCreate(osThread(task_usb_pump), NULL);


    // Construct all objects.
    odCAN = new ODriveCAN(can_config, &hcan1);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        SensorlessEstimator *sensorless_estimator = new SensorlessEstimator(sensorless_configs[i]);
        Controller *controller = new Controller(controller_configs[i]);
        OffboardThermistorCurrentLimiter *motor_thermistor = new OffboardThermistorCurrentLimiter(motor_thermistor_configs[i]);
        TrapezoidalTrajectory *trap = new TrapezoidalTrajectory(trap_configs[i]);
        Endstop *min_endstop = new Endstop(min_endstop_configs[i]);
        Endstop *max_endstop = new Endstop(max_endstop_configs[i]);
        axes[i] = new Axis(i, hw_configs[i].axis_config, axis_configs[i],
                encoders[i], *sensorless_estimator, *controller, fet_thermistors[i], *motor_thermistor, motors[i], *trap, *min_endstop, *max_endstop);

        controller_configs[i].parent = controller;
        motor_thermistor_configs[i].parent = motor_thermistor;
        min_endstop_configs[i].parent = min_endstop;
        max_endstop_configs[i].parent = max_endstop;
        axis_configs[i].parent = axes[i];
    }

    // Create main thread
    osThreadDef(defaultTask, rtos_main, osPriorityNormal, 0, stack_size_default_task / sizeof(StackType_t));
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // Start scheduler
    osKernelStart();
    
    for (;;);
}
