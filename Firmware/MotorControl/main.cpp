
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
osMessageQId uart_event_queue;
osMessageQId usb_event_queue;
osSemaphoreId sem_can;

#if defined(STM32F405xx)
// Place FreeRTOS heap in core coupled memory for better performance
__attribute__((section(".ccmram")))
#endif
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char _estack; // provided by the linker script


ODrive odrv{};


ConfigManager config_manager;

class StatusLedController {
public:
    void update();
};

StatusLedController status_led_controller;

void StatusLedController::update() {
#if HW_VERSION_MAJOR == 4
    uint32_t t = HAL_GetTick();

    bool is_booting = std::any_of(axes.begin(), axes.end(), [](Axis& axis){
        return axis.current_state_ == Axis::AXIS_STATE_UNDEFINED;
    });

    if (is_booting) {
        return;
    }

    bool is_armed = std::any_of(axes.begin(), axes.end(), [](Axis& axis){
        return axis.motor_.is_armed_;
    });
    bool any_error = odrv.any_error();

    if (is_armed) {
        // Fast green pulsating
        const uint32_t period_ms = 256;
        const uint8_t min_brightness = 0;
        const uint8_t max_brightness = 255;
        uint32_t brightness = std::abs((int32_t)(t % period_ms) - (int32_t)(period_ms / 2)) * (max_brightness - min_brightness) / (period_ms / 2) + min_brightness;
        brightness = (brightness * brightness) >> 8; // eye response very roughly sqrt
        status_led.set_color(rgb_t{(uint8_t)(any_error ? brightness / 2 : 0), (uint8_t)brightness, 0});
    } else if (any_error) {
        // Red pulsating
        const uint32_t period_ms = 1024;
        const uint8_t min_brightness = 0;
        const uint8_t max_brightness = 255;
        uint32_t brightness = std::abs((int32_t)(t % period_ms) - (int32_t)(period_ms / 2)) * (max_brightness - min_brightness) / (period_ms / 2) + min_brightness;
        brightness = (brightness * brightness) >> 8; // eye response very roughly sqrt
        status_led.set_color(rgb_t{(uint8_t)brightness, 0, 0});
    } else {
        // Slow blue pulsating
        const uint32_t period_ms = 4096;
        const uint8_t min_brightness = 50;
        const uint8_t max_brightness = 160;
        uint32_t brightness = std::abs((int32_t)(t % period_ms) - (int32_t)(period_ms / 2)) * (max_brightness - min_brightness) / (period_ms / 2) + min_brightness;
        brightness = (brightness * brightness) >> 8; // eye response very roughly sqrt
        status_led.set_color(rgb_t{0, 0, (uint8_t)brightness});
    }
#endif
}

static bool config_read_all() {
    bool success = board_read_config() &&
           config_manager.read(&odrv.config_) &&
           config_manager.read(&odrv.can_.config_);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = config_manager.read(&encoders[i].config_) &&
                  config_manager.read(&axes[i].sensorless_estimator_.config_) &&
                  config_manager.read(&axes[i].controller_.config_) &&
                  config_manager.read(&axes[i].trap_traj_.config_) &&
                  config_manager.read(&axes[i].min_endstop_.config_) &&
                  config_manager.read(&axes[i].max_endstop_.config_) &&
                  config_manager.read(&axes[i].mechanical_brake_.config_) &&
                  config_manager.read(&motors[i].config_) &&
                  config_manager.read(&motors[i].fet_thermistor_.config_) &&
                  config_manager.read(&motors[i].motor_thermistor_.config_) &&
                  config_manager.read(&axes[i].config_);
    }
    return success;
}

static bool config_write_all() {
    bool success = board_write_config() &&
           config_manager.write(&odrv.config_) &&
           config_manager.write(&odrv.can_.config_);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = config_manager.write(&encoders[i].config_) &&
                  config_manager.write(&axes[i].sensorless_estimator_.config_) &&
                  config_manager.write(&axes[i].controller_.config_) &&
                  config_manager.write(&axes[i].trap_traj_.config_) &&
                  config_manager.write(&axes[i].min_endstop_.config_) &&
                  config_manager.write(&axes[i].max_endstop_.config_) &&
                  config_manager.write(&axes[i].mechanical_brake_.config_) &&
                  config_manager.write(&motors[i].config_) &&
                  config_manager.write(&motors[i].fet_thermistor_.config_) &&
                  config_manager.write(&motors[i].motor_thermistor_.config_) &&
                  config_manager.write(&axes[i].config_);
    }
    return success;
}

static void config_clear_all() {
    odrv.config_ = {};
    odrv.can_.config_ = {};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        encoders[i].config_ = {};
        axes[i].sensorless_estimator_.config_ = {};
        axes[i].controller_.config_ = {};
        axes[i].controller_.config_.load_encoder_axis = i;
        axes[i].trap_traj_.config_ = {};
        axes[i].min_endstop_.config_ = {};
        axes[i].max_endstop_.config_ = {};
        axes[i].mechanical_brake_.config_ = {};
        motors[i].config_ = {};
        motors[i].fet_thermistor_.config_ = {};
        motors[i].motor_thermistor_.config_ = {};
        axes[i].clear_config();
    }
}

static bool config_apply_all() {
    bool success = odrv.can_.apply_config();
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = encoders[i].apply_config(motors[i].config_.motor_type)
               && axes[i].controller_.apply_config()
               && axes[i].min_endstop_.apply_config()
               && axes[i].max_endstop_.apply_config()
               && motors[i].apply_config()
               && motors[i].motor_thermistor_.apply_config()
               && axes[i].apply_config();
    }
    return success;
}

bool ODrive::save_configuration(void) {
    bool success;

    CRITICAL_SECTION() {
        bool any_armed = std::any_of(axes.begin(), axes.end(),
            [](auto& axis){ return axis.motor_.is_armed_; });
        if (any_armed) {
            return false;
        }

        size_t config_size = 0;
        success = config_manager.prepare_store()
               && config_write_all()
               && config_manager.start_store(&config_size)
               && config_write_all()
               && config_manager.finish_store();

        // FIXME: during save_configuration we might miss some interrupts
        // because the CPU gets halted during a flash erase. Missing events
        // (encoder updates, step/dir steps) is not good so to be sure we just
        // reboot.
        NVIC_SystemReset();
    }

    return success;
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

bool ODrive::any_error() {
    return error_ != ODrive::ERROR_NONE
        || std::any_of(axes.begin(), axes.end(), [](Axis& axis){
            return axis.error_ != Axis::ERROR_NONE
                || axis.motor_.error_ != Motor::ERROR_NONE
                || axis.sensorless_estimator_.error_ != SensorlessEstimator::ERROR_NONE
                || axis.encoder_.error_ != Encoder::ERROR_NONE
                || axis.controller_.error_ != Controller::ERROR_NONE;
        });
}

uint64_t ODrive::get_drv_fault() {
#if AXIS_COUNT == 1
    return motors[0].gate_driver_.get_error();
#elif AXIS_COUNT == 2
    return (uint64_t)motors[0].gate_driver_.get_error() | ((uint64_t)motors[1].gate_driver_.get_error() << 32ULL);
#else
    #error "not supported"
#endif
}

void ODrive::clear_errors() {
    for (auto& axis: axes) {
        axis.motor_.error_ = Motor::ERROR_NONE;
        axis.controller_.error_ = Controller::ERROR_NONE;
        axis.sensorless_estimator_.error_ = SensorlessEstimator::ERROR_NONE;
        axis.encoder_.error_ = Encoder::ERROR_NONE;
        axis.encoder_.spi_error_rate_ = 0.0f;
        axis.error_ = Axis::ERROR_NONE;
    }
    error_ = ERROR_NONE;
    if (odrv.config_.enable_brake_resistor) {
        safety_critical_arm_brake_resistor();
    }
}

extern "C" {

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for(auto& axis: axes){
        axis.motor_.disarm();
    }
    safety_critical_disarm_brake_resistor();
    for (;;); // TODO: safe action
}

void vApplicationIdleHook(void) {
    if (odrv.system_stats_.fully_booted) {
        odrv.system_stats_.uptime = xTaskGetTickCount();
        odrv.system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();

        uint32_t min_stack_space[AXIS_COUNT];
        std::transform(axes.begin(), axes.end(), std::begin(min_stack_space), [](auto& axis) { return uxTaskGetStackHighWaterMark(axis.thread_id_) * sizeof(StackType_t); });
        odrv.system_stats_.max_stack_usage_axis = axes[0].stack_size_ - *std::min_element(std::begin(min_stack_space), std::end(min_stack_space));
        odrv.system_stats_.max_stack_usage_usb = stack_size_usb_thread - uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_uart = stack_size_uart_thread - uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_startup = stack_size_default_task - uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_can = odrv.can_.stack_size_ - uxTaskGetStackHighWaterMark(odrv.can_.thread_id_) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_analog =  stack_size_analog_thread - uxTaskGetStackHighWaterMark(analog_thread) * sizeof(StackType_t);

        odrv.system_stats_.stack_size_axis = axes[0].stack_size_;
        odrv.system_stats_.stack_size_usb = stack_size_usb_thread;
        odrv.system_stats_.stack_size_uart = stack_size_uart_thread;
        odrv.system_stats_.stack_size_startup = stack_size_default_task;
        odrv.system_stats_.stack_size_can = odrv.can_.stack_size_;
        odrv.system_stats_.stack_size_analog = stack_size_analog_thread;

        odrv.system_stats_.prio_axis = osThreadGetPriority(axes[0].thread_id_);
        odrv.system_stats_.prio_usb = osThreadGetPriority(usb_thread);
        odrv.system_stats_.prio_uart = osThreadGetPriority(uart_thread);
        odrv.system_stats_.prio_startup = osThreadGetPriority(defaultTaskHandle);
        odrv.system_stats_.prio_can = osThreadGetPriority(odrv.can_.thread_id_);
        odrv.system_stats_.prio_analog = osThreadGetPriority(analog_thread);

        status_led_controller.update();
    }
}
}

/**
 * @brief Runs system-level checks that need to be as real-time as possible.
 * 
 * This function is called after every current measurement of every motor.
 * It should finish as quickly as possible.
 */
void ODrive::do_fast_checks() {
    if (!(vbus_voltage >= config_.dc_bus_undervoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_UNDER_VOLTAGE);
    if (!(vbus_voltage <= config_.dc_bus_overvoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_OVER_VOLTAGE);
}

/**
 * @brief Floats all power phases on the system (all motors and brake resistors).
 *
 * This should be called if a system level exception ocurred that makes it
 * unsafe to run power through the system in general.
 */
void ODrive::disarm_with_error(Error error) {
    CRITICAL_SECTION() {
        for (auto& axis: axes) {
            axis.motor_.disarm_with_error(Motor::ERROR_SYSTEM_LEVEL);
        }
        safety_critical_disarm_brake_resistor();
        error_ |= error;
    }
}

/**
 * @brief Runs the periodic sampling tasks
 * 
 * All components that need to sample real-world data should do it in this
 * function as it runs on a high interrupt priority and provides lowest possible
 * timing jitter.
 * 
 * All function called from this function should adhere to the following rules:
 *  - Try to use the same number of CPU cycles in every iteration.
 *    (reason: Tasks that run later in the function still want lowest possible timing jitter)
 *  - Use as few cycles as possible.
 *    (reason: The interrupt blocks other important interrupts (TODO: which ones?))
 *  - Not call any FreeRTOS functions.
 *    (reason: The interrupt priority is higher than the max allowed priority for syscalls)
 * 
 * Time consuming and undeterministic logic/arithmetic should live on
 * control_loop_cb() instead.
 */
void ODrive::sampling_cb() {
    n_evt_sampling_++;

    MEASURE_TIME(task_times_.sampling) {
        for (auto& axis: axes) {
            axis.encoder_.sample_now();
        }
    }
}

/**
 * @brief Runs the periodic control loop.
 * 
 * This function is executed in a low priority interrupt context and is allowed
 * to call CMSIS functions.
 * 
 * Yet it runs at a higher priority than communication workloads.
 * 
 * @param update_cnt: The true count of update events (wrapping around at 16
 *        bits). This is used for timestamp calculation in the face of
 *        potentially missed timer update interrupts. Therefore this counter
 *        must not rely on any interrupts.
 */
void ODrive::control_loop_cb(uint32_t timestamp) {
    last_update_timestamp_ = timestamp;
    n_evt_control_loop_++;

    // TODO: use a configurable component list for most of the following things

    MEASURE_TIME(task_times_.control_loop_misc) {
        // Reset all output ports so that we are certain about the freshness of
        // all values that we use.
        // If we forget to reset a value here the worst that can happen is that
        // this safety check doesn't work.
        // TODO: maybe we should add a check to output ports that prevents
        // double-setting the value.
        for (auto& axis: axes) {
            axis.acim_estimator_.slip_vel_.reset();
            axis.acim_estimator_.stator_phase_vel_.reset();
            axis.acim_estimator_.stator_phase_.reset();
            axis.controller_.torque_output_.reset();
            axis.encoder_.phase_.reset();
            axis.encoder_.phase_vel_.reset();
            axis.encoder_.pos_estimate_.reset();
            axis.encoder_.vel_estimate_.reset();
            axis.encoder_.pos_circular_.reset();
            axis.motor_.Vdq_setpoint_.reset();
            axis.motor_.Idq_setpoint_.reset();
            axis.open_loop_controller_.Idq_setpoint_.reset();
            axis.open_loop_controller_.Vdq_setpoint_.reset();
            axis.open_loop_controller_.phase_.reset();
            axis.open_loop_controller_.phase_vel_.reset();
            axis.open_loop_controller_.total_distance_.reset();
            axis.sensorless_estimator_.phase_.reset();
            axis.sensorless_estimator_.phase_vel_.reset();
            axis.sensorless_estimator_.vel_estimate_.reset();
        }

        uart_poll();
        odrv.oscilloscope_.update();
    }

    MEASURE_TIME(task_times_.control_loop_checks) {
        for (auto& axis: axes) {
            // look for errors at axis level and also all subcomponents
            bool checks_ok = axis.do_checks(timestamp);

            // make sure the watchdog is being fed. 
            bool watchdog_ok = axis.watchdog_check();

            if (!checks_ok || !watchdog_ok) {
                axis.motor_.disarm();
            }
        }
    }

    for (auto& axis: axes) {
        // Sub-components should use set_error which will propegate to this error_
        MEASURE_TIME(axis.task_times_.thermistor_update) {
            axis.motor_.fet_thermistor_.update();
            axis.motor_.motor_thermistor_.update();
        }

        MEASURE_TIME(axis.task_times_.encoder_update)
            axis.encoder_.update();
    }

    // Controller of either axis might use the encoder estimate of the other
    // axis so we process both encoders before we continue.

    for (auto& axis: axes) {
        MEASURE_TIME(axis.task_times_.sensorless_estimator_update)
            axis.sensorless_estimator_.update();

        MEASURE_TIME(axis.task_times_.endstop_update) {
            axis.min_endstop_.update();
            axis.max_endstop_.update();
        }

        MEASURE_TIME(axis.task_times_.controller_update)
            axis.controller_.update(); // uses position and velocity from encoder

        MEASURE_TIME(axis.task_times_.open_loop_controller_update)
            axis.open_loop_controller_.update(timestamp);

        MEASURE_TIME(axis.task_times_.motor_update)
            axis.motor_.update(timestamp); // uses torque from controller and phase_vel from encoder

        MEASURE_TIME(axis.task_times_.current_controller_update)
            axis.motor_.current_control_.update(timestamp); // uses the output of controller_ or open_loop_contoller_ and encoder_ or sensorless_estimator_ or acim_estimator_
    }

    // Tell the axis threads that the control loop has finished
    for (auto& axis: axes) {
        if (axis.thread_id_) {
            osSignalSet(axis.thread_id_, 0x0001);
        }
    }

    get_gpio(odrv.config_.error_gpio_pin).write(odrv.any_error());
}


/** @brief For diagnostics only */
uint32_t ODrive::get_interrupt_status(int32_t irqn) {
    if ((irqn < -14) || (irqn >= 240)) {
        return 0xffffffff;
    }

    uint8_t priority = (irqn < -12)
        ? 0 // hard fault and NMI always have maximum priority
        : NVIC_GetPriority((IRQn_Type)irqn);
    uint32_t counter = GET_IRQ_COUNTER((IRQn_Type)irqn);
    bool is_enabled = (irqn < 0)
        ? true // processor interrupt vectors are always enabled
        : NVIC->ISER[(((uint32_t)(int32_t)irqn) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)(int32_t)irqn) & 0x1FUL));
    
    return priority | ((counter & 0x7ffffff) << 8) | (is_enabled ? 0x80000000 : 0);
}

/** @brief For diagnostics only */
uint32_t ODrive::get_dma_status(uint8_t stream_num) {
    DMA_Stream_TypeDef* streams[] = {
        DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
        DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
    };
    if (stream_num >= 16) {
        return 0xffffffff;
    }
    DMA_Stream_TypeDef* stream = streams[stream_num];
    bool is_reset = (stream->CR == 0x00000000)
                 && (stream->NDTR == 0x00000000)
                 && (stream->PAR == 0x00000000)
                 && (stream->M0AR == 0x00000000)
                 && (stream->M1AR == 0x00000000)
                 && (stream->FCR == 0x00000021);
    uint8_t channel = ((stream->CR & DMA_SxCR_CHSEL_Msk) >> DMA_SxCR_CHSEL_Pos);
    uint8_t priority = ((stream->CR & DMA_SxCR_PL_Msk) >> DMA_SxCR_PL_Pos);
    return (is_reset ? 0 : 0x80000000) | ((channel & 0x7) << 2) | (priority & 0x3);
}

uint32_t ODrive::get_gpio_states() {
    // TODO: get values that were sampled synchronously with the control loop
    uint32_t val = 0;
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        val |= ((gpios[i].read() ? 1UL : 0UL) << i);
    }
    return val;
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
    pwm0_input.init();

    // Set up the CS pins for absolute encoders (TODO: move to GPIO init switch statement)
    for(auto& axis : axes){
        if(axis.encoder_.config_.mode & Encoder::MODE_FLAG_ABS){
            axis.encoder_.abs_spi_cs_pin_init();
        }
    }

    // Try to initialized gate drivers for fault-free startup.
    // If this does not succeed, a fault will be raised and the idle loop will
    // periodically attempt to reinit the gate driver.
    for(auto& axis: axes){
        axis.motor_.setup();
    }

    for(auto& axis: axes){
        axis.encoder_.setup();
    }

    for(auto& axis: axes){
        axis.acim_estimator_.idq_src_.connect_to(&axis.motor_.Idq_setpoint_);
    }

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();
    start_analog_thread();

    // Wait for up to 2s for motor to become ready to allow for error-free
    // startup. This delay gives the current sensor calibration time to
    // converge. If the DRV chip is unpowered, the motor will not become ready
    // but we still enter idle state.
    for (size_t i = 0; i < 2000; ++i) {
        bool motors_ready = std::all_of(axes.begin(), axes.end(), [](auto& axis) {
            return axis.motor_.current_meas_.has_value();
        });
        if (motors_ready) {
            break;
        }
        osDelay(1);
    }

    for (auto& axis: axes) {
        axis.sensorless_estimator_.error_ &= ~SensorlessEstimator::ERROR_UNKNOWN_CURRENT_MEASUREMENT;
    }

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].start_thread();
    }

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
    size_t config_size = 0;
    bool success = config_manager.start_load()
            && config_read_all()
            && config_manager.finish_load(&config_size)
            && config_apply_all();
    if (success) {
        odrv.user_config_loaded_ = config_size;
    } else {
        config_clear_all();
        config_apply_all();
    }

    odrv.misconfigured_ = odrv.misconfigured_
            || (odrv.config_.enable_uart_a && !uart_a)
            || (odrv.config_.enable_uart_b && !uart_b)
            || (odrv.config_.enable_uart_c && !uart_c);

    // Init board-specific peripherals
    if (!board_init()) {
        for (;;); // TODO: handle properly
    }

    // Init GPIOs according to their configured mode
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        // Skip unavailable GPIOs
        if (!get_gpio(i)) {
            continue;
        }

        ODriveIntf::GpioMode mode = odrv.config_.gpio_modes[i];

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = get_gpio(i).pin_mask_;

        // Set Alternate Function setting for this GPIO mode
        if (mode == ODriveIntf::GPIO_MODE_DIGITAL ||
            mode == ODriveIntf::GPIO_MODE_DIGITAL_PULL_UP ||
            mode == ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN ||
            mode == ODriveIntf::GPIO_MODE_MECH_BRAKE ||
            mode == ODriveIntf::GPIO_MODE_STATUS ||
            mode == ODriveIntf::GPIO_MODE_ANALOG_IN) {
            GPIO_InitStruct.Alternate = 0;
        } else {
            auto it = std::find_if(
                    alternate_functions[i].begin(), alternate_functions[i].end(),
                    [mode](auto a) { return a.mode == mode; });

            if (it == alternate_functions[i].end()) {
                odrv.misconfigured_ = true; // this GPIO doesn't support the selected mode
                continue;
            }
            GPIO_InitStruct.Alternate = it->alternate_function;
        }

        switch (mode) {
            case ODriveIntf::GPIO_MODE_DIGITAL: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_DIGITAL_PULL_UP: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ANALOG_IN: {
                GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
            } break;
            case ODriveIntf::GPIO_MODE_UART_A: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart_a) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_UART_B: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart_b) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_UART_C: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart_c) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_CAN_A: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_can_a) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_I2C_A: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_i2c_a) {
                    odrv.misconfigured_ = true;
                }
            } break;
            //case ODriveIntf::GPIO_MODE_SPI_A: { // TODO
            //} break;
            case ODriveIntf::GPIO_MODE_PWM: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC1: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC2: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_MECH_BRAKE: {
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_STATUS: {
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            default: {
                odrv.misconfigured_ = true;
                continue;
            }
        }

        HAL_GPIO_Init(get_gpio(i).port_, &GPIO_InitStruct);
    }

    // Init usb irq binary semaphore, and start with no tokens by removing the starting one.
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
    osSemaphoreWait(sem_usb_irq, 0);

    // Create an event queue for UART
    osMessageQDef(uart_event_queue, 4, uint32_t);
    uart_event_queue = osMessageCreate(osMessageQ(uart_event_queue), NULL);

    // Create an event queue for USB
    osMessageQDef(usb_event_queue, 7, uint32_t);
    usb_event_queue = osMessageCreate(osMessageQ(usb_event_queue), NULL);

    osSemaphoreDef(sem_can);
    sem_can = osSemaphoreCreate(osSemaphore(sem_can), 1);
    osSemaphoreWait(sem_can, 0);

    // Create main thread
    osThreadDef(defaultTask, rtos_main, osPriorityNormal, 0, stack_size_default_task / sizeof(StackType_t));
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // Start scheduler
    osKernelStart();
    
    for (;;);
}
