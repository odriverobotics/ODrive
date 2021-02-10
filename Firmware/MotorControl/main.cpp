
#define __MAIN_CPP__
#include "odrive_main.h"

#include "freertos_vars.h"
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

static bool config_read_all(size_t* size) {
    if (!board.nvm.open_read(size)) {
        return false;
    }
    bool success = 
           board.nvm.read(&odrv.config_) &&
           board.nvm.read(&odrv.can_.config_);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = board.nvm.read(&encoders[i].config_) &&
                  board.nvm.read(&axes[i].sensorless_estimator_.config_) &&
                  board.nvm.read(&axes[i].controller_.config_) &&
                  board.nvm.read(&axes[i].trap_traj_.config_) &&
                  board.nvm.read(&axes[i].min_endstop_.config_) &&
                  board.nvm.read(&axes[i].max_endstop_.config_) &&
                  board.nvm.read(&axes[i].mechanical_brake_.config_) &&
                  board.nvm.read(&motors[i].config_) &&
                  board.nvm.read(&motors[i].fet_thermistor_.config_) &&
                  board.nvm.read(&motors[i].motor_thermistor_.config_) &&
                  board.nvm.read(&axes[i].config_);
    }

    if (!board.nvm.close()) {
        success = false;
    }

    return success;
}

static bool config_write_all() {
    bool success = 
           board.nvm.write(&odrv.config_) &&
           board.nvm.write(&odrv.can_.config_);
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = board.nvm.write(&encoders[i].config_) &&
                  board.nvm.write(&axes[i].sensorless_estimator_.config_) &&
                  board.nvm.write(&axes[i].controller_.config_) &&
                  board.nvm.write(&axes[i].trap_traj_.config_) &&
                  board.nvm.write(&axes[i].min_endstop_.config_) &&
                  board.nvm.write(&axes[i].max_endstop_.config_) &&
                  board.nvm.write(&axes[i].mechanical_brake_.config_) &&
                  board.nvm.write(&motors[i].config_) &&
                  board.nvm.write(&motors[i].fet_thermistor_.config_) &&
                  board.nvm.write(&motors[i].motor_thermistor_.config_) &&
                  board.nvm.write(&axes[i].config_);
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

        board.nvm.erase(); // returns false if already erased
        success = board.nvm.open_write()
               && config_write_all()
               && board.nvm.close();

        // FIXME: during save_configuration we might miss some interrupts
        // because the CPU gets halted during a flash erase. Missing events
        // (encoder updates, step/dir steps) is not good so to be sure we just
        // reboot.
        NVIC_SystemReset();
    }

    return success;
}

void ODrive::erase_configuration(void) {
    if (!board.nvm.erase()) {
        return;
    }

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
        brake_resistor_.arm();
    }
}

// TODO: this could probably be part of the main control loop
static void analog_polling_thread(void *) {
    while (true) {
        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &odrv.config_.analog_mappings[i];

            if (fibre::is_endpoint_ref_valid(map->endpoint)) {
                float ratio = board.gpio_adc_values[i];
                float value = map->min + (ratio * (map->max - map->min));
                fibre::set_endpoint_from_float(map->endpoint, value);
            }
        }

        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &odrv.config_.pwm_mappings[i];

            if (fibre::is_endpoint_ref_valid(map->endpoint)) {
                float ratio = board.gpio_pwm_values[i];
                float value = map->min + (ratio * (map->max - map->min));
                fibre::set_endpoint_from_float(map->endpoint, value);
            }
        }
        osDelay(10);
    }
}


extern "C" {

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for(auto& axis: axes){
        axis.motor_.disarm();
    }
    odrv.brake_resistor_.disarm();
    for (;;); // TODO: safe action
}

void vApplicationIdleHook(void) {
    if (odrv.system_stats_.fully_booted) {
        odrv.system_stats_.uptime = HAL_GetTick(); //xTaskGetTickCount();
        odrv.system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();

        uint32_t min_stack_space[AXIS_COUNT];
        std::transform(axes.begin(), axes.end(), std::begin(min_stack_space), [](auto& axis) { return uxTaskGetStackHighWaterMark(axis.thread_id_) * sizeof(StackType_t); });
        odrv.system_stats_.max_stack_usage_axis = axes[0].stack_size_ - *std::min_element(std::begin(min_stack_space), std::end(min_stack_space));
        odrv.system_stats_.max_stack_usage_usb = stack_size_usb_thread - uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_uart = stack_size_uart_thread - uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_startup = stack_size_default_task - uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_can = odrv.can_.stack_size_ - uxTaskGetStackHighWaterMark(odrv.can_.thread_id_) * sizeof(StackType_t);

        odrv.system_stats_.stack_size_axis = axes[0].stack_size_;
        odrv.system_stats_.stack_size_usb = stack_size_usb_thread;
        odrv.system_stats_.stack_size_uart = stack_size_uart_thread;
        odrv.system_stats_.stack_size_startup = stack_size_default_task;
        odrv.system_stats_.stack_size_can = odrv.can_.stack_size_;

        odrv.system_stats_.prio_axis = osThreadGetPriority(axes[0].thread_id_);
        odrv.system_stats_.prio_usb = osThreadGetPriority(usb_thread);
        odrv.system_stats_.prio_uart = osThreadGetPriority(uart_thread);
        odrv.system_stats_.prio_startup = osThreadGetPriority(defaultTaskHandle);
        odrv.system_stats_.prio_can = osThreadGetPriority(odrv.can_.thread_id_);

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
    if (!(vbus_voltage_ >= config_.dc_bus_undervoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_UNDER_VOLTAGE);
    if (!(vbus_voltage_ <= config_.dc_bus_overvoltage_trip_level))
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
        odrv.brake_resistor_.disarm();
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
#if DMA_SxCR_CHSEL_Msk
    uint8_t channel = ((stream->CR & DMA_SxCR_CHSEL_Msk) >> DMA_SxCR_CHSEL_Pos);
#else
    int stream_number = (((uint32_t)((uint32_t*)stream) & 0xFFU) - 16U) / 24U;
    DMAMUX_Channel_TypeDef* dmamux = (DMAMUX_Channel_TypeDef *)((uint32_t)(((uint32_t)DMAMUX1_Channel0) + (stream_number * 4U)));
    uint8_t channel = ((dmamux->CCR & DMAMUX_CxCR_DMAREQ_ID_Msk) >> DMAMUX_CxCR_DMAREQ_ID_Pos);
#endif
    uint8_t priority = ((stream->CR & DMA_SxCR_PL_Msk) >> DMA_SxCR_PL_Pos);
    return (is_reset ? 0 : 0x80000000) | ((channel & 0x7) << 2) | (priority & 0x3);
}

uint32_t ODrive::get_gpio_states() {
    // TODO: get values that were sampled synchronously with the control loop
    uint32_t val = 0;
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        val |= ((board.gpios[i].read() ? 1UL : 0UL) << i);
    }
    return val;
}

/**
 * @brief Main thread started from main().
 */
static void rtos_main(void*) {
    // Init communications (this requires the axis objects to be constructed)
    init_communication();

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

    // Disarm motors
    for (auto& axis: axes) {
        axis.motor_.disarm();
    }

    for (Motor& motor: motors) {
        // Init PWM
        int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
        motor.timer_->Instance->CCR1 = half_load;
        motor.timer_->Instance->CCR2 = half_load;
        motor.timer_->Instance->CCR3 = half_load;

        // Enable PWM outputs (they are still masked by MOE though)
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_1);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_2);
        motor.timer_->Instance->CCER |= (TIM_CCx_ENABLE << TIM_CHANNEL_3);
        motor.timer_->Instance->CCER |= (TIM_CCxN_ENABLE << TIM_CHANNEL_3);
    }

    start_timers();

    if (odrv.config_.enable_brake_resistor) {
        odrv.brake_resistor_.arm();
    }

    // Start user ADC input thread
    osThreadDef(thread_def, analog_polling_thread, osPriorityLow, 0, 512 / sizeof(StackType_t));
    osThreadCreate(osThread(thread_def), NULL);

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
    // Init low level system functions (clocks, flash interface)
    if (!board.init()) {
        for (;;);
    }

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

    // Load configuration from NVM. This needs to happen after system_init()
    // since the flash interface must be initialized and before board_init()
    // since board initialization can depend on the config.
    size_t config_size = 0;
    bool success = config_read_all(&config_size)
            && config_apply_all();
    if (success) {
        odrv.user_config_loaded_ = config_size;
    } else {
        config_clear_all();
        config_apply_all();
    }


    BoardSupportPackage::BoardConfig config;

    auto maybe_use_gpio = [&](int gpio, int expected_mode){ 
        if (gpio >= 0 && odrv.config_.gpio_modes[gpio] == (ODrive::GpioMode)expected_mode) {
            return gpio;
        } else {
            return -1;
        }
    };

    bool enable_uart[] = {odrv.config_.enable_uart_a, odrv.config_.enable_uart_b, odrv.config_.enable_uart_c};
    uint32_t uart_baudrate[] = {odrv.config_.uart_a_baudrate, odrv.config_.uart_b_baudrate, odrv.config_.uart_c_baudrate};

    for (size_t i = 0; i < std::min(board.UART_COUNT, 3U); ++i) {
        config.uart_config[i].enabled = enable_uart[i];
        config.uart_config[i].baudrate = uart_baudrate[i];
        config.uart_config[i].tx_gpio = maybe_use_gpio(board.uart_tx_gpios[i], (int)ODriveIntf::GPIO_MODE_UART_A + i);
        config.uart_config[i].rx_gpio = maybe_use_gpio(board.uart_rx_gpios[i], (int)ODriveIntf::GPIO_MODE_UART_A + i);
    }

    for (size_t i = board.UART_COUNT; i < 3UL; ++i) {
        if (enable_uart[i]) {
            odrv.misconfigured_ = true;
        }
    }
    
    bool enable_can[] = {odrv.config_.enable_can_a};
    for (size_t i = 0; i < board.CANBUS_COUNT; ++i) {
        config.can_config[i].enabled = enable_can[i];
        config.can_config[i].r_gpio = maybe_use_gpio(board.can_r_gpios[i], (int)ODriveIntf::GPIO_MODE_CAN_A + i);
        config.can_config[i].d_gpio = maybe_use_gpio(board.can_d_gpios[i], (int)ODriveIntf::GPIO_MODE_CAN_A + i);
    }
    
    for (size_t i = 0; i < board.SPI_COUNT; ++i) {
        config.spi_config[i].enabled = !(HW_VERSION_MAJOR == 4); // always keep SPI enabled on v3.x, not supported yet on v4.x
        config.spi_config[i].miso_gpio = maybe_use_gpio(board.spi_miso_gpios[i], (int)ODriveIntf::GPIO_MODE_SPI_A + i);
        config.spi_config[i].mosi_gpio = maybe_use_gpio(board.spi_mosi_gpios[i], (int)ODriveIntf::GPIO_MODE_SPI_A + i);
        config.spi_config[i].sck_gpio = maybe_use_gpio(board.spi_sck_gpios[i], (int)ODriveIntf::GPIO_MODE_SPI_A + i);
    }
    
    for (size_t i = 0; i < board.INC_ENC_COUNT; ++i) {
        config.inc_enc_config[i].enabled = true; // always keep incremental encoders enabled
        config.inc_enc_config[i].a_gpio = maybe_use_gpio(board.inc_enc_a_gpios[i], (int)ODriveIntf::GPIO_MODE_ENC0 + i);
        config.inc_enc_config[i].b_gpio = maybe_use_gpio(board.inc_enc_b_gpios[i], (int)ODriveIntf::GPIO_MODE_ENC0 + i);
    }

    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        switch (odrv.config_.gpio_modes[i]) {
            case ODriveIntf::GPIO_MODE_DIGITAL: config.gpio_modes[i] = BoardSupportPackage::kDigitalInputNoPull; break;
            case ODriveIntf::GPIO_MODE_DIGITAL_PULL_UP: config.gpio_modes[i] = BoardSupportPackage::kDigitalInputPullUp; break;
            case ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN: config.gpio_modes[i] = BoardSupportPackage::kDigitalInputPullDown; break;
            case ODriveIntf::GPIO_MODE_ANALOG_IN: config.gpio_modes[i] = BoardSupportPackage::kAnalogInput; break;
            case ODriveIntf::GPIO_MODE_UART_A: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_UART_B: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_UART_C: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_CAN_A: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_I2C_A: odrv.misconfigured_ = true; break;
            case ODriveIntf::GPIO_MODE_SPI_A: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_PWM: config.gpio_modes[i] = BoardSupportPackage::kPwmInput; break;
            case ODriveIntf::GPIO_MODE_ENC0: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_ENC1: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_ENC2: config.gpio_modes[i] = BoardSupportPackage::kAlternateFunction; break;
            case ODriveIntf::GPIO_MODE_MECH_BRAKE: config.gpio_modes[i] = BoardSupportPackage::kDigitalOutput; break;
            case ODriveIntf::GPIO_MODE_STATUS: config.gpio_modes[i] = BoardSupportPackage::kDigitalOutput; break;
            case ODriveIntf::GPIO_MODE_BRAKE_RES: odrv.misconfigured_ = true; break;
            default: odrv.misconfigured_ = true; break;
        }
    }


    // Init board-specific peripherals
    if (!board.config(config)) {
        // Currently it's not safe to continue in degraded mode because the
        // firmware would use some uninitialized variables. For now we just
        // clear the NVM to make ODrive boot again.
        // This will cause a reboot if the NVM was not cleared already.
        odrv.erase_configuration();
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
