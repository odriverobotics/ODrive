
#define __MAIN_CPP__
#include "odrive_main.h"
#include "nvm_config.hpp"

#include "freertos_vars.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>

BoardConfig_t board_config;
Encoder::Config_t encoder_configs[AXIS_COUNT];
ControllerConfig_t controller_configs[AXIS_COUNT];
MotorConfig_t motor_configs[AXIS_COUNT];
AxisConfig_t axis_configs[AXIS_COUNT];
bool user_config_loaded_;

SystemStats_t system_stats_ = { 0 };

Axis *axes[AXIS_COUNT];

typedef Config<
    BoardConfig_t,
    Encoder::Config_t[AXIS_COUNT],
    ControllerConfig_t[AXIS_COUNT],
    MotorConfig_t[AXIS_COUNT],
    AxisConfig_t[AXIS_COUNT]> ConfigFormat;

void save_configuration(void) {
    if (ConfigFormat::safe_store_config(
            &board_config,
            &encoder_configs,
            &controller_configs,
            &motor_configs,
            &axis_configs)) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    } else {
        user_config_loaded_ = true;
    }
}

void load_configuration(void) {
    // Try to load configs
    if (NVM_init() ||
        ConfigFormat::safe_load_config(
                &board_config,
                &encoder_configs,
                &controller_configs,
                &motor_configs,
                &axis_configs)) {
        //If loading failed, restore defaults
        board_config = BoardConfig_t();
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            encoder_configs[i] = Encoder::Config_t();
            controller_configs[i] = ControllerConfig_t();
            motor_configs[i] = MotorConfig_t();
            axis_configs[i] = AxisConfig_t();
        }
    } else {
        user_config_loaded_ = true;
    }
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
int odrive_main(void);
void vApplicationStackOverflowHook(void) {
    for (;;); // TODO: safe action
}
void vApplicationIdleHook(void) {
    if (system_stats_.fully_booted) {
        system_stats_.uptime = xTaskGetTickCount();
        system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();
        system_stats_.min_stack_space_comms = uxTaskGetStackHighWaterMark(comm_thread) * sizeof(StackType_t);
        system_stats_.min_stack_space_axis0 = uxTaskGetStackHighWaterMark(axes[0]->thread_id_) * sizeof(StackType_t);
        system_stats_.min_stack_space_axis1 = uxTaskGetStackHighWaterMark(axes[1]->thread_id_) * sizeof(StackType_t);
        system_stats_.min_stack_space_usb = uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
        system_stats_.min_stack_space_uart = uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
        system_stats_.min_stack_space_usb_irq = uxTaskGetStackHighWaterMark(usb_irq_thread) * sizeof(StackType_t);
        system_stats_.min_stack_space_startup = uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
    }
}
}

int odrive_main(void) {
    // Load persistent configuration (or defaults)
    load_configuration();

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_i2c_instead_of_can) {
        // Set up the direction GPIO as input
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;

        GPIO_InitStruct.Pin = I2C_A0_PIN;
        HAL_GPIO_Init(I2C_A0_PORT, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = I2C_A1_PIN;
        HAL_GPIO_Init(I2C_A1_PORT, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = I2C_A2_PIN;
        HAL_GPIO_Init(I2C_A2_PORT, &GPIO_InitStruct);

        osDelay(1);
        i2c_stats_.addr = (0xD << 3);
        i2c_stats_.addr |= HAL_GPIO_ReadPin(I2C_A0_PORT, I2C_A0_PIN) != GPIO_PIN_RESET ? 0x1 : 0;
        i2c_stats_.addr |= HAL_GPIO_ReadPin(I2C_A1_PORT, I2C_A1_PIN) != GPIO_PIN_RESET ? 0x2 : 0;
        i2c_stats_.addr |= HAL_GPIO_ReadPin(I2C_A2_PORT, I2C_A2_PIN) != GPIO_PIN_RESET ? 0x4 : 0;
        MX_I2C1_Init(i2c_stats_.addr);
    } else
#endif
        MX_CAN1_Init();

    // Init general user ADC on some GPIOs.
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_1_Pin;
    HAL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_2_Pin;
    HAL_GPIO_Init(GPIO_2_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_3_Pin;
    HAL_GPIO_Init(GPIO_3_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_4_Pin;
    HAL_GPIO_Init(GPIO_4_GPIO_Port, &GPIO_InitStruct);
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
    GPIO_InitStruct.Pin = GPIO_5_Pin;
    HAL_GPIO_Init(GPIO_5_GPIO_Port, &GPIO_InitStruct);
#endif

    // Construct all objects.
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        Encoder *encoder = new Encoder(hw_configs[i].encoder_config,
                                       encoder_configs[i]);
        SensorlessEstimator *sensorless_estimator = new SensorlessEstimator();
        Controller *controller = new Controller(controller_configs[i]);
        Motor *motor = new Motor(hw_configs[i].motor_config,
                                 hw_configs[i].gate_driver_config,
                                 motor_configs[i]);
        axes[i] = new Axis(hw_configs[i].axis_config, axis_configs[i],
                *encoder, *sensorless_estimator, *controller, *motor);
    }
    
    // Start ADC for temperature measurements and user measurements
    start_general_purpose_adc();

    // TODO: make dynamically reconfigurable
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_uart) {
        axes[0]->config_.enable_step_dir = false;
        axes[0]->set_step_dir_enabled(false);
        SetGPIO12toUART();
    }
#endif
    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed)
    init_communication();

    // Setup hardware for all components
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i]->setup();
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

    system_stats_.fully_booted = true;
    return 0;
}
