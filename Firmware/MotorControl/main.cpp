
#define __MAIN_CPP__
#include "odrive_main.h"
#include "nvm_config.hpp"

#include "usart.h"
#include "freertos_vars.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>
#include <communication/interface_can.hpp>

BoardConfig_t board_config;
ODriveCAN::Config_t can_config;
Encoder::Config_t encoder_configs[AXIS_COUNT];
SensorlessEstimator::Config_t sensorless_configs[AXIS_COUNT];
Controller::Config_t controller_configs[AXIS_COUNT];
Motor::Config_t motor_configs[AXIS_COUNT];
Axis::Config_t axis_configs[AXIS_COUNT];
TrapezoidalTrajectory::Config_t trap_configs[AXIS_COUNT];
Endstop::Config_t min_endstop_configs[AXIS_COUNT];
Endstop::Config_t max_endstop_configs[AXIS_COUNT];
bool user_config_loaded_;

SystemStats_t system_stats_;

std::array<Axis*, AXIS_COUNT> axes;
ODriveCAN *odCAN = nullptr;

typedef Config<
    BoardConfig_t,
    ODriveCAN::Config_t,
    Encoder::Config_t[AXIS_COUNT],
    SensorlessEstimator::Config_t[AXIS_COUNT],
    Controller::Config_t[AXIS_COUNT],
    Motor::Config_t[AXIS_COUNT],
    TrapezoidalTrajectory::Config_t[AXIS_COUNT],
    Endstop::Config_t[AXIS_COUNT],
    Endstop::Config_t[AXIS_COUNT],
    Axis::Config_t[AXIS_COUNT]> ConfigFormat;

void save_configuration(void) {
    if (ConfigFormat::safe_store_config(
            &board_config,
            &can_config,
            &encoder_configs,
            &sensorless_configs,
            &controller_configs,
            &motor_configs,
            &trap_configs,
            &min_endstop_configs,
            &max_endstop_configs,
            &axis_configs)) {
        printf("saving configuration failed\r\n"); osDelay(5);
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
                &encoder_configs,
                &sensorless_configs,
                &controller_configs,
                &motor_configs,
                &trap_configs,
                &min_endstop_configs,
                &max_endstop_configs,
                &axis_configs)) {
        //If loading failed, restore defaults
        board_config = BoardConfig_t();
        can_config = ODriveCAN::Config_t();
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            encoder_configs[i] = Encoder::Config_t();
            sensorless_configs[i] = SensorlessEstimator::Config_t();
            controller_configs[i] = Controller::Config_t();
            motor_configs[i] = Motor::Config_t();
            trap_configs[i] = TrapezoidalTrajectory::Config_t();
            axis_configs[i] = Axis::Config_t();
            // Default step/dir pins are different, so we need to explicitly load them
            Axis::load_default_step_dir_pin_config(hw_configs[i].axis_config, &axis_configs[i]);
            Axis::load_default_can_id(i, axis_configs[i]);
            min_endstop_configs[i] = Endstop::Config_t();
            max_endstop_configs[i] = Endstop::Config_t();
            controller_configs[i].load_encoder_axis = i;
        }
    } else {
        user_config_loaded_ = true;
    }
    return user_config_loaded_;
}

void erase_configuration(void) {
    NVM_erase();

    // FIXME: this reboot is a workaround because we don't want the next save_configuration
    // to write back the old configuration from RAM to NVM. The proper action would
    // be to reset the values in RAM to default. However right now that's not
    // practical because several startup actions depend on the config. The
    // other problem is that the stack overflows if we reset to default here.
    NVIC_SystemReset();
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

extern "C" int construct_objects(){
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

    HAL_UART_DeInit(&huart4);
    huart4.Init.BaudRate = board_config.uart_baudrate;
    HAL_UART_Init(&huart4);

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
    odCAN = new ODriveCAN(&hcan1, can_config);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        Encoder *encoder = new Encoder(hw_configs[i].encoder_config,
                                       encoder_configs[i], motor_configs[i]);
        SensorlessEstimator *sensorless_estimator = new SensorlessEstimator(sensorless_configs[i]);
        Controller *controller = new Controller(controller_configs[i]);
        Motor *motor = new Motor(hw_configs[i].motor_config,
                                 hw_configs[i].gate_driver_config,
                                 motor_configs[i]);
        TrapezoidalTrajectory *trap = new TrapezoidalTrajectory(trap_configs[i]);
        Endstop *min_endstop = new Endstop(min_endstop_configs[i]);
        Endstop *max_endstop = new Endstop(max_endstop_configs[i]);
        axes[i] = new Axis(i, hw_configs[i].axis_config, axis_configs[i],
                *encoder, *sensorless_estimator, *controller, *motor, *trap, *min_endstop, *max_endstop);

    }
    initTree();      
    return 0;
}

extern "C" {
int odrive_main(void);
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
    for(auto& axis : axes){
        safety_critical_disarm_motor_pwm(axis->motor_);
    }
        safety_critical_disarm_brake_resistor();
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
        system_stats_.min_stack_space_can = uxTaskGetStackHighWaterMark(odCAN->thread_id_) * sizeof(StackType_t);

        // Actual usage, in bytes, so we don't have to math
        system_stats_.stack_usage_axis0 = axes[0]->stack_size_ - system_stats_.min_stack_space_axis0;
        system_stats_.stack_usage_axis1 = axes[1]->stack_size_ - system_stats_.min_stack_space_axis1;
        system_stats_.stack_usage_comms = stack_size_comm_thread - system_stats_.min_stack_space_comms;
        system_stats_.stack_usage_usb = stack_size_usb_thread - system_stats_.min_stack_space_usb;
        system_stats_.stack_usage_uart = stack_size_uart_thread - system_stats_.min_stack_space_uart;
        system_stats_.stack_usage_usb_irq = stack_size_usb_irq_thread - system_stats_.min_stack_space_usb_irq;
        system_stats_.stack_usage_startup = stack_size_default_task - system_stats_.min_stack_space_startup;
        system_stats_.stack_usage_can = odCAN->stack_size_ - system_stats_.min_stack_space_can;
    }
}
}

int odrive_main(void) {
    // Start ADC for temperature measurements and user measurements
  /* 
  for(auto& axis : axes){
        axis->encoder_.abs_spi_cs_pin_init();
    }
    */
    start_general_purpose_adc();

    // TODO: make dynamically reconfigurable
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (board_config.enable_uart) {
        SetGPIO12toUART();
    }
#endif
    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed)
    init_communication();

    // Start pwm-in compare modules
    // must happen after communication is initialized
    pwm_in_init();
    for(auto& axis : axes){
        axis->encoder_.setup();
    }
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
    osDelay(3000);

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i]->start_thread();
    }

    start_analog_thread();

    system_stats_.fully_booted = true;
    return 0;
}
