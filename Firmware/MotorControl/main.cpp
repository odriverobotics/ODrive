
#include "odrive_main.hpp"
#include <nvm_config.hpp>


EncoderConfig_t encoder_configs[AXIS_COUNT];
ControllerConfig_t controller_configs[AXIS_COUNT];
MotorConfig_t motor_configs[AXIS_COUNT];
AxisConfig_t axis_configs[AXIS_COUNT];
Axis *axes[AXIS_COUNT];

bool enable_uart;

typedef Config<AxisConfig_t[2], MotorConfig_t[2], float, bool> ConfigFormat;

void save_configuration(void) {
    if (ConfigFormat::safe_store_config(
        &axis_configs,
        &motor_configs,
        &brake_resistance,
        &enable_uart)) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    }
}

void load_configuration(void) {
    if (NVM_init() ||
        ConfigFormat::safe_load_config(
                &axis_configs,
                &motor_configs,
                &brake_resistance,
                &enable_uart)) {
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            axis_configs[i] = AxisConfig_t();
            motor_configs[i] = MotorConfig_t();
        }
        brake_resistance = 0.47f;
        enable_uart = true;
    }
}

void erase_configuration(void) {
    NVM_erase();
}

extern "C" {
int odrive_main(void);
}

int odrive_main(void) {
    // Load persistent configuration (or defaults)
    load_configuration();

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
    
    // TODO: make dynamically reconfigurable
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (enable_uart) {
        axes[0]->config.enable_step_dir = false;
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

    return 0;
}
