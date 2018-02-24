
/* Includes ------------------------------------------------------------------*/

#include "config.h"

#include <stdint.h>
#include <stdlib.h>
#include <stm32f405xx.h>

#include "nvm.h"
#include "crc.hpp"
#include "low_level.h"
#include "axis.h"

/* Private defines -----------------------------------------------------------*/
#define CRC16_INIT 0xabcd

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

typedef struct {
    Motor_control_mode_t control_mode;
    float counts_per_step;
    int32_t pole_pairs;
    float pos_gain;
    float vel_gain;
    float vel_integrator_gain;
    float vel_limit;
    float calibration_current;
    float resistance_calib_max_voltage;
    float phase_inductance;
    float phase_resistance;
    Motor_type_t motor_type;
    Rotor_mode_t rotor_mode;
    float current_control_current_lim;
    bool encoder_use_index;
    bool encoder_calibrated;
    float encoder_idx_search_speed;
    int32_t encoder_cpr;
    int32_t encoder_offset;
    int32_t motor_dir;
} MotorConfig_t;

/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

template<typename ... Ts>
struct Config;

template<>
struct Config<> {
    static size_t get_size() {
        return 0;
    }
    static int load_config(size_t offset, uint16_t* crc16) {
        return 0;
    }
    static int store_config(size_t offset, uint16_t* crc16) {
        return 0;
    }
};

template<typename T, typename ... Ts>
struct Config<T, Ts...> {
    static size_t get_size() {
        return sizeof(T) + Config<Ts...>::get_size();
    }

    static int load_config(size_t offset, uint16_t* crc16, T* val0, Ts* ... vals) {
        size_t size = sizeof(T);
        // save current CRC (in case val0 and crc16 point to the same address)
        size_t previous_crc16 = *crc16;
        if (NVM_read(offset, (uint8_t *)val0, size))
            return -1;
        *crc16 = calc_crc16(previous_crc16, (uint8_t *)val0, size);
        if (Config<Ts...>::load_config(offset + size, crc16, vals...))
            return -1;
        return 0;
    }

    static int store_config(size_t offset, uint16_t* crc16, const T* val0, const Ts* ... vals) {
        size_t size = sizeof(T);
        if (NVM_write(offset, (uint8_t *)val0, size))
            return -1;
        // update CRC _after_ writing (in case val0 and crc16 point to the same address)
        if (crc16)
            *crc16 = calc_crc16(*crc16, (uint8_t *)val0, size);
        if (Config<Ts...>::store_config(offset + size, crc16, vals...))
            return -1;
        return 0;
    }

    static int load_config(T* val0, Ts* ... vals) {
        //printf("have %d bytes\r\n", NVM_get_max_read_length()); osDelay(5);
        if (Config<T, Ts..., uint16_t>::get_size() > NVM_get_max_read_length())
            return -1;
        uint16_t crc16 = CRC16_INIT;
        if (Config<T, Ts..., uint16_t>::load_config(0, &crc16, val0, vals..., &crc16))
            return -1;
        if (crc16)
            return -1;
        return 0;
    }

    static int store_config(const T* val0, const Ts* ... vals) {
        size_t size = Config<T, Ts...>::get_size() + 2;
        //printf("config is %d bytes\r\n", size); osDelay(5);
        if (size > NVM_get_max_write_length())
            return -1;
        if (NVM_start_write(size))
            return -1;
        uint16_t crc16 = CRC16_INIT; // TODO: this is not a sufficient method to protect against changes of the struct signature
        if (Config<T, Ts...>::store_config(0, &crc16, val0, vals...))
            return -1;
        if (Config<uint8_t, uint8_t>::store_config(size - 2, nullptr, (uint8_t *)&crc16 + 1, (uint8_t *)&crc16))
            return -1;
        if (NVM_commit())
            return -1;
        return 0;
    }
};

// This function is obviously stupid and should go away (make MotorConfig_t a member of Motor_t)
// TODO: make this go away as part of the C++ refactoring
void set_motor_config(const MotorConfig_t* config, Motor_t* motor) {
    motor->control_mode = config->control_mode;
    motor->counts_per_step = config->counts_per_step;
    motor->pole_pairs = config->pole_pairs;
    motor->pos_gain = config->pos_gain;
    motor->vel_gain = config->vel_gain;
    motor->vel_integrator_gain = config->vel_integrator_gain;
    motor->vel_limit = config->vel_limit;
    motor->calibration_current = config->calibration_current;
    motor->resistance_calib_max_voltage = config->resistance_calib_max_voltage;
    motor->phase_inductance = config->phase_inductance;
    motor->phase_resistance = config->phase_resistance;
    motor->motor_type = config->motor_type;
    motor->rotor_mode = config->rotor_mode;

    motor->current_control.current_lim = config->current_control_current_lim;

    motor->encoder.use_index = config->encoder_use_index;
    motor->encoder.calibrated = config->encoder_calibrated;
    motor->encoder.idx_search_speed = config->encoder_idx_search_speed;
    motor->encoder.encoder_cpr = config->encoder_cpr;
    motor->encoder.encoder_offset = config->encoder_offset;
    motor->encoder.motor_dir = config->motor_dir;
}

// This function is obviously stupid and should go away (make MotorConfig_t a member of Motor_t)
// TODO: make this go away as part of the C++ refactoring
void get_motor_config(const Motor_t* motor, MotorConfig_t* config) {
    config->control_mode = motor->control_mode;
    config->counts_per_step = motor->counts_per_step;
    config->pole_pairs = motor->pole_pairs;
    config->pos_gain = motor->pos_gain;
    config->vel_gain = motor->vel_gain;
    config->vel_integrator_gain = motor->vel_integrator_gain;
    config->vel_limit = motor->vel_limit;
    config->calibration_current = motor->calibration_current;
    config->resistance_calib_max_voltage = motor->resistance_calib_max_voltage;
    config->phase_inductance = motor->phase_inductance;
    config->phase_resistance = motor->phase_resistance;
    config->motor_type = motor->motor_type;
    config->rotor_mode = motor->rotor_mode;

    config->current_control_current_lim = motor->current_control.current_lim;

    config->encoder_use_index = motor->encoder.use_index;
    config->encoder_calibrated = motor->encoder.calibrated;
    config->encoder_idx_search_speed = motor->encoder.idx_search_speed;
    config->encoder_cpr = motor->encoder.encoder_cpr;
    config->encoder_offset = motor->encoder.encoder_offset;
    config->motor_dir = motor->encoder.motor_dir;
}


void init_configuration(void) {
    MotorConfig_t motor_config[2];
    //TODO: we really shouldn't be hardcoding like this
    if (NVM_init() || Config<MotorConfig_t, MotorConfig_t, AxisConfig, AxisConfig>::load_config(&motor_config[0], &motor_config[1], &axis_configs[0], &axis_configs[1])) {
        //printf("no config found\r\n"); osDelay(5);
        // load default config
        // motor_config[0] = MotorConfig_t();
        // motor_config[1] = MotorConfig_t();

        // Default config coming from flashed Motor_t
        return;
    } else {
        //printf("load config successful\r\n"); osDelay(5);
    }
    
    set_motor_config(&motor_config[0], &motors[0]);
    set_motor_config(&motor_config[1], &motors[1]);
}

void save_configuration(void) {
    MotorConfig_t motor_config[2];
    get_motor_config(&motors[0], &motor_config[0]);
    get_motor_config(&motors[1], &motor_config[1]);
    //TODO: we really shouldn't be hardcoding like this
    if (Config<MotorConfig_t, MotorConfig_t, AxisConfig, AxisConfig>::store_config(&motor_config[0], &motor_config[1], &axis_configs[0], &axis_configs[1])) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    }
}

void erase_configuration(void) {
    NVM_erase();
}
