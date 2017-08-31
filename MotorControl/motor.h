#pragma once

#include <cmsis_os.h>
#include <configuration.h>
#include <constants.h>
#include <drv8301.h>
#include <encoder.h>
#include <error.h>
#include <sensorless.h>

#define MAX_NUM_MOTORS  (2)
#define TIMING_LOG_SIZE (16)

typedef enum {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
} Motor_thread_signals_t;

// Note: these should be sorted from lowest level of control to
// highest level of control, to allow "<" style comparisons.
typedef enum {
    CTRL_MODE_VOLTAGE_CONTROL,
    CTRL_MODE_CURRENT_CONTROL,
    CTRL_MODE_VELOCITY_CONTROL,
    CTRL_MODE_POSITION_CONTROL
} Motor_control_mode_t;

typedef struct
{
    float phB;
    float phC;
} Iph_BC_t;

typedef struct
{
    float current_lim;                   // [A]
    float p_gain;                        // [V/A]
    float i_gain;                        // [V/As]
    float v_current_control_integral_d;  // [V]
    float v_current_control_integral_q;  // [V]
    float Ibus;                          // DC bus current [A]
    // Voltage applied at end of cycle:
    float final_v_alpha;  // [V]
    float final_v_beta;   // [V]
} Current_control_t;

typedef enum {
    ROTOR_MODE_ENCODER,
    ROTOR_MODE_SENSORLESS,
    ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS  //Run on encoder, but still run estimator for testing
} Rotor_mode_t;

class Motor {
   public:
    Motor();

    Motor_control_mode_t control_mode;
    bool enable_step_dir;
    float counts_per_step;
    int error;
    float pos_setpoint;
    float pos_gain;
    float vel_setpoint;
    float vel_gain;
    float vel_integrator_gain;
    float vel_integrator_current;
    float vel_limit;
    float current_setpoint;
    float calibration_current;
    float phase_inductance;
    float phase_resistance;
    osThreadId motor_thread;
    bool thread_ready;
    bool enable_control;  // enable/disable via usb to start motor control. will be set to false again in case of errors.requires calibration_ok=true
    bool do_calibration;  //  trigger motor calibration. will be reset to false after self test
    bool calibration_ok;
    TIM_HandleTypeDef *motor_timer;
    uint16_t next_timings[3];
    uint16_t control_deadline;
    uint16_t last_cpu_time;
    Iph_BC_t current_meas;
    Iph_BC_t DC_calib;
    DRV8301_Obj gate_driver;
    DRV_SPI_8301_Vars_t gate_driver_regs;  //Local view of DRV registers
    float shunt_conductance;
    float phase_current_rev_gain;  //Reverse gain for ADC to Amps
    Current_control_t current_control;
    Rotor_mode_t rotor_mode;
    Encoder_t encoder;
    ODrive_Sensorless_t sensorless;
    int timing_log_index;
    uint16_t timing_log[TIMING_LOG_SIZE];

    // Functions
    bool calibrateMotor();
    void calculateCurrentGains();
    bool calculatePLLGains();

    bool measurePhaseResistance(float test_current, float max_voltage);
    bool measurePhaseInductance(float voltage_low, float voltage_high);
    bool calibrateEncoderOffset(float voltage_magnitude);

    uint16_t checkTiming();

    void queueVoltageTimings(float v_alpha, float v_beta);
    void queueModulationTimings(float mod_alpha, float mod_beta);

    void scanMotorLoop(float omega, float voltage_magnitude);

    bool checkDeadlines();
    
}

#ifdef __cplusplus
extern "C" {
#endif


    extern Motor_t motors[MAX_NUM_MOTORS];
    extern const int numMotors;

#ifdef __cplusplus
}
#endif