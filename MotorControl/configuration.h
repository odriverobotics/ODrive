#pragma once

// Mandatory Parameters
#define ENCODER_PPR (600)
#define POLE_PAIRS (7.0f)
#define BRAKE_RESISTANCE (0.47f)

// Mode Parameters
#define AXIS_0_ENABLED (true)
#define AXIS_1_ENABLED (true)

#define AXIS_0_DO_CALIBRATION (false)
#define AXIS_1_DO_CALIBRATION (false)

/**
 *  Options for control modes:
 *
 *  CTRL_MODE_VOLTAGE_CONTROL
 *  CTRL_MODE_CURRENT_CONTROL
 *  CTRL_MODE_VELOCITY_CONTROL
 *  CTRL_MODE_POSITION_CONTROL
 *
 */
#define AXIS_0_CONTROL_MODE (CTRL_MODE_POSITION_CONTROL)
#define AXIS_1_CONTROL_MODE (CTRL_MODE_POSITION_CONTROL)

/**
 * Changes the feedback mode from the motor
 *
 * ROTOR_MODE_ENCODER
 * ROTOR_MODE_SENSORLESS
 *
 * Position control mode is unavailable in sensorless rotor mode
 */
#define AXIS_0_ROTOR_MODE (ROTOR_MODE_ENCODER)
#define AXIS_1_ROTOR_MODE (ROTOR_MODE_ENCODER)

// Tuning Parameters
#define AXIS_0_Pos_Kp (20.0f)
#define AXIS_0_Vel_Kp (15.0f / 10000.0f)
#define AXIS_0_Vel_Ki (10.0f / 10000.0f)

#define AXIS_1_Pos_Kp (20.0f)
#define AXIS_1_Vel_Kp (15.0f / 10000.0f)
#define AXIS_1_Vel_Ki (10.0f / 10000.0f)

// Communication Parameters
#define STANDALONE_MODE  // Drive operates without USB communication

// Used internally.  Do not modify

#define PH_CURRENT_MEAS_TIMEOUT (2)  // [ms]
#define ENCODER_CPR (ENCODER_PPR * 4)