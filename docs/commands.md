# Parameters & Commands

We will use the `<odrv>` as a placeholder for any ODrive object. Every ODrive controller is an ODrive object. In `odrivetool` this is usually `odrv0`. Furthermore we use `<axis>` as a placeholder for any axis, which is an attribute of an ODrive object (for example `odrv0.axis0`). An axis represents where the motors are connected. (axis0 for M0 or axis1 for M1)

### Table of contents
<!-- TOC depthFrom:2 depthTo:2 -->

- [Per-Axis commands](#per-axis-commands)
- [System monitoring commands](#system-monitoring-commands)
- [General system commands](#general-system-commands)
- [Setting up sensorless](#setting-up-sensorless)

<!-- /TOC -->

## Per-Axis commands

For the most part, both axes on the ODrive can be controlled independently.

### State Machine

The current state of an axis is indicated by `<axis>.current_state`. The user can request a new state by assigning a new value to `<axis>.requested_state`. The default state after startup is `AXIS_STATE_IDLE`.

 1. `AXIS_STATE_IDLE` Disable motor PWM and do nothing.
 2. `AXIS_STATE_STARTUP_SEQUENCE` Run the [startup procedure](#startup-procedure).
 3. `AXIS_STATE_FULL_CALIBRATION_SEQUENCE` Run motor calibration and then encoder offset calibration (or encoder index search if `<axis>.encoder.config.use_index` is `True`).
 4. `AXIS_STATE_MOTOR_CALIBRATION` Measure phase resistance and phase inductance of the motor.
    * To store the results set `<axis>.motor.config.pre_calibrated` to `True` and [save the configuration](#saving-the-configuration). After that you don't have to run the motor calibration on the next start up.
    * This modifies the variables `<axis>.motor.config.phase_resistance` and `<axis>.motor.config.phase_inductance`.
 5. `AXIS_STATE_SENSORLESS_CONTROL` Run sensorless control.
    * The motor must be calibrated (`<axis>.motor.is_calibrated`)
    * [`<axis>.controller.control_mode`](#control-mode) must be `True`.
 6. `AXIS_STATE_ENCODER_INDEX_SEARCH` Turn the motor in one direction until the encoder index is traversed. This state can only be entered if `<axis>.encoder.config.use_index` is `True`.
 7. `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase.
    * Can only be entered if the motor is calibrated (`<axis>.motor.is_calibrated`).
    * A successful encoder calibration will make the `<axis>.encoder.is_ready` go to true.
 8. `AXIS_STATE_CLOSED_LOOP_CONTROL` Run closed loop control.
    * The action depends on the [control mode](#control-mode).
    * Can only be entered if the motor is calibrated (`<axis>.motor.is_calibrated`) and the encoder is ready (`<axis>.encoder.is_ready`).

### Startup Procedure

By default the ODrive takes no action at startup and goes to idle immediately.
In order to change what startup procedures are used, set the startup procedures you want to `True`.
The ODrive will sequence all enabled startup actions selected in the order shown below.

* `<axis>.config.startup_motor_calibration`
* `<axis>.config.startup_encoder_index_search`
* `<axis>.config.startup_encoder_offset_calibration`
* `<axis>.config.startup_closed_loop_control`
* `<axis>.config.startup_sensorless_control`

See [state machine](#state-machine) for a description of each state.

### Control Mode
The default control mode is position control.
If you want a different mode, you can change `<axis>.controller.config.control_mode`.
Possible values are:
* `CTRL_MODE_POSITION_CONTROL`
* `CTRL_MODE_VELOCITY_CONTROL`
* `CTRL_MODE_CURRENT_CONTROL`
* `CTRL_MODE_VOLTAGE_CONTROL` - this one is not normally used.

# Control Commands

* `<axis>.controller.pos_setpoint = <encoder_counts>`
* `<axis>.controller.current_setpoint = <current_in_A>`
* `<axis>.controller.vel_setpoint = <encoder_counts/s>`

### Tuning parameters
The motion control gains are currently manually tuned:
* `<axis>.controller.config.pos_gain = 20.0f` [(counts/s) / counts]
* `<axis>.controller.config.vel_gain = 5.0f / 10000.0f` [A/(counts/s)]
* `<axis>.controller.config.vel_integrator_gain = 10.0f / 10000.0f` [A/((counts/s) * s)]

An upcoming feature will enable automatic tuning. Until then, here is a rough tuning procedure:
* Set the integrator gain to 0
* Make sure you have a stable system. If it is not, decrease all gains until you have one.
* Increase `vel_gain` by around 30% per iteration until the motor exhibits some vibration.
* Back down `vel_gain` to 50% of the vibrating value.
* Increase `pos_gain` by around 30% per iteration until you see some overshoot.
* Back down `pos_gain` until you do not have overshoot anymore.
* The integrator can be set to `0.5 * bandwidth * vel_gain`, where `bandwidth` is the overall resulting tracking bandwidth of your system. Say your tuning made it track commands with a settling time of 100ms: this means the bandwidth was 1/100ms or 10. In this case you should set the `vel_integrator_gain = 0.5 * 10 * vel_gain`.

## System monitoring commands

### Encoder position and velocity
* View encoder position with `<axis>.encoder.pos_estimate` [counts]
* View rotational velocity with `<axis>.encoder.vel_estimate` [counts/s]

### Motor current and torque estimation
* View the commanded motor current with `<axis>.motor.current_control.Iq_setpoint` [A] 
* View the measured motor current with `<axis>.motor.current_control.Iq_measured` [A]. If you find that this returns noisy data then use the command motor current instead. The two values should be close so long as you are not approching the maximim achieveable rotational velocity of your motor for a given supply votlage, in which case the commanded current may become larger than the measured current. 

Using the motor current and the known KV of your motor you can estimate the motors torque using the following relationship: Torque [N.m] = 8.27 * Current [A] / KV. 

## General system commands

### Saving the configuration

All variables that are part of a `[...].config` object can be saved to non-volatile memory on the ODrive so they persist after you remove power. The relevant commands are:

 * `<odrv>.save_configuration()`: Stores the configuration to persistent memory on the ODrive.
 * `<odrv>.erase_configuration()`: Resets the configuration variables to their factory defaults. This only has an effect after a reboot. A side effect of this command is that motor control stops (in case it was running) and the USB communication breaks out temporarily. This is because erasing flash pages hangs the microcontroller for several seconds.

### Diagnostics

 * `<odrv>.serial_number`: A number that uniquely identifies your device. When printed in upper case hexadecimal (`hex(<odrv>.serial_number).upper()`), this is identical to the serial number indicated by the USB descriptor.
 * `<odrv>.fw_version_major`, `<odrv>.fw_version_minor`, `<odrv>.fw_version_revision`: The firmware version that is currently running.
 * `<odrv>.hw_version_major`, `<odrv>.hw_version_minor`, `<odrv>.hw_version_revision`: The hardware version of your ODrive.

## Setting up sensorless
The ODrive can run without encoder/hall feedback, but there is a minimum speed, usually around a few hunderd RPM.
However the units of this mode is different from when using an encoder. Velocities are not measured in counts/s, instead it is electrical rad/s. This also applies to the gains. For example, `vel_gain` is in units of `A / (rad/s)` instead of `A / (count/s)`.

To give an example, suppose you have a motor with 7 pole pairs, and you want to spin it at 3000 RPM. Then you would set the `vel_setpoint` to `3000 * 2*pi/60 * 7 = 2199 rad/s electrical`.

Below are some suggested starting parameters that you can use. Note that you _must_ set the `pm_flux_linkage` correctly for sensorless mode to work.

```
odrv0.axis0.controller.config.vel_gain = 0.01
odrv0.axis0.controller.config.vel_integrator_gain = 0.05
odrv0.axis0.controller.config.control_mode = 2
odrv0.axis0.controller.vel_setpoint = 400
odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (<pole pairs> * <motor kv>)
```

