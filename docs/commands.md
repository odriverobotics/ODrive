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

The current state of an axis is indicated by [`<axis>.current_state`](api/odrive.axis#current_state). The user can request a new state by assigning a new value to [`<axis>.requested_state`](api/odrive.axis#current_state). The default state after startup is `AXIS_STATE_IDLE`. A description of all states can be found [here](api/odrive.axis.axisstate).


### Startup Procedure

By default the ODrive takes no action at startup and goes to idle immediately.
In order to change what startup procedures are used, set the startup procedures you want to `True`.
The ODrive will sequence all enabled startup actions selected in the order shown below.

* `<axis>.config.startup_motor_calibration`
* `<axis>.config.startup_encoder_index_search`
* `<axis>.config.startup_encoder_offset_calibration`
* `<axis>.config.startup_closed_loop_control`

See [here](api/odrive.axis.axisstate) for a description of each state.

### Control Mode
The default control mode is position control.
If you want a different mode, you can change `<axis>.controller.config.control_mode`.
Possible values are listed [here](api/odrive.controller.controlmode).

### Input Mode

As of version v0.5.0, ODrive now intercepts the incoming commands and can apply filters to them. The old protocol values `pos_setpoint`, `vel_setpoint`, and `current_setpoint` are still used internally by the closed-loop cascade control, but the user cannot write to them directly.  This allows us to condense the number of ways the ODrive accepts motion commands. The new commands are:

### Control Commands
* `<axis>.controller.input_pos = <turn>`
* `<axis>.controller.input_vel = <turn/s>`
* `<axis>.controller.input_torque = <torque in Nm>`

Modes can be selected by changing `<axis>.controller.config.input_mode`.
The default input mode is `INPUT_MODE_PASSTHROUGH`.
Possible values are listed [here](api/odrive.controller.inputmode).

## System monitoring commands

### Encoder position and velocity
* View encoder position with `<axis>.encoder.pos_estimate` [turns] or `<axis>.encoder.pos_est_counts` [counts]
* View rotational velocity with `<axis>.encoder.vel_estimate` [turn/s] or `<axis>.encoder.vel_est_counts` [count/s]

### Motor current and torque estimation
* View the commanded motor current with `<axis>.motor.current_control.Iq_setpoint` [A] 
* View the measured motor current with `<axis>.motor.current_control.Iq_measured` [A]. If you find that this returns noisy data then use the command motor current instead. The two values should be close so long as you are not approaching the maximum achievable rotational velocity of your motor for a given supply voltage, in which case the commanded current may become larger than the measured current. 

Using the motor current and the known KV of your motor you can estimate the motors torque using the following relationship: Torque [N.m] = 8.27 * Current [A] / KV. 

## General system commands

### Saving the configuration

All variables that are part of a `[...].config` object can be saved to non-volatile memory on the ODrive so they persist after you remove power. The relevant commands are:

 * `<odrv>.save_configuration()`: Stores the configuration to persistent memory on the ODrive.
 * `<odrv>.erase_configuration()`: Resets the configuration variables to their factory defaults. This also reboots the device.

### Diagnostics

 * `<odrv>.serial_number`: A number that uniquely identifies your device. When printed in upper case hexadecimal (`hex(<odrv>.serial_number).upper()`), this is identical to the serial number indicated by the USB descriptor.
 * `<odrv>.fw_version_major`, `<odrv>.fw_version_minor`, `<odrv>.fw_version_revision`: The firmware version that is currently running.
 * `<odrv>.hw_version_major`, `<odrv>.hw_version_minor`, `<odrv>.hw_version_revision`: The hardware version of your ODrive.

## Setting up sensorless
The ODrive can run without encoder/hall feedback, but there is a minimum speed, usually around a few hundred RPM. In other words, sensorless mode does not support stopping or changing direction!

Sensorless mode starts by ramping up the motor speed in open loop control and then switches to closed loop control automatically. The sensorless speed ramping parameters are in `axis.config.sensorless_ramp` The `vel` and `accel` (in [radians/s] and [radians/s^2]) control the speed that the ramp tries to reach and how quickly it gets there. When the ramp reaches `sensorless_ramp.vel`, `controller.input_vel` is automatically set to the same velocity, in [turns/s], and the state switches to closed loop control.

If your motor comes to a stop after the ramp, try incrementally raising the `vel` parameter. The goal is to be above the minimum speed necessary for sensorless position and speed feedback to converge - this is not well-parameterized per motor. The parameters suggested below work for the D5065 motor, with 270KV and 7 pole pairs. If your motor grinds and skips during the ramp, lower the `accel` parameter until it is tolerable.

Below are some suggested starting parameters that you can use for the ODrive D5065 motor. Note that you _must_ set the `pm_flux_linkage` correctly for sensorless mode to work. Motor calibration and setup must also be completed before sensorless mode will work.


```
odrv0.axis0.controller.config.vel_gain = 0.01
odrv0.axis0.controller.config.vel_integrator_gain = 0.05
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_limit = <a value greater than axis.config.sensorless_ramp.vel / (2pi * <pole_pairs>)>
odrv0.axis0.motor.config.current_lim = 2 * odrv0.axis0.config.sensorless_ramp.current
odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (<pole pairs> * <motor kv>)
odrv0.axis0.config.enable_sensorless_mode = True
```

To start the motor:
```
<axis>.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
```
