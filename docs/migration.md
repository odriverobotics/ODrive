# Migration Guide

## v0.5.1 -> v0.5.2

The change from v0.5.1 to v0.5.2 had fewer breaking changes than the v0.4.12 to v0.5.1 change.

## GPIO modes
The GPIO configuration is now more explicit. For example, to use `gpio1` for for step signals (as part of a step/dir interface), it must be set to
```
odrv0.config.gpio1_mode = GPIO_MODE_DIGITAL
```

## Braking behavior
Before using the brake resistor, it must be explicitly enabled as follows:
```
odrv0.config.enable_brake_resistor = True
```
and then save the configuration and reboot for the setting to take effect.

## Step/direction settings
Previously, steps were added incrementally to `input_pos`. This caused issues with accumulated floating point rounding error. Now, an absolute step count is used. This change requires that the circular setpoints mode is used `odrv0.axis0.controller.config.circular_setpoints = True` when step/dir signals are used.

In addition, `odrv0.axis0.config.turns_per_step` has been removed and `odrv0.axis0.controller.config.steps_per_circular_range` is used. For example:
```
# previous
odrv0.axis0.config.turns_per_step = 1.0/1024.0

# v0.5.2
odrv0.axis0.controller.config.circular_setpoints = True
odrv0.axis0.controller.config.circular_setpoint_range = 1.0
odrv0.axis0.controller.config.steps_per_circular_range = 1024
```

For best results, set both the circular range and steps per circular range to powers of 2.

## API changes

For other API changes, see the Changelog file on github.

## v0.4.12 -> v0.5.1

Certain changes occurred between firmware versions v0.4.12 and v0.5.1 that will break existing configurations. This document is a guide for how to take a working v0.4.12 ODrive config and change it to work with firmware v0.5.1.

## Unit Changes
ODrive now uses units of [turns], [turns/s], and [turns/s^2] instead of [counts], [counts/s], and [counts/s^2]. In addition, the motor controller class now has an input command of torque in [Nm] instead of current in [Amps]. In general, every user-facing parameter that has to do with position or velocity is affected by the unit change.

**Note:** For the torque to be in correct in [Nm] you need to configure the `motor.config.torque_constant`. See the updated [getting started](getting-started.md/#configure-m0) for more details.

## Control Parameter Names
ODrive now uses `input_pos`, `input_vel`, and `input_torque` as commands instead of `pos_setpoint`, `vel_setpoint`, and `current_setpoint`

## Guide
For a working v0.4.12 ODrive configuration, use the following equations to convert parameters as required.

- `pos_gain` is unaffected ( [counts/s / count] -> [turns/s / turns] )
- `vel_gain` is `vel_gain_old` * `torque_constant` * `encoder cpr`
- `vel_integrator_gain` is `vel_integrator_gain_old` * `torque_constant` * `encoder cpr`

For other values, [turns] = [counts] / [encoder cpr]. Converting [counts/s] and [counts/s^2] is similar.

### Affected Variables
- `axis.controller.input_pos`
- `axis.controller.input_vel`
- `axis.controller.input_torque`
- `axis.controller.config.vel_limit`
- `axis.controller.config.vel_ramp_rate`
- `axis.controller.config.current_ramp_rate` is now `axis.controller.config.torque_ramp_rate`
- `axis.controller.config.circular_setpoint_range`
- `axis.controller.config.inertia`
- `axis.controller.config.homing_speed`
- `axis.controller.pos_setpoint`
- `axis.controller.vel_setpoint`
- `axis.controller.torque_setpoint` instead of `axis.controller.current_setpoint`
- `axis.trap_traj.config.vel_limit`
- `axis.trap_traj.config.accel_limit`
- `axis.trap_traj.config.decel_limit`
- `axis.encoder.pos_estimate`
- `axis.encoder.pos_estimate_circular`
- `axis.encoder.vel_estimate`
- `axis.config.counts_per_step` is now `turns_per_step` for the step/direction interface
- `axis.sensorless_estimator.vel_estimate` is in mechanical [turns/s] instead of electrical [radians/s]

