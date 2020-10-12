# Migration Guide

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

