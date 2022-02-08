================================================================================
Migration Guide
================================================================================

.. contents::
   :depth: 1
   :local:
   
v0.5.1 -> v0.5.2
--------------------------------------------------------------------------------

The change from v0.5.1 to v0.5.2 had fewer breaking changes than the v0.4.12 to v0.5.1 change.

GPIO Modes
--------------------------------------------------------------------------------

The GPIO configuration is now more explicit. For example, to use :code:`gpio1` for for step signals (as part of a step/dir interface), it must be set to

.. code:: iPython

    odrv0.config.gpio1_mode = GPIO_MODE_DIGITAL


Braking Behavior
--------------------------------------------------------------------------------

Before using the brake resistor, it must be explicitly enabled as follows:

.. code:: iPython

    odrv0.config.enable_brake_resistor = True

and then save the configuration and reboot for the setting to take effect.

Step/Direction Settings
--------------------------------------------------------------------------------

Previously, steps were added incrementally to :code:`input_pos`. 
This caused issues with accumulated floating point rounding error. 
Now, an absolute step count is used. 
This change requires that the circular setpoints mode is used :code:`odrv0.axis0.controller.config.circular_setpoints = True` when step/dir signals are used.

In addition, :code:`odrv0.axis0.config.turns_per_step` has been removed and :code:`odrv0.axis0.controller.config.steps_per_circular_range` is used. 

For example:

previously:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: iPython

    odrv0.axis0.config.turns_per_step = 1.0/1024.0

v0.5.2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: iPython

    odrv0.axis0.controller.config.circular_setpoints = True
    odrv0.axis0.controller.config.circular_setpoint_range = 1.0
    odrv0.axis0.controller.config.steps_per_circular_range = 1024

For best results, set both the circular range and steps per circular range to powers of 2.

API changes
--------------------------------------------------------------------------------

For other API changes, see the Changelog file on github.

v0.4.12 -> v0.5.1
--------------------------------------------------------------------------------

Certain changes occurred between firmware versions v0.4.12 and v0.5.1 that will break existing configurations. 
This document is a guide for how to take a working v0.4.12 ODrive config and change it to work with firmware v0.5.1.

Unit Changes
--------------------------------------------------------------------------------

ODrive now uses units of [turns], [turns/s], and [turns/s^2] instead of [counts], [counts/s], and [counts/s^2]. 
In addition, the motor controller class now has an input command of torque in [Nm] instead of current in [Amps]. 
In general, every user-facing parameter that has to do with position or velocity is affected by the unit change.

.. note:: 
    For the torque to be in correct in [Nm] you need to configure the :code:`motor.config.torque_constant`. 
    See the updated :ref:`getting started <motor-config>` for more details.

Control Parameter Names
--------------------------------------------------------------------------------

ODrive now uses :code:`input_pos`, :code:`input_vel`, and :code:`input_torque` as commands instead of :code:`pos_setpoint`, :code:`vel_setpoint`, and :code:`current_setpoint`.

Guide
--------------------------------------------------------------------------------

For a working v0.4.12 ODrive configuration, use the following equations to convert parameters as required.

* :code:`pos_gain` is unaffected ( [counts/s / count] `*`> [turns/s / turns] )
* :code:`vel_gain` is :code:`vel_gain_old * torque_constant * encoder cpr`
* :code:`vel_integrator_gain` is :code:`vel_integrator_gain_old * torque_constant * encoder cpr`

For other values, [turns] = [counts] / [encoder cpr]. Converting [counts/s] and [counts/s^2] is similar.

Affected Variables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* :code:`axis.controller.input_pos`
* :code:`axis.controller.input_vel`
* :code:`axis.controller.input_torque`
* :code:`axis.controller.config.vel_limit`
* :code:`axis.controller.config.vel_ramp_rate`
* :code:`axis.controller.config.current_ramp_rate` is now :code:`axis.controller.config.torque_ramp_rate`
* :code:`axis.controller.config.circular_setpoint_range`
* :code:`axis.controller.config.inertia`
* :code:`axis.controller.config.homing_speed`
* :code:`axis.controller.pos_setpoint`
* :code:`axis.controller.vel_setpoint`
* :code:`axis.controller.torque_setpoint` instead of :code:`axis.controller.current_setpoint`
* :code:`axis.trap_traj.config.vel_limit`
* :code:`axis.trap_traj.config.accel_limit`
* :code:`axis.trap_traj.config.decel_limit`
* :code:`axis.encoder.pos_estimate`
* :code:`axis.encoder.pos_estimate_circular`
* :code:`axis.encoder.vel_estimate`
* :code:`axis.config.counts_per_step` is now :code:`turns_per_step` for the step/direction interface
* :code:`axis.sensorless_estimator.vel_estimate` is in mechanical [turns/s] instead of electrical [radians/s]

