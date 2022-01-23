
.. _commands-doc:

================================================================================
Parameters & Commands
================================================================================

We will use the :code:`<odrv>` as a placeholder for any ODrive object. Every ODrive controller is an ODrive object. In :code:`odrivetool` this is usually :code:`odrv0`. Furthermore we use `<axis>` as a placeholder for any axis, which is an attribute of an ODrive object (for example `odrv0.axis0`). An axis represents where the motors are connected. (axis0 for M0 or axis1 for M1)

.. contents::
   :depth: 1
   :local:

Per-Axis Commands
-------------------------------------------------------------------------------

For the most part, both axes on the ODrive can be controlled independently.

State Machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The current state of an axis is indicated by :attr:`<axis>.current_state <ODrive.Axis.current_state>`. 
The user can request a new state by assigning a new value to :attr:`<axis>.requested_state <ODrive.Axis.current_state>`. 
The default state after startup is :code:`AXIS_STATE_IDLE`. A description of all states can be found :attr:`here <ODrive.Axis.AxisState>`.

.. _commands-startup-procedure:

Startup Procedure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default the ODrive takes no action at startup and goes to idle immediately.
In order to change what startup procedures are used, set the startup procedures you want to `True`.
The ODrive will sequence all enabled startup actions selected in the order shown below.

* :code:`<axis>.config.startup_motor_calibration`
* :code:`<axis>.config.startup_encoder_index_search`
* :code:`<axis>.config.startup_encoder_offset_calibration`
* :code:`<axis>.config.startup_closed_loop_control`

See :attr:`here <ODrive.Axis.AxisState>` for a description of each state.

Control Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The default control mode is position control.
If you want a different mode, you can change :code:`<axis>.controller.config.control_mode`.
Possible values are listed :attr:`here <ODrive.Controller.ControlMode>`.

Input Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As of version v0.5.0, ODrive now intercepts the incoming commands and can apply filters to them. 
The old protocol values :code:`pos_setpoint`, :code:`vel_setpoint`, and `current_setpoint` are still used internally by the closed-loop cascade control, but the user cannot write to them directly.  
This allows us to condense the number of ways the ODrive accepts motion commands. 


Control Commands
********************************************************************************

* :code:`<axis>.controller.input_pos = <turn>`
* :code:`<axis>.controller.input_vel = <turn/s>`
* :code:`<axis>.controller.input_torque = <torque in Nm>`

Modes can be selected by changing :code:`<axis>.controller.config.input_mode`.
The default input mode is :code:`INPUT_MODE_PASSTHROUGH`.
Possible values are listed :attr:`here <ODrive.Controller.InputMode>`.

System Monitoring Commands
-------------------------------------------------------------------------------

Encoder Position and Velocity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* View encoder position with :code:`<axis>.encoder.pos_estimate` [turns] or :code:`<axis>.encoder.pos_est_counts` [counts]
* View rotational velocity with :code:`<axis>.encoder.vel_estimate` [turn/s] or :code:`<axis>.encoder.vel_est_counts` [count/s]

Motor Current and Torque Estimation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* View the commanded motor current with :code:`<axis>.motor.current_control.Iq_setpoint` [A] 
* View the measured motor current with :code:`<axis>.motor.current_control.Iq_measured` [A]. 
  If you find that this returns noisy data then use the command motor current instead. 
  The two values should be close so long as you are not approaching the maximum achievable rotational velocity of your motor for a given supply voltage, in which case the commanded current may become larger than the measured current. 

Using the motor current and the known KV of your motor you can estimate the motors torque using the following relationship: Torque [N.m] = 8.27 * Current [A] / KV. 

General System Commands
-------------------------------------------------------------------------------

Saving the Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All variables that are part of a :code:`[...].config` object can be saved to non-volatile memory on the ODrive so they persist after you remove power. 
The relevant commands are:

 * :code:`<odrv>.save_configuration()`: Stores the configuration to persistent memory on the ODrive.
 * :code:`<odrv>.erase_configuration()`: Resets the configuration variables to their factory defaults. This also reboots the device.

Diagnostics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 * :code:`<odrv>.serial_number`: A number that uniquely identifies your device. When printed in upper case hexadecimal (:code:`hex(<odrv>.serial_number).upper()`), this is identical to the serial number indicated by the USB descriptor.
 * :code:`<odrv>.fw_version_major`, :code:`<odrv>.fw_version_minor`, :code:`<odrv>.fw_version_revision`: The firmware version that is currently running.
 * :code:`<odrv>.hw_version_major`, :code:`<odrv>.hw_version_minor`, :code:`<odrv>.hw_version_revision`: The hardware version of your ODrive.

.. _sensorless-setup:

Setting up Sensorless
-------------------------------------------------------------------------------

The ODrive can run without encoder/hall feedback, but there is a minimum speed, usually around a few hundred RPM. 
In other words, sensorless mode does not support stopping or changing direction!

Sensorless mode starts by ramping up the motor speed in open loop control and then switches to closed loop control automatically. 
The sensorless speed ramping parameters are in :code:`axis.config.sensorless_ramp`. 
The :code:`vel` and :code:`accel` (in [radians/s] and [radians/s^2]) parameters control the speed that the ramp tries to reach and how quickly it gets there. 
When the ramp reaches :code:`sensorless_ramp.vel`, :code:`controller.input_vel` is automatically set to the same velocity, in [turns/s], and the state switches to closed loop control.

If your motor comes to a stop after the ramp, try incrementally raising the :code:`vel` parameter. 
The goal is to be above the minimum speed necessary for sensorless position and speed feedback to converge - this is not well-parameterized per motor. 
The parameters suggested below work for the D5065 motor, with 270KV and 7 pole pairs. 
If your motor grinds and skips during the ramp, lower the :code:`accel` parameter until it is tolerable.

Below are some suggested starting parameters that you can use for the ODrive D5065 motor. 
Note that you **must** set the :code:`pm_flux_linkage` correctly for sensorless mode to work. 
Motor calibration and setup must also be completed before sensorless mode will work.


.. code:: iPython

   odrv0.axis0.controller.config.vel_gain = 0.01
   odrv0.axis0.controller.config.vel_integrator_gain = 0.05
   odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
   odrv0.axis0.controller.config.vel_limit = <a value greater than axis.config.sensorless_ramp.vel / (2pi * <pole_pairs>)>
   odrv0.axis0.motor.config.current_lim = 2 * odrv0.axis0.config.sensorless_ramp.current
   odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (<pole pairs> * <motor kv>)
   odrv0.axis0.config.enable_sensorless_mode = True


To start the motor:

.. code:: iPython
  
      odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

