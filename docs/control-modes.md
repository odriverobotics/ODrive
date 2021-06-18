
The default control mode is unfiltered position control in the absolute encoder reference frame. You may wish to use a controlled trajectory instead. Or you may wish to control position in a circular frame to allow continuous rotation forever without growing the numeric value of the setpoint too large.

You may also wish to control velocity (directly or with a ramping filter).
You can also directly control the current of the motor, which is proportional to torque.

- [Filtered position control](#filtered-position-control)
- [Trajectory control](#trajectory-control)
- [Circular position control](#circular-position-control)
- [Velocity control](#velocity-control)
- [Ramped velocity control](#ramped-velocity-control)
- [Torque control](#torque-control)


### Filtered position control
Asking the ODrive controller to go as hard as it can to raw setpoints may result in jerky movement. Even if you are using a planned trajectory generated from an external source, if that is sent at a modest frequency, the ODrive may chase each stair in the incoming staircase in a jerky way. In this case, a good starting point for tuning the filter bandwidth is to set it to one half of your setpoint command rate.

You can use the second order position filter in these cases.
Set the filter bandwidth: `axis.controller.config.input_filter_bandwidth = 2.0` [1/s]<br>
Activate the setpoint filter: `axis.controller.config.input_mode = INPUT_MODE_POS_FILTER`.<br>
You can now control the velocity with `axis.controller.input_pos = 1` [turns].

![secondOrderResponse](secondOrderResponse.PNG)<br>
Step response of a 1000 to 0 position input with a filter bandwidth of 1.0 [/sec].

### Trajectory control
See the **Usage** section for usage details.<br>
This mode lets you smoothly accelerate, coast, and decelerate the axis from one position to another. With raw position control, the controller simply tries to go to the setpoint as quickly as possible. Using a trajectory lets you tune the feedback gains more aggressively to reject disturbance, while keeping smooth motion.

![Taptraj](TrapTrajPosVel.PNG)<br>
In the above image blue is position and orange is velocity.

#### Parameters
```
<odrv>.<axis>.trap_traj.config.vel_limit = <Float>
<odrv>.<axis>.trap_traj.config.accel_limit = <Float>
<odrv>.<axis>.trap_traj.config.decel_limit = <Float>
<odrv>.<axis>.controller.config.inertia = <Float>
```

`vel_limit` is the maximum planned trajectory speed.  This sets your coasting speed.<br>
`accel_limit` is the maximum acceleration in turns / sec^2<br>
`decel_limit` is the maximum deceleration in turns / sec^2<br>
`controller.config.inertia` is a value which correlates acceleration (in turns / sec^2) and motor torque. It is 0 by default. It is optional, but can improve response of your system if correctly tuned. Keep in mind this will need to change with the load / mass of your system.

All values should be strictly positive (>= 0).

Keep in mind that you must still set your safety limits as before.  It is recommended you set these a little higher ( > 10%) than the planner values, to give the controller enough control authority.
```
<odrv>.<axis>.motor.config.current_lim = <Float>
<odrv>.<axis>.controller.config.vel_limit = <Float>
```

#### Usage
Make sure you are in position control mode. To activate the trajectory module, set the input mode to trajectory:
```
axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
```

Simply send a position command to execute the move:
```
<odrv>.<axis>.controller.input_pos = <Float>
```

Use the `move_incremental` function to move to a relative position.
To set the goal relative to the current actual position, use `from_goal_point = False`
To set the goal relative to the previous destination, use `from_goal_point = True`
```
<odrv>.<axis>.controller.move_incremental(pos_increment, from_goal_point)
```

You can also execute a move with the [appropriate ascii command](ascii-protocol.md#motor-trajectory-command).

### Circular position control

To enable Circular position control, set `axis.controller.config.circular_setpoints = True`

This mode is useful for continuous incremental position movement. For example a robot rolling indefinitely, or an extruder motor or conveyor belt moving with controlled increments indefinitely.
In the regular position mode, the `input_pos` would grow to a very large value and would lose precision due to floating point rounding.

In this mode, the controller will try to track the position within only one turn of the motor. Specifically, `input_pos` is expected in the range `[0, 1)`. If the `input_pos` is incremented to outside this range (say via step/dir input), it is automatically wrapped around into the correct value.
Note that in this mode `encoder.pos_circular` is used for feedback instead of `encoder.pos_estimate`.

If you try to increment the axis with a large step in one go that exceeds `1` turn, the motor will go to the same angle around the wrong way. This is also the case if there is a large disturbance. If you have an application where you would like to handle larger steps, you can use a larger circular range. Set `controller.config.circular_setpoints_range = N`. Choose N to give you an appropriate circular space for your application.

### Velocity control
Set `axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL`.<br>
You can now control the velocity with `axis.controller.input_vel = 1` [turn/s].

### Ramped velocity control
Set `axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL`.<br>
Set the velocity ramp rate (acceleration): `axis.controller.config.vel_ramp_rate = 0.5` [turn/s^2]<br>
Activate the ramped velocity mode: `axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP`.<br>
You can now control the velocity with `axis.controller.input_vel = 1` [turn/s].

### Torque control
Set `axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL`.<br>
You can now control the torque with `axis.controller.input_torque = 0.1` [Nm].

Note: If you exceed `vel_limit` in torque control mode, the current is reduced. To disable this, set `axis.controller.enable_current_mode_vel_limit = False`.
