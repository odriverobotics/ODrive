# Control

The motor controller is a cascaded style position, velocity and current control loop, as per the diagram below. When the control mode is set to position control, the whole loop runs. When running in velocity control mode, the position control part is removed and the velocity command is fed directly in to the second stage input. In torque control mode, only the current controller is used.

![Cascaded pos vel I loops](controller_with_ff.png)

Each stage of the control loop is a variation on a [PID controller](https://en.wikipedia.org/wiki/PID_controller). A PID controller is a mathematical model that can be adapted to control a wide variety of systems. This flexibility is essential as it allows the ODrive to be used to control all kinds of mechanical systems.

* Note:  The controller has been updated to use `torque` in Newton-meters instead of current at the "system" level.  There is a `torque_constant` parameter which converts between torque and current, after which the rest of this explanation still holds.

### Position loop:
The position controller is a P loop with a single proportional gain.
```text
pos_error = pos_setpoint - pos_feedback
vel_cmd = pos_error * pos_gain + vel_feedforward
```

### Velocity loop:
The velocity controller is a PI loop.
```text
vel_error = vel_cmd - vel_feedback
current_integral += vel_error * vel_integrator_gain
current_cmd = vel_error * vel_gain + current_integral + current_feedforward
```

### Current loop:
The current controller is a PI loop.
```text
current_error = current_cmd - current_fb
voltage_integral += current_error * current_integrator_gain
voltage_cmd = current_error * current_gain + voltage_integral (+ voltage_feedforward when we have motor model)
```

Note: `current_gain` and `current_integrator_gain` are automatically set according to `motor.config.current_control_bandwidth`

For more detail refer to [controller.cpp](https://github.com/madcowswe/ODrive/blob/master/Firmware/MotorControl/controller.cpp#L86).

### Controller Details:
The ultimate output of the controller is the voltage applied to the gate of each FET to deliver current through each coil of the motor. The current through the motor linearly relates to the torque output of the motor. This means that the inputs to the cascaded controller are theoretically the position (angle), velocity (angle/time), and acceleration (angle/time/time) of the motor. Note that when thinking about the controller from the perpective of the physics of the motor you would expect to see the time in the Velocity and Current loops, but it is absent because the time difference between iterations is always 125 microseconds (8kHz). Because the time difference between controller loops is a constant and can simply be wrapped into the controller gains. 

The output of each stage of the controller is clamped before being fed into the next stage. So after the `vel_cmd` is calculated from the position controller, the `vel_cmd` is clamped to the velocity limit. The `torque_cmd` output of the velocity controller is then clamped and fed to the current controller. Oddly enough the controller class does not contain the current controller, but instead the current controller is housed in the motor class due to the complexity of the motor driver schema.

The feedforward terms available when using the position or velocity control mode are meant to enable better performance when the dynamics of a system are known and the host controller can predict the motion based on the load. A perfect example of this is the use of the trajectory controller that sets the position, velocity, and torque based on the desired position, velocity, and acceleration. If you take a trapezoidal velocity profile for example, you can imagine on the ramp upward the velocity will be increasing over time, while the torque is a non-zero constant. At the flat portion of the profile the velocity will be a non-zero constant, but the acceleration will be zero. This trajectory controller use case uses the cascaded controller with multiple inputs to achieve the desired motion with the best performance.  

## Tuning
Tuning the motor controller is an essential step to unlock the full potential of the ODrive. Tuning allows for the controller to quickly respond to disturbances or changes in the system (such as an external force being applied or a change in the setpoint) without becoming unstable. Correctly setting the three tuning parameters (called gains) ensures that ODrive can control your motors in the most effective way possible. The three values are:
* `<axis>.controller.config.pos_gain = 20.0` [(turn/s) / turn]
* `<axis>.controller.config.vel_gain = 0.16 ` [Nm/(turn/s)]
* `<axis>.controller.config.vel_integrator_gain = 0.32` [Nm/((turn/s) * s)]

An upcoming feature will enable automatic tuning. Until then, here is a rough tuning procedure:
* Set vel_integrator_gain gain to 0
* Make sure you have a stable system. If it is not, decrease all gains until you have one.
* Increase `vel_gain` by around 30% per iteration until the motor exhibits some vibration.
* Back down `vel_gain` to 50% of the vibrating value.
* Increase `pos_gain` by around 30% per iteration until you see some overshoot.
* Back down `pos_gain` until you do not have overshoot anymore.
* The integrator can be set to `0.5 * bandwidth * vel_gain`, where `bandwidth` is the overall resulting tracking bandwidth of your system. Say your tuning made it track commands with a settling time of 100ms (the time from when the setpoint changes to when the system arrives at the new setpoint); this means the bandwidth was 1/(100ms) = 1/(0.1s) = 10hz. In this case you should set the `vel_integrator_gain = 0.5 * 10 * vel_gain`.

The liveplotter tool can be immensely helpful in dialing in these values. To display a graph that plots the position setpoint vs the measured position value run the following in the ODrive tool:

`start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])` 
