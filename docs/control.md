# Control

The motor controller is a cascaded style position, velocity and current control loop, as per the diagram below. When the control mode is set to position control, the whole loop runs. When running in velocity control mode, the position control part is removed and the velocity command is fed directly in to the second stage input. In current control mode, only the current controller is used.

![Cascaded pos vel I loops](https://github.com/madcowswe/ODrive/blob/master/docs/controller_with_ff.png?raw=true)

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

For more detail refer to [controller.cpp](https://github.com/madcowswe/ODrive/blob/master/Firmware/MotorControl/controller.cpp#L86).
