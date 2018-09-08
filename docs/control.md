# Control

The motor controller is a cascaded style position, velocity and current control loop, as per the diagram below. When the control mode is set to position control, the whole loop runs. When running in velocity control mode, the position control part is removed and the velocity command is fed directly in to the second stage input. In current control mode, only the current controller is used.

![Cascaded pos vel I loops](https://static1.squarespace.com/static/58aff26de4fcb53b5efd2f02/t/5b66284a0e2e72aae8818d64/1533421649405/CascadedController.png?format=2500w)

### Position loop:
The position controller is a P loop with a single proportional gain.
* `vel_cmd = (pos_setpoint - pos_feedback) * pos_gain`

### Velocity loop:
The velocity controller is a PI loop.
* `vel_error_sum += (vel_cmd - vel_feedback) * vel_integrator_gain`
* `current_cmd = (vel_cmd - vel_feedback) * vel_gain + vel_error_sum + vel_feedforward`
### Current loop:
The current controller is a PI loop.
* `current_error_sum += (current_cmd - current_fb) * current_integrator_gain`
* `voltage_cmd = (current_cmd - current_fb) * current_gain + current_error_sum + current_feedforward`

For more detail refer to [controller.cpp](https://github.com/madcowswe/ODrive/blob/master/Firmware/MotorControl/controller.cpp#L86).
