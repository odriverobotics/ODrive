# Control

The motor controller is a cascaded style position, velocity and current control loop, as per the diagram below. When the control mode is set to position control, the whole loop runs. When running in velocity control mode, the position control part is removed and the velocity command is fed directly in to the second stage input. In current control mode, only the current controller is used.

![Cascaded pos vel I loops](https://static1.squarespace.com/static/58aff26de4fcb53b5efd2f02/t/5b66284a0e2e72aae8818d64/1533421649405/CascadedController.png?format=2500w)

* The position controller is a P loop with a single proportional gain.
* The velocity controller is a PI loop.
* The current controller is a PI loop.
