# Step/direction
This is the simplest possible way of controlling the ODrive. It is also the most primitive and fragile one. So don't use it unless you must interoperate with other hardware that you don't control.

### Pinout
* Step/dir signals: Any GPIOs can be used. Also see [Pinout](pinout.md) for more info.
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.

### How to configure

 1. Choose any two of the unused GPIOs for step/dir input. Let's say you chose GPIO7 for the step signal and GPIO8 for the dir signal.
 2. Configure the GPIO modes:

        <odrv>.config.gpio7_mode = GPIO_MODE_DIGITAL_PULL_DOWN
        <odrv>.config.gpio8_mode = GPIO_MODE_DIGITAL

 3. Configure the axis:

        <axis>.config.step_gpio_pin = 7
        <axis>.config.dir_gpio_pin = 8
        <axis>.config.enable_step_dir = True

 4. Enable circular setpoints

        <axis>.controller.config.circular_setpoints = True

After this, step and direction will be enabled when you put the axis into closed loop control. Note that to change out of step/dir, you need to set `<axis>.config.enable_step_dir = False`, go to `AXIS_STATE_IDLE`, and then back into closed loop control.

Circular setpoints are used to keep floating point error at a manageable error for systems where the motor can rotate large amounts. If the motor is commanded out of the circular range, the position setpoint automatically wraps around to stay in the range. Two parameters are used to control this behavior: `<odrv>.<axis>.controller.config.circular_setpoint_range` and `<odrv>.<axis>.controller.config.steps_per_circular_range`. The circular setpoint range sets the operating range of input_pos, starting at 0.0. The `steps per circular range` setting controls how many steps are needed to traverse the entire range. For example, to use 1024 steps per 1 full motor turn, set

```
<odrv>.<axis>.controller.config.circular_setpoint_range = 1.0   [turns]
<odrv>.<axis>.controller.config.steps_per_circular_range = 1024 [steps]
```

The circular range is a floating point value and the steps per circular range parameter is an integer. For best results, set both parameters to powers of 2.

The maximum step rate is pending tests, but 250kHz step rates with both axes in closed loop has been achieved.

Please be aware that there is no enable line right now, and the step/direction interface is enabled by default, and remains active as long as the ODrive is in position control mode. To get the ODrive to go into position control mode at bootup, see how to configure the [startup procedure](commands.md#startup-procedure).
