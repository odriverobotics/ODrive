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

There is also a config variable called `<axis>.config.turns_per_step`, which specifies how many turns a "step" corresponds to. The default value is 1.0f/1024.0f. It can be any floating point value.
The maximum step rate is pending tests, but it should handle at least 50kHz. If you want to test it, please be aware that the failure mode on too high step rates is expected to be that the motors shuts down and coasts.

Please be aware that there is no enable line right now, and the step/direction interface is enabled by default, and remains active as long as the ODrive is in position control mode. To get the ODrive to go into position control mode at bootup, see how to configure the [startup procedure](commands.md#startup-procedure).