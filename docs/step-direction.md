# Step/direction
This is the simplest possible way of controlling the ODrive. It is also the most primitive and fragile one. So don't use it unless you must interoperate with other hardware that you don't control.

Pinout:
* Step/dir signals: see [Pinout](pinout.md). Note in that section how to reassign the pins.
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.

To enable step/dir mode for the GPIO, set `<axis>.config.enable_step_dir` to true for each axis that you wish to use this on.
Axis 0 step/dir pins conflicts with UART, and the UART takes priority. So to be able to use step/dir on Axis 0, you must also set `odrv0.config.enable_uart = False`. See the [pin function priorities](pinout.md#pin-function-priorities) for more detail. Don't forget to save configuration and reboot.

There is also a config variable called `<axis>.config.turns_per_step`, which specifies how many turns a "step" corresponds to. The default value is 1.0f/1024.0f. It can be any floating point value.
The recommended maximum step rate is 50kHz

Please be aware that there is no enable line right now, and the step/direction interface is enabled by default, and remains active as long as the ODrive is in position control mode. To get the ODrive to go into position control mode at bootup, see how to configure the [startup procedure](commands.md#startup-procedure).
