# Interfaces

<div class="alert"> While developing custom ODrive control code it is recommend that your motors are free to spin continuously and are not connected to a drivetrain with limited travel. </div>

The ODrive can be controlled over various interfaces and protocols.

[TODO: include a picture that shows all interfaces with the supported protocols]

### Setting up UART
Baud rate: 115200
Pinout:
* GPIO 1: Tx (connect to Rx of other device)
* GPIO 2: Rx (connect to Tx of other device)
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.

## Native protocol

If you have a choice, this is the recommended protocol for all applications.

### Python

The ODrive Tool you installed as part of the [Getting Started guide](getting-started#downloading-and-installing-tools) comes with a library that you can use to easily control the ODrive from Python.

Assuming you already installed the odrive library (`pip install odrive`), the simplest program to control the ODrive is this:

```python
import odrive.discovery
odrv0 = odrive.discovery.find_any()
print(str(odrv0.vbus_voltage))
```

For a more comprehensive example, see [odrive_demo.py](../tools/odrive_demo.py).

### Other languages

We don't have an official library for you just yet. Check the community, there might be someone working on it. If you want to write a library yourself, refer to the [native protocol specification](protocol). You are of course welcome to contribute it back.

## ASCII protocol

This is a simpler alternative to the native protocol if you don't need all its bells and whistles. Before you use this, be sure that you're ok with its limitations.

This protocol may be extended in the future to support a selected set of GCode commands.

For more details, see the [ASCII protocol specification](ascii-protocol.md).

### C++ (Arduino)

[See ODrive Arduino Library](https://github.com/madcowswe/ODriveArduino)

## Step/direction
This is the simplest possible way of controlling the ODrive. It is also the most primitive and brittle one. So don't use it unless you must interoperate with other hardware that you don't control.

Pinout:
* GPIO 1: M0 step
* GPIO 2: M0 dir
* GPIO 3: M1 step
* GPIO 4: M1 dir
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.

Please note that GPIO_3 and GPIO_4 are NOT 5v tolerant on ODrive v3.2 and earlier, so 3.3V signals only!
ODrive v3.3 and onward have 5V tolerant GPIO pins.

To enable step/dir mode for the GPIO, set `<axis>.config.enable_step_dir` to true and reboot the ODrive.

There is also a config variable called `<axis>.config.counts_per_step`, which specifies how many encoder counts a "step" corresponds to. It can be any floating point value.
The maximum step rate is pending tests, but it should handle at least 16kHz. If you want to test it, please be aware that the failure mode on too high step rates is expected to be that the motors shuts down and coasts.

Please be aware that there is no enable line right now, and the step/direction interface is enabled by default, and remains active as long as the ODrive is in position control mode. By default the ODrive starts in position control mode, so you don't need to send any commands over USB to get going. You can still send USB commands if you want to.
