# Interfaces

<div class="alert"> While developing custom ODrive control code it is recommend that your motors are free to spin continuously and are not connected to a drivetrain with limited travel. </div>

The ODrive can be controlled over various ports and protocols. If you're comfortable with embedded systems development, you can also run custom code directly on the ODrive. For that refer to the [developer documentation](developer-guide.md).

### Table of contents

<!-- MarkdownTOC depth=2 autolink=true bracket=round -->

- [Pinout](#pinout)
- [Native Protocol](#native-protocol)
- [ASCII Protocol](#ascii-protocol) (and Arduino)
- [Step/direction](#stepdirection)
- [RC PWM input](#rc-pwm-input) (coming soon)
- [Ports](#ports)
   - [USB](#usb)
   - [UART](#uart)

<!-- /MarkdownTOC -->

## Pinout

| GPIO      | primary   | step/dir      | other                   |
|-----------|-----------|---------------|-------------------------|
| GPIO1     | UART TX   | Axis0 Step    | Analog input, PWM input |
| GPIO2     | UART RX   | Axis0 Dir     | Analog input, PWM input |
| GPIO3     |           | Axis1 Step (+)| Analog input, PWM input |
| GPIO4     |           | Axis1 Dir (+) | Analog input, PWM input |
| GPIO5     |           |               | Analog input (*)        |
| GPIO6 (*) |           |               |                         |
| GPIO7 (*) |           | Axis1 Step (*)|                         |
| GPIO8 (*) |           | Axis1 Dir (*) |                         |

(+) on ODrive v3.4 and earlier <br>
(*) ODrive v3.5 and later

ODrive v3.3 and onward have 5V tolerant GPIO pins.

## Native Protocol

This protocol is what the ODrive Tool uses to talk to the ODrive. If you have a choice, this is the recommended protocol for all applications. The native protocol runs on USB and can also be configured to run on UART.

#### Python

The ODrive Tool you installed as part of the [Getting Started guide](getting-started#downloading-and-installing-tools) comes with a library that you can use to easily control the ODrive from Python.

Assuming you already installed the odrive library (`pip install odrive`), the simplest program to control the ODrive is this:

```python
import odrive
odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))
```

For a more comprehensive example, see [tools/odrive_demo.py](../tools/odrive_demo.py).

#### Other languages

We don't have an official library for you just yet. Check the community, there might be someone working on it. If you want to write a library yourself, refer to the [native protocol specification](protocol). You are of course welcome to contribute it back.

## ASCII protocol

This is a simpler alternative to the native protocol if you don't need all its bells and whistles. Before you use this, be sure that you're ok with its limitations. The ASCII protocol is enabled by default on UART and can also be enabled on USB alongside with the native protocol.

For more details, see the [ASCII protocol specification](ascii-protocol.md).

### Arduino
There is an Arduino library that gives some expamples on how to use the ASCII protocol to communicate with the ODrive. Check it out [here](../Arduino/ODriveArduino).

## Step/direction
This is the simplest possible way of controlling the ODrive. It is also the most primitive and fragile one. So don't use it unless you must interoperate with other hardware that you don't control.

Pinout:
* Step/dir signals: see [Pinout](#pinout) above.
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.

To enable step/dir mode for the GPIO, set `<axis>.config.enable_step_dir` to true for each axis that you wish to use this on.
Axis 0 step/dir pins conflicts with UART, and the UART takes priority. So to be able to use step/dir on Axis 0, you must also set `odrv0.config.enable_uart = False`.
To apply these settings you must reboot, and to keep them on reboot you must save configuration:
* `odrv0.save_configuration()`
* `odrv0.reboot()`

There is also a config variable called `<axis>.config.counts_per_step`, which specifies how many encoder counts a "step" corresponds to. It can be any floating point value.
The maximum step rate is pending tests, but it should handle at least 50kHz. If you want to test it, please be aware that the failure mode on too high step rates is expected to be that the motors shuts down and coasts.

Please be aware that there is no enable line right now, and the step/direction interface is enabled by default, and remains active as long as the ODrive is in position control mode. To get the ODrive to go into position control mode at bootup, see how to configure the [startup procedure](commands.md#startup-procedure).

<!--
## RC PWM input

You can control the ODrive directly from an hobby RC receiver.

Up to 4 channels (GPIOs 1, 2, 3 and 4) can be used simultaneously if the respective pins are not assigned to other functions. Any of the numerical parameters that are writable from the ODrive Tool can be hooked up to a PWM input.

As an example, we'll configure GPIO4 to control the angle of axis 0. We want the axis to move within a range of -1500 to 1500 encoder counts.

1. Make sure you're able control the axis 0 angle by writing to `odrv0.axis0.controller.pos_setpoint`. If you need help with this follow the [getting started guide](getting-started.md).
2. It is recommended that you configure the ODrive such that axis 0 automatically goes operational after a reboot. You may have to set `odrv0.axis0.config.startup_encoder_offset_calibration` and `odrv0.axis0.config.startup_closed_loop_control` to `True`. The exact procedure may vary depending on what type of encoder you're using.
3. In ODrive Tool, configure the PWM input mapping
    ```
    In [1]: odrv0.config.gpio4_pwm_mapping.min = -1500
    
    In [2]: odrv0.config.gpio4_pwm_mapping.max = 1500
    
    In [3]: odrv0.config.gpio4_pwm_mapping.endpoint = odrv0.axis0.controller._remote_attributes['pos_setpoint']
    ```
   Note: you can disable the input by setting `odrv0.config.gpio4_pwm_mapping.endpoint = None`
4. Save the configuration and reboot
    ```
    In [4]: odrv0.save_configuration()
    
    In [5]: odrv0.reboot()
    ```
5. Connect the RC receiver ground to the ODrive's GND and one of the RC receiver signals to GPIO4. You may try to power the receiver from the ODrive's 5V supply if it doesn't draw too much power. Power up the the RC transmitter. You should now be able to control axis 0 from one of the RC sticks.

-->

## Ports
Note: when you use an existing library you don't have to deal with the specifics described in this section.

### USB

This section assumes that you are familiar with the general USB architecture, in particular with terms like "configuration", "interface" and "endpoint".

On USB the ODrive provides a single configuration which is a composite device consisting of a CDC device (virtual COM port) and a vendor specific device.

<details><summary markdown="span">What is a composite device?</summary><div markdown="block">
A composite device is a device where interfaces are grouped by interface association descriptors. For such devices, the host OS loads an intermediate driver, so that each of the interface groups can be treated like a separate device and have its own host-side driver attached.
</div></details>

On the ODrive, the following interface groups are present:

 * Interface Association: Communication Device Class (CDC)
    * Interface 0:
        * Endpoint `0x82`: CDC commands
    * Interface 1:
        * Endpoint `0x01`: CDC data OUT
        * Endpoint `0x81`: CDC data IN
 * Interface Association: Vendor Specific Device Class
    * Interface 2:
        * Endpoint `0x03`: data OUT
        * Endpoint `0x83`: data IN

The endpoint pairs `0x01, 0x81` and `0x03, 0x83` behave exactly identical, only their descriptors (interface class, ...) are different.

If you plan to access the USB endpoints directly it is recommended that you use interface 2. The other interfaces (the ones associated with the CDC device) are usually claimed by the CDC driver of the host OS, so their endpoints cannot be used without first detaching the CDC driver.

### UART
Baud rate: 115200
Pinout:
* GPIO 1: Tx (connect to Rx of other device)
* GPIO 2: Rx (connect to Tx of other device)
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.
