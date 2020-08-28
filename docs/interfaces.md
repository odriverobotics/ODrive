# Interfaces

<div class="alert"> While developing custom ODrive control code it is recommend that your motors are free to spin continuously and are not connected to a drivetrain with limited travel. </div>

The ODrive can be controlled over various ports and protocols. If you're comfortable with embedded systems development, you can also run custom code directly on the ODrive. For that refer to the [developer documentation](developer-guide.md).

### Table of contents
<!-- TOC depthFrom:2 depthTo:2 -->

- [Pinout](#pinout)
- [Native Protocol](#native-protocol)
- [ASCII protocol](#ascii-protocol)
- [Step/direction](#stepdirection)
- [RC PWM input](#rc-pwm-input)
- [Ports](#ports)

<!-- /TOC -->

## Pinout

| #  | Label         | `GPIO_MODE_DIGITAL`    | `GPIO_MODE_ANALOG_IN` | `GPIO_MODE_UART0` | `GPIO_MODE_PWM0` | `GPIO_MODE_CAN0` | `GPIO_MODE_I2C0` | `GPIO_MODE_ENC0` | `GPIO_MODE_ENC1` | `GPIO_MODE_MECH_BRAKE` |
|----|---------------|------------------------|-----------------------|-------------------|------------------|------------------|------------------|------------------|------------------|------------------------|
|  0 | _not a pin_   |                        |                       |                   |                  |                  |                  |                  |                  |                        |
|  1 | GPIO1 (+)     | general purpose        | analog input          | **UART0.TX**      | PWM0.0           |                  |                  |                  |                  | mechanical brake       |
|  2 | GPIO2 (+)     | general purpose        | analog input          | **UART0.RX**      | PWM0.1           |                  |                  |                  |                  | mechanical brake       |
|  3 | GPIO3         | general purpose        | **analog input**      |                   | PWM0.2           |                  |                  |                  |                  | mechanical brake       |
|  4 | GPIO4         | general purpose        | **analog input**      |                   | PWM0.3           |                  |                  |                  |                  | mechanical brake       |
|  5 | GPIO5         | general purpose        | **analog input** (*)  |                   |                  |                  |                  |                  |                  | mechanical brake       |
|  6 | GPIO6 (*) (+) | **general purpose**    |                       |                   |                  |                  |                  |                  |                  | mechanical brake       |
|  7 | GPIO7 (*) (+) | **general purpose**    |                       |                   |                  |                  |                  |                  |                  | mechanical brake       |
|  8 | GPIO8 (*) (+) | **general purpose**    |                       |                   |                  |                  |                  |                  |                  | mechanical brake       |
|  9 | M0.A          | general purpose        |                       |                   |                  |                  |                  | **ENC0.A**       |                  |                        |
| 10 | M0.B          | general purpose        |                       |                   |                  |                  |                  | **ENC0.B**       |                  |                        |
| 11 | M0.Z          | **general purpose**    |                       |                   |                  |                  |                  |                  |                  |                        |
| 12 | M1.A          | general purpose        |                       |                   |                  |                  | I2C.SCL          |                  | **ENC1.A**       |                        |
| 13 | M1.B          | general purpose        |                       |                   |                  |                  | I2C.SDA          |                  | **ENC1.B**       |                        |
| 14 | M1.Z          | **general purpose**    |                       |                   |                  |                  |                  |                  |                  |                        |
| 15 | _not exposed_ | general purpose        |                       |                   |                  | **CAN0.RX**      | I2C.SCL          |                  |                  |                        |
| 16 | _not exposed_ | general purpose        |                       |                   |                  | **CAN0.TX**      | I2C.SDA          |                  |                  |                        |


(*) ODrive v3.5 and later <br>
(+) On ODrive v3.5 and later these pins have noise suppression filters. This is useful for step/dir input. <br>

Notes:
* Changes to the pin configuration only take effect after `odrv0.save_configuration()` and `odrv0.reboot()`
* Bold font marks the default configuration.
* If a GPIO is set to an unsupported mode it will be left uninitialized.
* When setting a GPIO to a special purpose mode (e.g. `GPIO_MODE_UART0`) you must also enable the corresponding feature (e.g. `<odrv>.config.enable_uart`).
* Digital mode is a general purpose mode that can be used for these functions: step, dir, enable, encoder index, hall effect encoder, SPI encoder nCS.
* You must also connect GND between ODrive and your other board.
* ODrive v3.3 and onward have 5V tolerant GPIO pins.

## Native Protocol

This protocol is what the ODrive Tool uses to talk to the ODrive. If you have a choice, this is the recommended protocol for all applications. The native protocol runs on USB and can also be configured to run on UART.

### Python

The ODrive Tool you installed as part of the [Getting Started guide](getting-started.md#downloading-and-installing-tools) comes with a library that you can use to easily control the ODrive from Python.

Assuming you already installed the odrive library (`pip install odrive`), the simplest program to control the ODrive is this:

```python
import odrive
odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))
```

For a more comprehensive example, see [tools/odrive_demo.py](../tools/odrive_demo.py).

### Other languages

We don't have an official library for you just yet. Check the community, there might be someone working on it. If you want to write a library yourself, refer to the [native protocol specification](protocol). You are of course welcome to contribute it back.

## ASCII protocol

This is a simpler alternative to the native protocol if you don't need all its bells and whistles. Before you use this, be sure that you're ok with its limitations. The ASCII protocol is enabled by default on UART and can also be enabled on USB alongside with the native protocol.

For more details, see the [ASCII protocol specification](ascii-protocol.md).

### Arduino
There is an Arduino library that gives some examples on how to use the ASCII protocol to communicate with the ODrive. Check it out [here](../Arduino/ODriveArduino).

## Step/direction
This is the simplest possible way of controlling the ODrive. It is also the most primitive and fragile one. So don't use it unless you must interoperate with other hardware that you don't control.

Pinout:
* Step/dir signals: see [Pinout](#pinout) above. Note in that section how to reassign the pins.
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.

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

## RC PWM input
You can control the ODrive directly from a hobby RC receiver.

Any of the numerical parameters that are writable from the ODrive Tool can be hooked up to a PWM input. The [Pinout](#pinout) tells you which pins are PWM input capable. As an example, we'll configure GPIO4 to control the angle of axis 0. We want the axis to move within a range of -2 to 2 turns.

1. Make sure you're able control the axis 0 angle by writing to `odrv0.axis0.controller.input_pos`. If you need help with this follow the [getting started guide](getting-started.md).
2. If you want to control your ODrive with the PWM input without using anything else to activate the ODrive, you can configure the ODrive such that axis 0 automatically goes operational at startup. See [here](commands.md#startup-procedure) for more information.
3. In ODrive Tool, configure the PWM input mapping
    ```
    odrv0.config.gpio4_mode = GPIO_MODE_PWM0
    odrv0.config.gpio4_pwm_mapping.min = -2
    odrv0.config.gpio4_pwm_mapping.max = 2
    odrv0.config.gpio4_pwm_mapping.endpoint = odrv0.axis0.controller._remote_attributes['input_pos']
    ```
   Note: you can disable the input by setting `odrv0.config.gpio4_pwm_mapping.endpoint = None`
4. Save the configuration and reboot
    ```
    odrv0.save_configuration()
    odrv0.reboot()
    ```
5. With the ODrive powered off, connect the RC receiver ground to the ODrive's GND and one of the RC receiver signals to GPIO4. You may try to power the receiver from the ODrive's 5V supply if it doesn't draw too much power. Power up the the RC transmitter. You should now be able to control axis 0 from one of the RC sticks.

Be sure to setup the Failsafe feature on your RC Receiver so that if connection is lost between the remote and the receiver, the receiver outputs 0 for the velocity setpoint of both axes (or whatever is safest for your configuration). Also note that if the receiver turns off (loss of power, etc) or if the signal from the receiver to the ODrive is lost (wire comes unplugged, etc), the ODrive will continue the last commanded velocity setpoint. There is currently no timeout function in the ODrive for PWM inputs.

## Analog input
Analog inputs can be used to measure voltages between 0 and 3.3V. ODrive uses a 12 bit ADC (4096 steps) and so has a maximum resolution of 0.8 mV. A GPIO must be configured with `<odrv>.config.gpioX_mode = GPIO_MODE_ANALOG_IN` before it can be used as an analog input. To read the voltage on GPIO1 in odrivetool the following would be entered: `odrv0.get_adc_voltage(1)`.

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

UART0 is enabled by default with a baudrate of 115200 on the pins as shown in [Pinout](#pinout). Don't forget to also connect GND of the two UART devices. You can use `odrv0.config.uart0_baudrate` to change the baudrate and `odrv0.config.enable_uart0` to disable/reenable UART0.

## CAN Simple Protocol

See [CAN Protocol](can-protocol).
