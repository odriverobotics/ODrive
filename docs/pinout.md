# Pinout

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

Notes:
* You must also connect GND between ODrive and your other board.
* ODrive v3.3 and onward have 5V tolerant GPIO pins.
* ODrive v3.5 and later have some noise suppression filters on the default step/dir pins
* You can change the step/dir pins using `axis.config.<step/dir>_gpio_pin`.

### Pin function priorities
1. PWM in, if enabled. Disabled by default.
1. UART, **Enabled by default**.
1. Step/Dir, if enabled. Disabled by default.
1. Analog, default behavior if not overridden (only on supported pins).
1. Digital in, default behavior on pins not capable of analog input.

For predictable results, try to have only one feature enabled for any one pin. When changing pin assignments you must:
* `odrv0.save_configuration()`
* `odrv0.reboot()`

### Analog input
Analog inputs can be used to measure voltages between 0 and 3.3V. Odrive uses a 12 bit ADC (4096 steps) and so has a maximum resolution of 0.8 mV. Some GPIO pins require the appropriate pin priority (see above) to be set before they can be used as an analog input. To read the voltage on GPIO1 in odrive tool the following would be entered: `odrv0.get_adc_voltage(1)`