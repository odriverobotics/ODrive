# Pinout

## ODrive v4.1

**TODO**

## ODrive v3.x

| #  | Label         | `GPIO_MODE_DIGITAL`    | `GPIO_MODE_ANALOG_IN` | `GPIO_MODE_UART_A` | `GPIO_MODE_UART_B` | `GPIO_MODE_PWM` | `GPIO_MODE_CAN_A` | `GPIO_MODE_I2C_A` | `GPIO_MODE_ENC0` | `GPIO_MODE_ENC1` | `GPIO_MODE_MECH_BRAKE` |
|----|---------------|------------------------|-----------------------|--------------------|--------------------|-----------------|------------------|-------------------|------------------|------------------|------------------------|
|  0 | _not a pin_   |                        |                       |                    |                    |                 |                  |                   |                  |                  |                        |
|  1 | GPIO1 (+)     | general purpose        | analog input          | **UART_A.TX**      |                    | PWM0.0          |                  |                   |                  |                  | mechanical brake       |
|  2 | GPIO2 (+)     | general purpose        | analog input          | **UART_A.RX**      |                    | PWM0.1          |                  |                   |                  |                  | mechanical brake       |
|  3 | GPIO3         | general purpose        | **analog input**      |                    | **UART_B.TX**      | PWM0.2          |                  |                   |                  |                  | mechanical brake       |
|  4 | GPIO4         | general purpose        | **analog input**      |                    | **UART_B.RX**      | PWM0.3          |                  |                   |                  |                  | mechanical brake       |
|  5 | GPIO5         | general purpose        | **analog input** (*)  |                    |                    |                 |                  |                   |                  |                  | mechanical brake       |
|  6 | GPIO6 (*) (+) | **general purpose**    |                       |                    |                    |                 |                  |                   |                  |                  | mechanical brake       |
|  7 | GPIO7 (*) (+) | **general purpose**    |                       |                    |                    |                 |                  |                   |                  |                  | mechanical brake       |
|  8 | GPIO8 (*) (+) | **general purpose**    |                       |                    |                    |                 |                  |                   |                  |                  | mechanical brake       |
|  9 | M0.A          | general purpose        |                       |                    |                    |                 |                  |                   | **ENC0.A**       |                  |                        |
| 10 | M0.B          | general purpose        |                       |                    |                    |                 |                  |                   | **ENC0.B**       |                  |                        |
| 11 | M0.Z          | **general purpose**    |                       |                    |                    |                 |                  |                   |                  |                  |                        |
| 12 | M1.A          | general purpose        |                       |                    |                    |                 |                  | I2C.SCL           |                  | **ENC1.A**       |                        |
| 13 | M1.B          | general purpose        |                       |                    |                    |                 |                  | I2C.SDA           |                  | **ENC1.B**       |                        |
| 14 | M1.Z          | **general purpose**    |                       |                    |                    |                 |                  |                   |                  |                  |                        |
| 15 | _not exposed_ | general purpose        |                       |                    |                    |                 | **CAN_A.RX**     | I2C.SCL           |                  |                  |                        |
| 16 | _not exposed_ | general purpose        |                       |                    |                    |                 | **CAN_A.TX**     | I2C.SDA           |                  |                  |                        |


(*) ODrive v3.5 and later <br>
(+) On ODrive v3.5 and later these pins have noise suppression filters. This is useful for step/dir input. <br>

## Notes

* Changes to the pin configuration only take effect after `odrv0.save_configuration()` and `odrv0.reboot()`
* Bold font marks the default configuration.
* If a GPIO is set to an unsupported mode it will be left uninitialized.
* When setting a GPIO to a special purpose mode (e.g. `GPIO_MODE_UART_A`) you must also enable the corresponding feature (e.g. `<odrv>.config.enable_uart_a`).
* Digital mode is a general purpose mode that can be used for these functions: step, dir, enable, encoder index, hall effect encoder, SPI encoder nCS.
* You must also connect GND between ODrive and your other board.
* ODrive v3.3 and onward have 5V tolerant GPIO pins.
* Simultaneous operation of UART_A and UART_B is currently not supported.
