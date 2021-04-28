# UART Interface

The ODrive's UART interface is enabled by default (see `odrv0.config.enable_uart`). Currently it runs both the [Native Protocol](native-protocol) and the [ASCII Protocol](ascii-protocol) at the same time.

The baudrate is 115200 by default and can be changed using `odrv0.config.uart_baudrate`. Changes to the UART configuration require a reboot to take effect.

The logic level of the ODrive is 3.3V. The GPIOs are 5V tolerant.

Pinout:
* GPIO 1: Tx (connect to Rx of other device)
* GPIO 2: Rx (connect to Tx of other device)
* GND: you must connect the grounds of the devices together. Use any GND pin on J3 of the ODrive.
