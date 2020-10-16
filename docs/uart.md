# UART Interface

The ODrive's UART0 interface is enabled by default with a baudrate of 115200 on the pins as shown in [Pinout](pinout).

To use UART connect it like this:

* Tx of the ODrive <=> Rx of other device
* Rx of the ODrive <=> Tx of other device
* GND of the ODrive (use any GND pin on J3 of the ODrive) <=> GND of the other device

The logic level of the ODrive is 3.3V. The GPIOs are 5V tolerant.

You can use `odrv0.config.uart0_baudrate` to change the baudrate and `odrv0.config.enable_uart0` to disable/reenable UART0. Currently the UART port runs both the [Native Protocol](native-protocol) and the [ASCII Protocol](ascii-protocol) at the same time.
