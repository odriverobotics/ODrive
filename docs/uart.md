# UART Interface

The ODrive's UART_A interface is enabled by default with a baudrate of 115200 on the pins as shown in [Pinout](pinout).

To use UART connect it like this:

* Tx of the ODrive <=> Rx of other device
* Rx of the ODrive <=> Tx of other device
* GND of the ODrive (use any GND pin on J3 of the ODrive) <=> GND of the other device

The logic level of the ODrive is 3.3V. The GPIOs are 5V tolerant.

You can use `odrv0.config.uart_a_baudrate` to change the baudrate and `odrv0.config.enable_uart_a` to disable/reenable UART_A. Currently the UART port runs both the [Native Protocol](native-protocol) and the [ASCII Protocol](ascii-protocol) at the same time.

### How to use UART on GPIO3/4

If you need GPIO1/2 for some function other than UART you can disable UART_A and instead use UART_B on GPIO3/4. Here's how you do it:

    odrv0.config.enable_uart_a = False
    odrv0.config.gpio1_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio2_mode = GPIO_MODE_DIGITAL
    odrv0.config.enable_uart_b = True
    odrv0.config.gpio3_mode = GPIO_MODE_UART_B
    odrv0.config.gpio4_mode = GPIO_MODE_UART_B
    odrv0.reboot()
