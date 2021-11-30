.. _uart-doc:

================================================================================
UART Interface
================================================================================

The ODrive's :code:`UART_A` interface is enabled by default with a baudrate of 115200 on the pins as shown in :ref:`Pinout <pinout-chart>`.

To use UART connect it like this:

* Tx of the ODrive <=> Rx of other device
* Rx of the ODrive <=> Tx of other device
* GND of the ODrive (use any GND pin on J3 of the ODrive) <=> GND of the other device

The logic level of the ODrive is 3.3V. The GPIOs are 5V tolerant.

You can use :code:`odrv0.config.uart_a_baudrate` to change the baudrate and :code:`odrv0.config.enable_uart_a` to disable/reenable :code:`UART_A`. 
The :code:`UART_A` port can run the :ref:`Native Protocol <native-protocol>` or the :ref:`ASCII Protocol <ascii-protocol>`, but not both at the same time. 
You can configure this by setting :code:`odrv0.config.uart0_protocol` to either :code:`STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT` for the ASCII protocol or :code:`STREAM_PROTOCOL_TYPE_FIBRE` for the native protocol.

How to use UART on GPIO3/4
--------------------------------------------------------------------------------

If you need GPIO1/2 for some function other than UART you can disable :code:`UART_A` and instead use :code:`UART_B` on GPIO3/4. Here's how you do it:

.. code:: iPython
    
    odrv0.config.enable_uart_a = False
    odrv0.config.gpio1_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio2_mode = GPIO_MODE_DIGITAL
    odrv0.config.enable_uart_b = True
    odrv0.config.gpio3_mode = GPIO_MODE_UART_B
    odrv0.config.gpio4_mode = GPIO_MODE_UART_B
    odrv0.reboot()
