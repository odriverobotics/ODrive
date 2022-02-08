.. _pinout-chart:

================================================================================
ODrive v3.x Pinout
================================================================================

.. contents::
   :depth: 1
   :local:
   
.. ODrive v4.1
.. --------------------------------------------------------------------------------

.. **TODO**

.. ODrive v3.x
.. --------------------------------------------------------------------------------

.. csv-table:: Pinout Table
   :file: figures/pinout.csv
   :header-rows: 1

**key:**

* **(*)** ODrive v3.5 and later.

* **(+)** On ODrive v3.5 and later these pins have noise suppression filters. This is useful for step/dir input.

Notes
--------------------------------------------------------------------------------

* Changes to the pin configuration only take effect after :code:`odrv0.save_configuration()` and :code:`odrv0.reboot()`
* **Bold font** marks the default configuration.
* If a GPIO is set to an unsupported mode it will be left uninitialized.
* When setting a GPIO to a special purpose mode (e.g. :code:`GPIO_MODE_UART_A`) you must also enable the corresponding feature (e.g. :code:`<odrv>.config.enable_uart_a`).
* Digital mode is a general purpose mode that can be used for these functions: step, dir, enable, encoder index, hall effect encoder, SPI encoder nCS.
* You must also connect GND between ODrive and your other board.
* ODrive v3.3 and onward have 5V tolerant GPIO pins.
* Simultaneous operation of UART_A and UART_B is currently not supported.
