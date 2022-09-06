================================================================================
Analog Input
================================================================================

Analog inputs can be used to measure voltages between 0 and 3.3V. 
ODrive uses a 12 bit ADC (4096 steps) and so has a maximum resolution of 0.8 mV. 
A GPIO must be configured with :code:`<odrv>.config.gpioX_mode = GPIO_MODE_ANALOG_IN` before it can be used as an analog input. 
To read the voltage on GPIO1 in odrivetool the following would be entered: :code:`odrv0.get_adc_voltage(1)`.

Similar to RC PWM input, analog inputs can also be used to feed any of the numerical properties that are visible in :code:`odrivetool`. 
This is done by configuring :code:`odrv0.config.gpio3_analog_mapping` and :code:`odrv0.config.gpio4_analog_mapping`. 
Refer to :ref:`RC PWM <rc-pwm-doc>` for instructions on how to configure the mappings.

You may also retrieve voltage measurements from analog inputs via the CAN protocol by sending the Get ADC Voltage message with the GPIO number of the analog input you wish to read. Refer to :ref: `CAN Protocol <can-protocol-doc>` for guidance on how to use the CAN Protocol.