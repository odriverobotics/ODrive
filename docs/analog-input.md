# Analog Input

Analog inputs can be used to measure voltages between 0 and 3.3V. ODrive uses a 12 bit ADC (4096 steps) and so has a maximum resolution of 0.8 mV. A GPIO must be configured with `<odrv>.config.gpioX_mode = GPIO_MODE_ANALOG_IN` before it can be used as an analog input. To read the voltage on GPIO1 in odrivetool the following would be entered: `odrv0.get_adc_voltage(1)`.

Similar to RC PWM input, analog inputs can also be used to feed any of the numerical properties that are visible in `odrivetool`. This is done by configuring `odrv0.config.gpio3_analog_mapping` and `odrv0.config.gpio4_analog_mapping`. Refer to [RC PWM](rc-pwm) for instructions on how to configure the mappings.
