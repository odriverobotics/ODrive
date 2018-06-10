# Troubleshooting

## Error codes
If your ODrive is not working as expected, run `odrivetool` and type `hex(<axis>.error)` <kbd>Enter</kbd> where `<axis>` is the axis that isn't working. This will display a [hexadecimal](https://en.wikipedia.org/wiki/Hexadecimal) representation of the error code. Each bit represents one error flag.

<details><summary markdown="span">Example</summary><div markdown="block">

Say you got this error output:
```python
In [1]: hex(odrv0.axis0.error)
Out[1]: '0x6'
```

Written in binary, the number `0x6` corresponds to `110`, that means bits 1 and 2 are set (counting starts at 0).

Looking at the reference below, this means that both `ERROR_DC_BUS_UNDER_VOLTAGE` and `ERROR_DC_BUS_OVER_VOLTAGE` occurred.

</div></details>

There is a slight chance that the values here are out of sync with the actual firmware. To be completely sure, check the linked definition in the source code.

### Axis error flags

Defined [here](../Firmware/MotorControl/axis.hpp)

0. `ERROR_INVALID_STATE`
1. `ERROR_DC_BUS_UNDER_VOLTAGE`
2. `ERROR_DC_BUS_OVER_VOLTAGE`
3. `ERROR_CURRENT_MEASUREMENT_TIMEOUT`
4. `ERROR_BRAKE_RESISTOR_DISARMED`
5. `ERROR_MOTOR_DISARMED`
6. `ERROR_MOTOR_FAILED` (check `.motor.error` for more details)
7. `ERROR_SENSORLESS_ESTIMATOR_FAILED` (check `.sensorless_estimator.error` for more details)
8. `ERROR_ENCODER_FAILED` (check `.encoder.error` for more details)
9. `ERROR_CONTROLLER_FAILED`
10. `ERROR_POS_CTRL_DURING_SENSORLESS`

### Motor error flags

Defined [here](../Firmware/MotorControl/motor.hpp)

0. `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE`
1. `ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE`
2. `ERROR_ADC_FAILED`
3. [`ERROR_DRV_FAULT`](#drv-fault)
4. `ERROR_CONTROL_DEADLINE_MISSED`
5. `ERROR_NOT_IMPLEMENTED_MOTOR_TYPE`
6. `ERROR_BRAKE_CURRENT_OUT_OF_RANGE`
7. `ERROR_NUMERICAL`

### Encoder error flags

Defined [here](../Firmware/MotorControl/encoder.hpp)

0. `ERROR_NUMERICAL`
1. `ERROR_CPR_OUT_OF_RANGE`
2. `ERROR_RESPONSE`

### Sensorless estimator error flags

Defined [here](../Firmware/MotorControl/sensorless_estimator.hpp)

0. `ERROR_NUMERICAL`


## DRV fault

The ODrive v3.4 is known to have a hardware issue whereby the motors would stop operating
when applying high currents to M0. The reported error of both motors in this case
is `ERROR_DRV_FAULT`.

The conjecture is that the high switching current creates large ripples in the
power supply of the DRV8301 gate driver chips, thus tripping its undervoltage
fault detection.

* Limit the M0 current to 40A. The lowest current at which the DRV fault was observed is 45A on one test motor and 50A on another test motor.
* Refer to [this post](https://discourse.odriverobotics.com/t/drv-fault-on-odrive-v3-4/558) for instructions for a hardware fix


## USB Connectivity Issues

 * Try turning it off and on again (the ODrive, the script, the PC)
 * Make sure you're using the latest firmware and python tools release
 * **Linux**: Type `lsusb` to list all USB devices. Verify that your ODrive is listed.
 * **Linux**: Make sure you [set up your udev rules](getting-started#downloading-and-installing-tools) correctly.
 * **Windows**: Right-click on the start menu and open "Device Manager". Verify that your ODrive is listed.
 * **Windows**: Use the [Zadig utility](http://zadig.akeo.ie/) to verify the driver is set to `libusb-win32`.
 * Ensure that no other ODrive program is running
 * Run `odrivetools` with the `--verbose` option.
 * Run `PYUSB_DEBUG=debug odrivetools` to get even more log output.
 * If you're a developer you can use Wireshark to capture USB traffic.
