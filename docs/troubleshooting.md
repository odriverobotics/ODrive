# Troubleshooting

## Error codes
If your ODrive is not working as expected, run `odrivetool` and type `hex(odrv0.axis0.error)` <kbd>Enter</kbd>. This will display a [hexadecimal](https://en.wikipedia.org/wiki/Hexadecimal) representation of the error code. Each bit represents one error flag.

<details><summary markdown="span">Example</summary><div markdown="block">

Say you typed `hex(odrv0.axis0.error)` and got `0x6` as a result. Written in binary this number corresponds to `110`, so bits 1 and 2 are set.

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

Defined [here](MotorControl/motor.hpp)

0. ERROR_PHASE_RESISTANCE_OUT_OF_RANGE
1. ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE
2. ERROR_ADC_FAILED
3. ERROR_DRV_FAULT
4. ERROR_CONTROL_DEADLINE_MISSED
5. ERROR_NOT_IMPLEMENTED_MOTOR_TYPE
6. ERROR_BRAKE_CURRENT_OUT_OF_RANGE
7. ERROR_NUMERICAL

### Encoder error flags

Defined [here](MotorControl/encoder.hpp)

0. ERROR_NUMERICAL
1. ERROR_CPR_OUT_OF_RANGE
2. ERROR_RESPONSE

### Sensorless estimator error flags

Defined [here](MotorControl/sensorless_estimator.hpp)

0. ERROR_NUMERICAL


### Cannot connect to the ODrive

ensure no other ODrive program is running
prepend `PYUSB_DEBUG=debug` to the script

## USB issues ##

* Firmware:
  1. ODrive Firmware
  2. STM HAL code
* Electrical:
  1. ODrive hardware
  2. connection (cables, hub)
* PC side
  1. PC-side USB host controller
  2. kernel
  3. libusb driver
  4. libusb library
  5. PyUSB
  6. python code



## DRV fault ##

Hardware: 5330 (190kv), v3.4-48V, V_bus=12V
Settings: default gains, current_lim=50A, position control, stationary
Action: Applying a high torque manually
Result: Trips at I_q=45A (+- 2A) on the big motor, I_q=50A (+-1A) on the black motor, I_bus=8.3A (+-0.4),

phase=-2.3
-2.7 (did not trip from -0.5 to -2.7 @ 50A)
-2.8
-2.7
-2.4689412117004395
0.7
0.5

other odrive:
-0.8651647567749023

Repeatability: about 5/5


```
ODrive control utility v0.3.7.dev
Please connect your ODrive.
Type help() for help.

Connected to ODrive 385F324D3037 as odrv1
In [1]: odrv1.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
In [2]: start_liveplotter(lambda: [odrv1.axis0.motor.current_control.Ibus, odrv1.axis0.motor.current_control.Iq_setpoint])
In [4]: odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
In [9]: odrv1.axis0.motor.config.current_lim=50
In [11]: odrv1.axis0.controller.config.vel_limit
Out[11]: 20000.0
In [12]: odrv1.axis0.controller.config.vel_gain
Out[12]: 0.0005000000237487257
```



```
In [9]: odrive.utils.print_drv_regs("M0",odrv1.axis0.motor)
M0: 0
DRV Fault Code: 0
Status Reg 1: 0 (0b00000000)
Status Reg 2: 1 (0b00000001)
Control Reg 1: 1360 (0b10101010000)
Control Reg 2: 8 (0b0001000)

In [10]: odrive.utils.print_drv_regs("M1",odrv1.axis0.motor)
M1: 0
DRV Fault Code: 0
Status Reg 1: 0 (0b00000000)
Status Reg 2: 1 (0b00000001)
Control Reg 1: 1360 (0b10101010000)
Control Reg 2: 8 (0b0001000)

In [11]: (hex(odrv1.axis0.error), hex(odrv1.axis0.motor.error))
Out[11]: ('0x41', '0x18')

In [12]: (hex(odrv1.axis1.error), hex(odrv1.axis1.motor.error))
Out[12]: ('0x61', '0x10')
```
