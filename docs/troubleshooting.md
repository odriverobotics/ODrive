# Troubleshooting

Table of Contents:
<!-- TOC depthFrom:2 depthTo:2 -->

- [Error codes](#error-codes)
- [Common Axis Errors](#common-axis-errors)
- [Common Motor Errors](#common-motor-errors)
- [Common Encoder Errors](#common-encoder-errors)
- [USB Connectivity Issues](#usb-connectivity-issues)
- [Firmware Issues](#firmware-issues)
- [Other issues that may not produce an error code](#other-issues-that-may-not-produce-an-error-code)

<!-- /TOC -->

## Error codes
If your ODrive is not working as expected, run `odrivetool` and type `dump_errors(odrv0)` <kbd>Enter</kbd>. This will dump a list of all the errors that are present. To also clear all the errors, you can run `dump_errors(odrv0, True)`.

The following sections will give some guidance on the most common errors. You may also check the code for the full list of errors:
* Axis error flags defined [here](../Firmware/MotorControl/axis.hpp).
* Motor error flags defined [here](../Firmware/MotorControl/motor.hpp).
* Encoder error flags defined [here](../Firmware/MotorControl/encoder.hpp).
* Controller error flags defined [here](../Firmware/MotorControl/controller.hpp).
* Sensorless estimator error flags defined [here](../Firmware/MotorControl/sensorless_estimator.hpp).

## Common Axis Errors 

* `ERROR_INVALID_STATE = 0x01`

You tried to run a state before you are allowed to. Typically you tried to run encoder calibration or closed loop control before the motor was calibrated, or you tried to run closed loop control before the encoder was calibrated.

* `ERROR_DC_BUS_UNDER_VOLTAGE = 0x02`

Confirm that your power leads are connected securely. For initial testing a 12V PSU which can supply a couple of amps should be sufficient while the use of low current 'wall wart' plug packs may lead to inconsistent behaviour and is not recommended. 

You can monitor your PUS voltage using liveplotter in odrive tool by entering `start_liveplotter(lambda: [odrv0.vbus_voltage])`. If you see your votlage drop below ~ 8V then you will trip this error. Even a relatively small motor can draw multiple kW momentary and so unless you have a very large PSU or are running of a battery you may encounter this error when executing high speed movements with a high current limit. To limit your PSU power draw you can limit your motor current and/or velocity limit `odrv0.axis0.controller.config.vel_limit` and `odrv0.axis0.motor.config.current_lim`.

* `ERROR_DC_BUS_OVER_VOLTAGE = 0x04`

Confirm that you have a brake resistor of the correct value connected securly and that `odrv0.config.brake_resistance` is set to the value of your brake resistor. 

You can monitor your PUS voltage using liveplotter in odrive tool by entering `start_liveplotter(lambda: [odrv0.vbus_voltage])`. If during a move you see the voltage rise above your PSU's nominal set voltage then you have your brake resistance set too low. This may happen if you are using long wires or small gauge wires to connect your brake resistor to your odrive which will added extra resistance. This extra resistance needs to be accounted for to prevent this voltage spike. If you have checked all your connections you can also try increasing your brake resistance by ~ 0.01 Ohm at a time to a maximum of 0.05 greater than your brake resistor value.

## Common Motor Errors 

* `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001` and `ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002`

During calibration the motor resistance and [inductance](https://en.wikipedia.org/wiki/Inductance) is measured. If the measured motor resistance or inductance falls outside a set range this error will be returned. Check that all motor leads are connected securely.

The measured values can be viewed using odrivetool as is shown below:
```
In [2]: odrv0.axis0.motor.config.phase_inductance
Out[2]: 1.408751450071577e-05

In [3]: odrv0.axis0.motor.config.phase_resistance
Out[3]: 0.029788672924041748
```
Some motors will have a considerably different phase resistance and inductance than this. For example, gimbal motors, some small motors (e.g. < 10A peak current). If you think this applies to you try increasing `odrv0.axis0.motor.config.resistance_calib_max_voltage` from its default value of 1 using odrivetool and repeat the motor calibration process. If your motor has a small peak current draw (e.g. < 20A) you can also try decreasing `odrv0.axis0.motor.config.calibration_current` from its default value of 10A.

In general, you need
```text
resistance_calib_max_voltage > calibration_current * phase_resistance
resistance_calib_max_voltage < 0.5 * vbus_voltage
```

* `ERROR_DRV_FAULT = 0x0008`

The ODrive v3.4 is known to have a hardware issue whereby the motors would stop operating
when applying high currents to M0. The reported error of both motors in this case
is `ERROR_DRV_FAULT`.

The conjecture is that the high switching current creates large ripples in the
power supply of the DRV8301 gate driver chips, thus tripping its under-voltage fault detection. 

To resolve this issue you can limit the M0 current to 40A. The lowest current at which the DRV fault was observed is 45A on one test motor and 50A on another test motor. Refer to [this post](https://discourse.odriverobotics.com/t/drv-fault-on-odrive-v3-4/558) for instructions for a hardware fix.

* `ERROR_MODULATION_MAGNITUDE = 0x0080`

The bus voltage was insufficent to push the requested current through the motor. Reduce `motor.config.calibration_current` and/or `motor.config.current_lim`, for errors at calibration-time and closed loop control respectively.

For gimbal motors, it is recommended to set the calibration_current and current_lim to half your bus voltage, or less.

## Common Encoder Errors

* `ERROR_CPR_OUT_OF_RANGE = 0x02`

Confirm you have entered the correct count per rotation (CPR) for [your encoder](https://docs.odriverobotics.com/encoders). Note that the AMT encoders are configurable using the micro-switches on the encoder PCB and so you may need to check that these are in the right positions. If your encoder lists its pulse per rotation (PPR) multiply that number by four to get CPR.

* `ERROR_NO_RESPONSE = 0x04`

Confirm that your encoder is plugged into the right pins on the odrive board.

* `ERROR_INDEX_NOT_FOUND_YET = 0x20`

Check that your encoder is a model that has an index pulse. If your encoder does not have a wire connected to pin Z on your odrive then it does not output an index pulse.


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
 * Try a different USB cable
 * Try routing your USB cable so that it is far away from the motor and PSU cables to reduce EMI

## Firmware Issues

### Failure to build the firmware when running `make`
- Clear out temporary files from previous compiles by first running `make clean` to prevent conflicts.
- **Windows users**: Confirm that tup has been correctly added to path by running `env|grep PATH` in Git Bash. If you see no mention of tup then you must [add its location to your PATH environment variable.](https://docs.alfresco.com/4.2/tasks/fot-addpath.html). Note that you may need to restart for the added path to take effect.

### Failure to flash the firmware when running `make flash`
- If using an ST-link, confirm that the ST-link is connected the correct pins and that you have power supplied to the board. This can be by the 5V pin on the ST link or the main DC power jack. No power is supplied over the USB connection.

## Other issues that may not produce an error code

###  Motor cuts off or spins uncontrollably at high rotational speeds (ie: > 5000 RPM)
- You may be approaching the limit of your encoder. The 2400 count/rotation encoders that were initially included with odrive are realistically limited to around 5000 RPM. Exceeding this speed causes the odrive to lose track of position. This can only be fixed by using an alternative encoder or gearing down the output of your motor onto your encoder so that it still sees < 5000RPM at full speed. If using the gearing options be sure to change your counts/rotation accordingly.

### Motor vibrates when stationary or makes constant noise

- Likely due to incorrect gains, specifically `vel_gain` may be set too high. Try following the [tuning procedure](https://docs.odriverobotics.com/commands).
- Check encoder shaft connection. Grub screws may vibrate lose with time. If using a CUI shaft encoder try remounting the plastic retaining ring and confirm that it is not coming into contact with the encoder housing. Also confirm that the encoder is securely mounted.
- If you are using a high resolution encoder (>4000 counts/rotation) then increasing encoder_pll_bandwidth may help reduce vibration.
- If you connect your motor to an object with a large moment of inertia (such as a flywheel) this will help reduce vibrations at high gians. However, make sure that all connections are ridged. Cheap shaft couplers or belts under low tension can introduce enough flex into a system that the motor may still vibrate independently.

### Motor overshoots target position or oscillates back and forth
- Likely due to incorrect gains for a given motor current limit. Specifically `pos_gain` is set too high. Try following the [tuning procedure](https://docs.odriverobotics.com/commands).
- Increase the current limit of your motor for more torque.

### Motor slowly starts to increase in speed
- Encoder has likely slipped. This may occur when your motor makes a hard stop or violently vibrates causing something to come lose. Power the board off and on again so that it undertakes a new calibration. If you are using an index search on startup then you will need to repeat the index calibration process.

### Motor feels like it has less torque than it should and/or gets hot sitting still while under no load.
- Encoder has likely slipped causing the motor controller to commutate the wrong windings slightly which reduces output torque and produces excess heat as the motor 'fights itself'.

### False steps or direction changes when using step/dir
- Prior to Odrive board V3.5 no filtering is present on the GPIO pins used for step/dir interface and so inductively coupled noise may causes false steps to be detected. Odrive V3.5 and has onboard filtering to resolve this issue.
- If you experience this issue use a twisted pair cable between your microcontroller that’s generating the step/dir signals and your odrive board. A section cut from cat-5 cable works well as does just twisting some normal insulated wire together.
- Ensure that the step/dir signal cables are not draped over the odrive board, are not running in parallel to the motor or power supply cables.
- If the above does not resolve your issue on V3.4 boards and lower try adding a ~22 Ohm resistor in series with the step and direction pins along with a ~ 4.7 nF capacitor between the ground pin and the step and dir pins such as shown [here](https://cdn.discordapp.com/attachments/369667319280173069/420811057431445504/IMG_20180306_211224.jpg).
