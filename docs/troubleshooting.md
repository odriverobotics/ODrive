# Troubleshooting

Table of Contents:
<!-- TOC depthFrom:2 depthTo:2 -->

- [Error codes](#error-codes)
- [USB Connectivity Issues](#usb-connectivity-issues)
- [Firmware Issues](#firmware-issues)
- [Other issues that may not produce an error code](#other-issues-that-may-not-produce-an-error-code)

<!-- /TOC -->

## Error codes
If your ODrive is not working as expected, run `odrivetool` and type `dump_errors(odrv0)` <kbd>Enter</kbd>. This will dump a list of all the errors that are present. To clear all the errors, you can run `odrv0.clear_errors()`.

With this information you can look up the API documentation for your error(s):
* System error flags documented [here](api/odrive.error).
* Axis error flags documented [here](api/odrive.axis.error).
* Motor error flags documented [here](api/odrive.motor.error).
* Encoder error flags documented [here](api/odrive.encoder.error).
* Controller error flags documented [here](api/odrive.controller.error).
* Sensorless estimator error flags documented [here](odrive.sensorlessestimator.error).

### What if `dump_errors()` gives me python errors? 
If you get output like this:
  <details><summary markdown="span">Show code:</summary><div markdown="block">

```py
In [1]: dump_errors(odrv0)
axis0
---------------------------------------------------------------------------
AttributeError                            Traceback (most recent call last)
~/.local/lib/python3.6/site-packages/fibre/shell.py in <module>
----> 1 dump_errors(odrv0)

~/.local/lib/python3.6/site-packages/odrive/utils.py in dump_errors(odrv, clear)
     78             ('axis', axis, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("AXIS_ERROR_")}),
     79             ('motor', axis.motor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("MOTOR_ERROR_")}),
---> 80             ('fet_thermistor', axis.fet_thermistor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("THERMISTOR_CURRENT_LIMITER_ERROR")}),
     81             ('motor_thermistor', axis.motor_thermistor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("THERMISTOR_CURRENT_LIMITER_ERROR")}),
     82             ('encoder', axis.encoder, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("ENCODER_ERROR_")}),

~/.local/lib/python3.6/site-packages/fibre/remote_object.py in __getattribute__(self, name)
    243             return attr
    244         else:
--> 245             return object.__getattribute__(self, name)
    246             #raise AttributeError("Attribute {} not found".format(name))
    247 

AttributeError: 'RemoteObject' object has no attribute 'fet_thermistor'
```

  </div></details>
when you call `dump_errors()`, you have a version mismatch between odrivetool and the firmware on your ODrive. To get the newest version of odrivetool, you can run `pip install odrive --upgrade`. To get the newest ODrive firmware, run `odrivetool dfu`. See the [odrivetool](odrivetool.md#device-firmware-update) page for more details.

## USB Connectivity Issues

 * Try turning it off and on again (the ODrive, the script, the PC)
 * Make sure you're using the latest firmware and python tools release
 * **Linux**: Type `lsusb` to list all USB devices. Verify that your ODrive is listed.
 * **Linux**: Make sure you [set up your udev rules](getting-started#downloading-and-installing-tools) correctly.
 * **Windows**: Right-click on the start menu and open "Device Manager". Verify that your ODrive is listed.
 * **Windows**: Use the [Zadig utility](http://zadig.akeo.ie/) to verify the driver is set to `WinUSB` or `libusb-win32`. Note that there are two options listed in Zadig for ODrive: `ODrive 3.x Native Interface (Interface 2)` and `ODrive 3.x CDC Interface (Interface 0)`. Only the driver setting of the native interface is important to `odrivetool`.
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

- Likely due to incorrect gains, specifically `vel_gain` may be set too high. Try following the [tuning procedure](control.md#Tuning).
- Check encoder shaft connection. Grub screws may vibrate lose with time. If using a CUI shaft encoder try remounting the plastic retaining ring and confirm that it is not coming into contact with the encoder housing. Also confirm that the encoder is securely mounted.
- If you are using a high resolution encoder (>4000 counts/rotation) then increasing encoder_pll_bandwidth may help reduce vibration.
- If you connect your motor to an object with a large moment of inertia (such as a flywheel) this will help reduce vibrations at high gians. However, make sure that all connections are ridged. Cheap shaft couplers or belts under low tension can introduce enough flex into a system that the motor may still vibrate independently.

### Motor overshoots target position or oscillates back and forth
- Likely due to incorrect gains for a given motor current limit. Specifically `pos_gain` is set too high. Try following the [tuning procedure](control.md#Tuning).
- Increase the current limit of your motor for more torque.

### Motor slowly starts to increase in speed
- Encoder has likely slipped. This may occur when your motor makes a hard stop or violently vibrates causing something to come lose. Power the board off and on again so that it undertakes a new calibration. If you are using an index search on startup then you will need to repeat the index calibration process.

### Motor feels like it has less torque than it should and/or gets hot sitting still while under no load.
- Encoder has likely slipped causing the motor controller to commutate the wrong windings slightly which reduces output torque and produces excess heat as the motor 'fights itself'.
- This can also be caused if the rotor bell slips on the motor shaft. On some motors the rotor bell is secured against the shaft with a grub screw. Confirm that this screw is tight enough. For further details on how to resolve this issue see [this forum post](https://discourse.odriverobotics.com/t/motor-gets-hot-has-less-torque-in-one-direction-than-the-other/2394).

### False steps or direction changes when using step/dir
- Prior to Odrive board V3.5 no filtering is present on the GPIO pins used for step/dir interface and so inductively coupled noise may causes false steps to be detected. Odrive V3.5 and has onboard filtering to resolve this issue.
- If you experience this issue use a twisted pair cable between your microcontroller that’s generating the step/dir signals and your odrive board. A section cut from cat-5 cable works well as does just twisting some normal insulated wire together.
- Ensure that the step/dir signal cables are not draped over the odrive board, are not running in parallel to the motor or power supply cables.
- If the above does not resolve your issue on V3.4 boards and lower try adding a ~22 Ohm resistor in series with the step and direction pins along with a ~ 4.7 nF capacitor between the ground pin and the step and dir pins such as shown [here](https://cdn.discordapp.com/attachments/369667319280173069/420811057431445504/IMG_20180306_211224.jpg).
