# Motors & Encoders

[TODO: UPDATE]

## Encoder Calibration
By default the encoder-to-motor calibration will run on every startup. During encoder calibration the rotor must be allowed to rotate without any biased load during startup. That means mass and weak friction loads are fine, but gravity or spring loads are not okay.

### Encoder with Index signal
If you have an encoder with an index (Z) signal, you may avoid having to do the calibration on every startup, and instead use the index signal to re-sync the encoder to a stored calibration. Below are the steps to do the one-time calibration and configuration. Note that you can follow these steps with one motor at a time, or all motors together, as you wish.

* Since you will only do this once, it is recommended that you mechanically disengage the motor from anything other than the encoder, so it can spin freely.
* All the parameters we will be modifying are in the motor structs at the top of [MotorControl/low_level.c](MotorControl/low_level.c).
* Set `.encoder.use_index = true` and `.encoder.calibrated = false`.
* Flash this configuration, and let the motor scan for the index pulse and then complete the encoder calibration.
* Enter the following to print out the calibration parameters (substitute the motor number you are calibrating for `<NUM>`):
  * `odrv0.motor<NUM>.encoder.encoder_offset` - This should print a number, like -326 or 1364.
  * `odrv0.motor<NUM>.encoder.motor_dir` - This should print 1 or -1.
* Copy these numbers to the corresponding entries in low_level.c: `.encoder.encoder_offset` and `.encoder.motor_dir`.
  * _Warning_: Please be careful to enter the correct numbers, and not to confuse the motor channels. Incorrect values may cause the motor to spin out of control.
* Set `.encoder.calibrated = true`.
* Flash this configuration and check that the motor scans for the index pulse but skips the encoder calibration.
* Congratulations, you are now done. You may now attach the motor to your mechanical load.


* If you wish to scan for the index pulse in the other direction (if for example your axis usually starts close to a hard-stop), you can set a negative value in `.encoder.config.idx_search_speed`.
* If your motor has problems reaching the index location due to the mechanical load, you can increase `.encoder.calibration_current`.
