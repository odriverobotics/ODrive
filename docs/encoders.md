# Encoders

## Known and Supported Encoders
Be sure to read the [ODrive Encoder Guide](https://docs.google.com/spreadsheets/d/1OBDwYrBb5zUPZLrhL98ezZbg94tUsZcdTuwiVNgVqpU).

## Encoder Calibration
Please take into account that all encoder types supported by ODrive require that you do some sort of encoder calibration. This requires the following:
* selecting an encoder and mounting it to your motor
* choosing an interface (e.g., AB, ABI or SPI)
* connecting the pins to the odrive
* loading the correct odrive firmware (the default will work in many cases)
* motor calibration
* saving the settings in the odrive for correct bootup

### Encoder without index signal

During encoder offset calibration the rotor must be allowed to rotate without any biased load during startup. That means mass and weak friction loads are fine, but gravity or spring loads are not okay.

In the `odrivetool`, type `<axis>.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION` <kbd>Enter</kbd>.

To verify everything went well, check the following variables:

 * `<axis>.error` should be 0.
 * `<axis>.encoder.config.offset` - This should print a number, like -326 or 1364.
 * `<axis>.motor.config.direction` - This should print 1 or -1.

### Encoder with index signal
If you have an encoder with an index (Z) signal, you may avoid having to do the offset calibration on every startup, and instead use the index signal to re-sync the encoder to a stored calibration.

Below are the steps to do the one-time calibration and configuration. Note that you can follow these steps with one motor at a time, or all motors together, as you wish.

* Since you will only do this once, it is recommended that you mechanically disengage the motor from anything other than the encoder, so it can spin freely.
* Set `<axis>.encoder.config.use_index` to `True`.
* Run `<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH`. This will make the motor turn in one direction until it finds the encoder index.
* Follow the calibration instructions for an [encoder without index signal](#encoder-without-index-signal).
* Set `<axis>.encoder.config.pre_calibrated` to `True` to confirm that the offset is valid with respect to the index pulse.
* If you would like to search for the index at startup, set `<axis>.config.startup_encoder_index_search` to `True`.
  * If you'd rather do it manually, just run `<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH` on every bootup.
* If you are looking to start your machine as quickly as possible on bootup, also set `<axis>.motor.config.pre_calibrated` to `True` to save the current motor calibration and avoid doing it again on bootup.
* Save the configuration by typing `<odrv>.save_configuration()` <kbd>Enter</kbd>.

That's it, now on every reboot the motor will turn in one direction until it finds the encoder index.

* If you wish to scan for the index pulse in the other direction (if for example your axis usually starts close to a hard-stop), you can set a negative value in `<axis>.encoder.config.idx_search_speed`.
* If your motor has problems reaching the index location due to the mechanical load, you can increase `<axis>.motor.config.calibration_current`.

### Startup sequence notes

The following are variables that MUST be set up for your encoder configuration. Your values will vary depending on your encoder:

* `<axis>.encoder.config.cpr = 4000`
* `<axis>.encoder.config.mode = ENCODER_MODE_INCREMENTAL`

The following are examples of values that MAY impact the success of calibration. Your values will vary depending on your setup:
* `<axis>.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT`
* `<axis>.encoder.config.calib_range = 0.05`
* `<axis>.motor.config.calibration_current = 10.0`
* `<axis>.motor.config.resistance_calib_max_voltage = 12.0`
* `<axis>.controller.config.vel_limit = 50000`

Lots of other values can get you. It's a process. Thankfully there is a lot of good people that will help you debug calibration problems. 

If calibration works, congratulations.

Now try: 
* `<axis>.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`
* `<axis>.controller.set_vel_setpoint(3000,0) `
let it loop a few times and then set:
* `<axis>.requested_state = AXIS_STATE_IDLE`

Do you still have no errors? Awesome. Now set these variables:
* `<axis>.encoder.config.pre_calibrated = True`
* `<axis>.motor.config.pre_calibrated  = True `

And see if ODrive agrees that calibration worked by just running
* `<axis>.encoder.config.pre_calibrated`

(using no "= True" ). Make sure that 'pre_calibrated' is in fact True. 

Also, if you have calibrated and encoder.pre_calibrated is equal to true, and you had no errors so far. Run this: 
* `<axis>.encoder.config.use_index = True`
* `odrv0.save_configuration()`
* `odrv0.reboot()`

and now see if after a reboot you can run: 
* `<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH`

without getting errors. 

## Encoder Problems

There are several issues that may prevent you from completing encoder calibration. 

ODrive may not complete the calibrate sequence when you go to:
* `<axis>.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`

It completes the calibrate sequence after:
* `<axis>.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`

but fails after you go to:
* `odrvN.axisN.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`

Or maybe it vibrates in an entertaining way. See:
https://www.youtube.com/watch?v=gaRUmwvSyAs

## Encoder Signals

If your encoder is properly connected, run the command:
* `<axis>.encoder.shadow_count `

and look at your value. Then turn your motor by hand and see if that value changes. Also, notice that the command:
* `<axis>.encoder.config.cpr = 4000`

must reflect the number of counts odrive receives after one complete turn of the motor. So use shadow_count to test if that is working properly. 

You will probably never be able to properly debug if you have problems unless you use an oscilloscope. If you have one, try the following:
Connect to the AB pins, see if you get square waves as you turn the motor.
Connect to the I pin, see if you get a pulse on a complete rotation. Sometimes this is hard to see.
If you are using SPI, connect to CLK, and CS pins, look for a signal like this:
[INSERT PIC]

## Encoder Noise
Noise is found in all circuits, life is just about figuring out if it is preventing your system from working. Lots of users have no problems with noise interferring with their odrive operation, others will tell you "_I've been using the same encoder as you with no problems_". Power to 'em, that may be true, but it doesn't mean it will work for you. If you are concerned about noise, there are several possible sources:

* Encoder wires may be too close to motor wires
* Long wires between encoder and ODrive
* Use of ribbon cable

The following _might_ mitigate noise problems. Use twisted pairs, where one side of each twisted pair is tied to ground, the other side is tied to your signal. If you are using SPI, use a 20-50 ohm resistor in series on CLK, which is more susceptable noise.

## AS5047/AS5048 Encoders
The AS5047/AS5048 encoders are Hall Effect/Magenetic sensors that can serve as rotary encoders for the ODrive.

The AS5047 has 3 independent output interfaces: SPI, ABI, and PWM. 
The AS5048 has 4 independent output interfaces: SPI, ABI, I2C, and PWM.

Both chips come with evaluation boards that can simplify mounted the chips to your motor. For our purposes if you are using an evaluation board you should select the settings for 3.2v, and tie MOSI high to 3.2v. 

If you are having calibration problems - make sure your magnet is centered on the axis of rotation on the motor, some users report this has a significant impact on calibration. Also make sure your magnet height is within range of the spec sheet. 

#### Using ABI. 
You can use ABI with the AS5047/AS5048 with the default ODrive firmware. For your wiring, connect A, B, 3.2v, GND to the labeled pins on the odrive
The acronym I and Z mean the same thing, connect those as well if you are using an index signal. 

#### Using SPI.
TobinHall has written a [branch](https://github.com/TobinHall/ODrive/tree/Non-Blocking_Absolute_SPI) that supports the SPI option on the AS5047/AS5048. Use his build to flash firmware on your ODrive and connect MISO, SCK, and CS to the labeled pins on the odrive

Tie MOSI to 3.2v, connect to the SCK, CLK, MISO, GND and 3.2v pins on the ODrive. (note for SPI users, the acronym SCK and CLK mean the same thing, the acronym CSn and CS mean the same thing.)

Add these commands to your calibration / startup script:
* `<axis>.encoder.config.abs_spi_cs_gpio_pin = 4` or which ever GPIO pin you choose
* `<axis>.encoder.config.mode = 257`
* `<odrv>.axis0.encoder.config.cpr = 2**14`
