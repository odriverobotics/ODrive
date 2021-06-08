# Encoders

## Known and Supported Encoders
Be sure to read the [ODrive Encoder Guide](https://docs.google.com/spreadsheets/d/1OBDwYrBb5zUPZLrhL98ezZbg94tUsZcdTuwiVNgVqpU).

## Encoder Calibration
All encoder types supported by ODrive require that you do some sort of encoder calibration. This requires the following:
* Selecting an encoder and mounting it to your motor
* Choosing an interface (e.g., AB, ABI or SPI)
* Connecting the pins to the odrive
* Loading the correct odrive firmware (the default will work in many cases)
* Motor calibration
* Saving the settings in the odrive for correct bootup

### Encoder without index signal
During encoder offset calibration the rotor must be allowed to rotate without any biased load during startup. That means mass and weak friction loads are fine, but gravity or spring loads are not okay.

In the `odrivetool`, type `<axis>.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION` <kbd>Enter</kbd>.

To verify everything went well, check the following variables:

 * `<axis>.error` should be 0.
 * `<axis>.encoder.config.phase_offset` - This should print a number, like -326 or 1364.
 * `<axis>.encoder.config.direction` - This should print 1 or -1.

### Encoder with index signal
If you have an encoder with an index (Z) signal, you can avoid doing the offset calibration on every startup, and instead use the index signal to re-sync the encoder to a stored calibration.

Below are the steps to do the one-time calibration and configuration. Note that you can follow these steps with one motor at a time, or all motors together, as you wish.

* Since you will only do this once, it is recommended that you mechanically disengage the motor from anything other than the encoder, so that it can spin freely.
* Set `<axis>.encoder.config.use_index` to `True`.
* Run `<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH`. This will make the motor turn in one direction until it finds the encoder index.
* Follow the calibration instructions for an [encoder without index signal](#encoder-without-index-signal).
* Set `<axis>.encoder.config.pre_calibrated` to `True` to confirm that the offset is valid with respect to the index pulse.
* If you would like to search for the index at startup, set `<axis>.config.startup_encoder_index_search` to `True`.
  * If you'd rather do it manually, just run `<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH` on every bootup.
* If you are looking to start your machine as quickly as possible on bootup, also set `<axis>.motor.config.pre_calibrated` to `True` to save the current motor calibration and avoid doing it again on bootup.
* Save the configuration by typing `<odrv>.save_configuration()` <kbd>Enter</kbd>.

That's it, now on every reboot the motor will turn in one direction until it finds the encoder index.

* If your motor has problems reaching the index location due to the mechanical load, you can increase `<axis>.motor.config.calibration_current`.

### Reversing index search
Sometimes you would like the index search to only happen in a particular direction (the reverse of the default). Instead of swapping the motor leads, you can ensure that the following three values are negative:
* `<axis0>.config.calibration_lockin.vel`
* `<axis0>.config.calibration_lockin.accel`
* `<axis0>.config.calibration_lockin.ramp_distance`


*IMPORTANT:* Your motor should find the same rotational position when the ODrive performs an index search if the index signal is working properly. This means that the motor should spin, and stop at the same position if you have set <axis>.config.startup_encoder_index_search so the search starts on reboot, or you if call the command:<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH after reboot. You can test this. Send the reboot() command, and while it's rebooting turn your motor, then make sure the motor returns back to the correct position each time when it comes out of reboot. Try this procedure a couple of times to be sure. 

### Hall Effect Encoders  
Hall effect encoders can also be used with ODrive. The encoder CPR should be set to `6 * <# of motor pole pairs>`. Due to the low resolution of hall effect encoders compared to other types of encoders, low speed performance will be worse than other encoder types.

When the encoder mode is set to hall feedback, the pinout on the encoder port is as follows:

| Label on ODrive | Hall feedback |
|-----------------|---------------|
| A               | Hall A        |
| B               | Hall B        |
| Z               | Hall C        |

To use hall effect encoders, the calibration sequence is different than incremental or absolute encoders. You must first run `AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION` before `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` The hall polarity calibration will automatically determine the order and polarity of the hall signals. When using `AXIS_STATE_FULL_CALIBRATION_SEQUENCE`, these steps are automatically used if the encoder is set to hall mode.

### Startup sequence notes
The following are variables that MUST be set up for your encoder configuration. Your values will vary depending on your encoder:

* `<axis>.encoder.config.cpr = 8192`
* `<axis>.encoder.config.mode = ENCODER_MODE_INCREMENTAL`

The following are examples of values that can impact the success of calibration. These are not all of the variables you have to set for startup. Only change these when you understand why they are needed; your values will vary depending on your setup:
* `<axis>.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT` The type of motor you have. Valid choices are high current or gimbal.
* `<axis>.encoder.config.calib_range = 0.05` Helps to relax the accuracy of encoder counts during calibration 
* `<axis>.motor.config.calibration_current = 10.0` The motor current used for calibration. For large motors, this value can be increased to overcome friction and cogging.
* `<axis>.motor.config.resistance_calib_max_voltage = 12.0` Max motor voltage used for measuring motor resistance. For motor calibration, it must be possible for the motor current to reach the calibration current without the applied voltage exceeding this config setting.
* `<axis>.controller.config.vel_limit = 5` [turn/s] low values result in the spinning motor stopping abruptly during calibration

Lots of other values can get you. It's a process. Thankfully there are a lot of good people that will help you debug calibration problems. 

If calibration works, congratulations.

Now try: 
* `<axis>.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`
* `<axis>.controller.input_vel = 1.5`
let it loop a few times and then set:
* `<axis>.requested_state = AXIS_STATE_IDLE`

Do you still have no errors? Awesome. Now, setup the motor and encoder to use known calibration values. This allows you to skip motor calibration and encoder offset calibration before using closed loop control. Note that this only works if you are using an absolute encoder or the encoder index input (see "Encoder with index signal" above).
* `<axis>.encoder.config.pre_calibrated = True`
* `<axis>.motor.config.pre_calibrated  = True `

And see if ODrive agrees that the calibration worked by just running
* `<axis>.encoder.config.pre_calibrated`

(using no "= True" ). Make sure that 'pre_calibrated' is in fact True. 

Also, if you have calibrated and encoder.pre_calibrated is equal to true, and you had no errors so far, run this: 
* `odrv0.save_configuration()`
* `odrv0.reboot()`

and now see if after a reboot you can run: 
* `<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH`

without getting errors. 

## What happens if calibration fails
There are subtle ways that encoder problems will impact your ODrive. For example, ODrive may not complete the calibrate sequence when you go to:
* `<axis>.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`

Or, ODrive may complete the calibrate sequence after:
* `<axis>.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`

but then it fails after you go to:
* `<axis>.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`

Or ODrive may just vibrate in an entertaining way. See:
https://www.youtube.com/watch?v=gaRUmwvSyAs

## Encoder Testing
There are things you can test to make sure your encoder is properly connected. `shadow_count` tracks encoder motion, even before the encoder or motor are calibrated. If your encoder is working, you should see this value change when you turn the motor.
Run the command:
* `<axis>.encoder.shadow_count `

and look at your value. Then turn your motor by hand and see if that value changes. Also, notice that the command:
* `<axis>.encoder.config.cpr = 4000`

must reflect the number of counts ODrive receives after one complete turn of the motor. So use shadow_count to test if that is working properly. 

You will probably never be able to properly debug if you have problems unless you use an oscilloscope. If you have one, try the following:
Connect to the AB pins, see if you get square waves as you turn the motor.
Connect to the I pin, see if you get a pulse on a complete rotation. Sometimes this is hard to see.

If you are using SPI, use a logic analyzer and connect to the CLK, MISO, and CS pins. Set a trigger for the CS pin and ensure that the encoder position is being sent and is increasing/decreasing as you spin the motor. There is extremely cheap hardware that is supported by [Sigrok](https://sigrok.org/) for protocol analysis. 


## Encoder Noise
Noise is found in all circuits, life is just about figuring out if it is preventing your system from working. Lots of users have no problems with noise interfering with their ODrive operation, others will tell you "_I've been using the same encoder as you with no problems_". Power to 'em, that may be true, but it doesn't mean it will work for you. If you are concerned about noise, there are several possible sources:

* Importantly, encoder wires may be too close to motor wires, avoid overlap as much as possible
* Long wires between encoder and ODrive
* Use of ribbon cable

The following _might_ mitigate noise problems. Use shielded cable, or use twisted pairs, where one side of each twisted pair is tied to ground and the other side is tied to your signal. If you are using SPI, use a 20-50 ohm resistor in series on CLK, which is more susceptible noise.

If you are using an encoder with an index signal, another problem that has been encountered is noise on the Z input of ODrive. Symptoms for this problem include:
* difficulty with requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE, where your calibration sequence may not complete
* strange behavior after performing odrv0.save_configuration() and odrv0.reboot()
* when performing an index_search, the motor does not return to the same position each time.
One easy step that _might_ fix the noise on the Z input is to solder a 22nF-47nF capacitor to the Z pin and the GND pin on the underside of the ODrive board. 

## Hall feedback pinout
If position accuracy is not a concern, you can use A/B/C hall effect encoders for position feedback.

To use this mode, configure the corresponding encoder mode: `<encoder>.config.mode = ENCODER_MODE_HALL`. Configure the corresponding GPIOs as digital inputs:

For encoder 0:

    <odrv>.config.gpio9_mode = GPIO_MODE_DIGITAL
    <odrv>.config.gpio10_mode = GPIO_MODE_DIGITAL
    <odrv>.config.gpio11_mode = GPIO_MODE_DIGITAL

For encoder 1:

    <odrv>.config.gpio12_mode = GPIO_MODE_DIGITAL
    <odrv>.config.gpio13_mode = GPIO_MODE_DIGITAL
    <odrv>.config.gpio14_mode = GPIO_MODE_DIGITAL

In this mode, the pinout on the encoder port is as follows:

| Label on ODrive | Hall feedback |
|-----------------|---------------|
| A               | Hall A        |
| B               | Hall B        |
| Z               | Hall C        |

## SPI Encoders

Apart from (incremental) quadrature encoders, ODrive also supports absolute SPI encoders (since firmware v0.5). These usually measure an absolute angle. This means you don't need to repeat the encoder calibration after every ODrive reboot. Currently, the following modes are supported:

 * **CUI protocol**: Compatible with the AMT23xx family (AMT232A, AMT232B, AMT233A, AMT233B).
 * **AMS protocol**: Compatible with AS5047P and AS5048A.

Some of these chips come with evaluation boards that can simplify mounting the chips to your motor. For our purposes if you are using an evaluation board you should select the settings for 3.3v.

**Note:** The AMT23x family has a hardware bug that causes them to not properly tristate the MISO line. To use them with ODrive, there are two workarounds. One is to sequence power to the encoder a second or two after the ODrive recieves power. This allows 1 encoder to be used without issue. Another solution is to add a tristate buffer, such as the 74AHC1G125SE, on the MISO line between the ODrive and each AMT23x encoder. Tie the enable pin on the buffer to the CS line for the respective encoder. This allows for more than one AMT23x encoder, or one AMT23x and another SPI encoder, to be used at the same time.

1. Connect the encoder to the ODrive's SPI interface:
   
    - The encoder's SCK, MISO (aka "DATA" on CUI encoders), MOSI (if present on the encoder), GND and 3.3V should connect to the ODrive pins with the same label. If you want to save a wire with AMS encoders, you can also connect the encoder's MOSI to the encoder's VDD instead.
    - The encoder's Chip Select (aka nCS/CSn) can be connected to any of the ODrive's GPIOs (caution: GPIOs 1 and 2 are usually used by UART).

If you are having calibration problems, make sure that your magnet is centered on the axis of rotation on the motor. Some users report that this has a significant impact on calibration. Also make sure that your magnet height is within range of the spec sheet. 

2. In `odrivetool`, run:

       <axis>.encoder.config.abs_spi_cs_gpio_pin = 4  # or which ever GPIO pin you choose
       <axis>.encoder.config.mode = ENCODER_MODE_SPI_ABS_CUI   # or ENCODER_MODE_SPI_ABS_AMS
       <axis>.encoder.config.cpr = 2**14              # or 2**12 for AMT232A and AMT233A
       <odrv>.save_configuration()
       <odrv>.reboot()

3. Run the [offset calibration](#encoder-without-index-signal) and then save the calibration with `<odrv>.save_configuration()`.
   The next time you reboot, the encoder should be immediately ready.

Sometimes the encoder takes longer than the ODrive to start, in which case you need to clear the errors after every restart.

If you are having calibration problems - make sure your magnet is centered on the axis of rotation on the motor, some users report this has a significant impact on calibration. Also make sure your magnet height is within range of the spec sheet.

