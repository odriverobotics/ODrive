# Encoders

## Known and Supported Encoders
Check out the [ODrive Encoder Guide](https://docs.google.com/spreadsheets/d/1OBDwYrBb5zUPZLrhL98ezZbg94tUsZcdTuwiVNgVqpU).

## Encoder Calibration

All encoder types that are currently supported require the ODrive to do some sort of encoder calibration at every startup before you can run the motor control. Take this into account when designing your application.


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
