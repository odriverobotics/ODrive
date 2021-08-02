# Anti-cogging

ODrive supports an anti-cogging algorithm that attempts to compensate for the rather high cogging torques seen in hobby motors.

`<odrv>.<axis>.controller.config.anticogging` is a configuration structure that contains the following items:

Name | Type | Use
-- | -- | --
index | uint32 | The current position being used for calibration
pre_calibrated | bool | If true and using index or absolute encoder, load anticogging map from NVM at startup
calib_anticogging | bool | True when calibration is ongoing
calib_pos_threshold | float32 | (pos_estimate - index) must be < this value to calibrate.  Larger values speed up calibration but hurt accuracy
calib_vel_threshold | float32 | (vel_estimate) must be < this value to calibrate.  Larger values speed up calibration but hurt accuracy.
cogging_ratio | float32 | Deprecated
anticogging_enabled | bool | Enable or disable anticogging.  A valid anticogging map can be ignored by setting this to `false`

## Calibration

To calibrate anticogging, first make sure you can adequately control the motor. It should respond to position commands.

Start by putting the axis in `AXIS_STATE_CLOSED_LOOP_CONTROL` with `CONTROL_MODE_POSITION_CONTROL` and `INPUT_MODE_PASSTHROUGH`.  Make sure you have good control of the motor in this state (it responds to position commands).  Now, tune the motor to be very stiff - high `pos_gain` and relatively high `vel_integrator_gain`.  This will help in calibration.

Run `controller.start_anticogging_calibration()`.  The motor will start turning slowly, calibrating each point.  If you like, you can start a liveplotter session before running this command so that you can watch the position move.

Once it's complete (it should take about 1 minute), the motor will return to 0 and the value `controller.anticogging_valid` should report True.  If `controller.config.anticogging.anticogging_enabled` == True, anticogging will now be running on this axis.

## Saving to NVM

As of v0.5.1, the anticogging map is saved to NVM after calibrating and calling `odrv0.save_configuration()`

The anticogging map can be reloaded automatically at startup by setting `controller.config.anticogging.pre_calibrated = True` and saving the configuration.  However, this map is only valid and will only be loaded for absolute encoders, or encoders with index pins after the index search.

## Example

``` Py
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.start_anticogging_calibration()

# Wait until controller.config.anticogging.calib_anticogging == False

odrv0.axis0.controller.config.anticogging.pre_calibrated = True

odrv0.save_configuration()
odrv0.reboot()
```
