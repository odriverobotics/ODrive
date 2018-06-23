roflcopter

```txt
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25
odrv0.axis0.motor.set_current_control_bandwidth(100)
odrv0.axis0.encoder.config.cpr = 90
odrv0.axis0.encoder.config.mode = 1
odrv0.axis0.encoder.config.bandwidth = 100
```

```txt
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02
```

```txt
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.motor
  error = 0x0000 (int)
  phase_inductance = 0.00033594953129068017 (float)
  phase_resistance = 0.1793474406003952 (float)
odrv0.axis0.motor.config.pre_calibrated = True
```

```txt
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
  error = 0x0000 (int)
  offset_float = 0.5126956701278687 (float)
    something close to 0.5 is good
odrv0.axis0.encoder.config.pre_calibrated = True
```


```txt
odrv0.config.enable_uart = False
odrv0.config.gpio1_pwm_mapping.min = -100
odrv0.config.gpio1_pwm_mapping.max = 100
odrv0.config.gpio1_pwm_mapping.endpoint = odrv0.axis0.controller._remote_attributes['vel_setpoint']
odrv0.save_configuration()
odrv0.reboot()
odrv0.axis0.controller.vel_setpoint
```

```txt
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()
```