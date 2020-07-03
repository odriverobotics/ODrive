odrv0.axis0.current_state

# calib motor 

odrv0.axis0.requested_stateư=AXIS_STATE_MOTOR_CALIBRATION 

odrv0.axis0.motor.config.pre_calibrated =True de luu thong so calib motor ( R và L)

Hoặc cài tay
odrv0.axis0.motor.config.phase_resistance 
odrv0.axis0.motor.config.phase_inductance

```
odrv0.save_configuration()
odrv0.reboot()
```

#SPI : encoder
1. Connect

```
The encoder's SCK, MISO (aka "DATA" on CUI encoders), GND and 3.3V should connect to the ODrive pins with the same label.
The encoder's MOSI should be tied to 3.3V (AMS encoders only. CUI encoders don't have this pin.)
The encoder's Chip Select (aka nCS/CSn) can be connected to any of the ODrive's GPIOs (caution: GPIOs 1 and 2 are usually used by UART).
```

2. Setting.

```
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4  # or which ever GPIO pin you choose
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS   # or ENCODER_MODE_SPI_ABS_AMS là x101 = 257
odrv0.axis0.encoder.config.cpr = 2**14              # or 2**12 for AMT232A and AMT233A
odrv0.axis0.save_configuration()
odrv0.axis0.reboot()
```

3. Setting Encoder.

```
<axis>.encoder.config.cpr = 8192
<axis>.encoder.config.mode = ENCODER_MODE_INCREMENTAL
```
```
<axis>.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT select if you have a gimbal or high amp motor
<axis>.encoder.config.calib_range = 0.05 helps to relax the accuracy of encoder counts during calibration
<axis>.motor.config.calibration_current = 10.0 sometimes needed if this is a large motor
<axis>.motor.config.resistance_calib_max_voltage = 12.0 sometimes needed depending on motor
<axis>.controller.config.vel_limit = 50000 low values result in the spinning motor stopping abruptly during calibration
```

4. Calib

- Chuyển mode calib ```odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION```
 .config.startup_encoder_index_search = True
 <axis>.encoder.config.pre_calibrated = True 

5. Save Calib

Test

```
<axis>.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
<axis>.controller.input_vel = 3000 let it loop a few times and then set:
<axis>.requested_state = AXIS_STATE_IDLE
```



odrv0.axis0.motor.config.current_lim = 30
odrv0.axis0.motor.config.calibration_current = 10
odrv0.axis0.encoder.config.pre_calibrated=False
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis0.encoder.config.pre_calibrated=true
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL 
odrv0.axis0.requested_state = AXIS_STATE_IDLE 
