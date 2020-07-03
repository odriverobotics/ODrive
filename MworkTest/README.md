odrv0.axis0.current_state


1. Dòng Điện ```odrv0.axis0.motor.config.current_lim = 10``` limit 10A
2. Vận Tốc ```odrv0.axis0.controller.config.vel_limit = 20000``` la 20000[counts/s].
3. Calibration current
4. Điện trở xả ```odrv0.config.brake_resistance = 0.5``` Điện trở xả 
5. Poles ```odrv0.axis0.motor.config.pole_pairs = 24 ``` Poles pair
6. Motor Type ```odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT```

- MOTOR_TYPE_HIGH_CURRENT
- MOTOR_TYPE_GIMBAL

7. Encoder ```odrv0.axis0.encoder.config.cpr =  ```  Encoder Count Per Revolution [CPR]
8. Save Config ```odrv0.save_configuration() ```
9. Reboot ```odrv0.reboot()```




Kv (rpm/V)

odrv0.axis0.controller.config.control_mode = 1
```
CTRL_MODE_POSITION_CONTROL  1
CTRL_MODE_VELOCITY_CONTROL  2
CTRL_MODE_CURRENT_CONTROL   3
CTRL_MODE_VOLTAGE_CONTROL 
```

#define R_PHASE 0.13f           //Ohms
#define L_D 0.00002f            //Henries
#define L_Q 0.00002f            //Henries
#define KT .08f                 //N-m per peak phase amp, = WB*NPP*3/2
#define NPP 21                  //Number of pole pairs



# calib motor 
```
odrv0.axis0.requested_stateư=AXIS_STATE_MOTOR_CALIBRATION 

odrv0.axis0.motor.config.pre_calibrated =True de luu thong so calib motor ( R và L)

Hoặc cài tay
odrv0.axis0.motor.config.phase_resistance 
odrv0.axis0.motor.config.phase_inductance
```
```
odrv0.save_configuration()
odrv0.reboot()
```

# SPI : encoder

1. Connect
-----------

```
The encoder's SCK, MISO (aka "DATA" on CUI encoders), GND and 3.3V should connect to the ODrive pins with the same label.
The encoder's MOSI should be tied to 3.3V (AMS encoders only. CUI encoders don't have this pin.)
The encoder's Chip Select (aka nCS/CSn) can be connected to any of the ODrive's GPIOs (caution: GPIOs 1 and 2 are usually used by UART).
```

2. Setting.
---------

```
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 6  # or which ever GPIO pin you choose
odrv0.axis0.encoder.config.mode = 257
 # or ENCODER_MODE_SPI_ABS_AMS là x101 = 257
odrv0.axis0.encoder.config.cpr = 2**14              # or 2**12 for AMT232A and AMT233A
odrv0.save_configuration()
odrv0.reboot()
```

3. Setting Encoder.
-----------


```
 odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT select if you have a gimbal or high amp motor
 odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_GIMBAL là 2
 odrv0.axis0.encoder.config.calib_range = 0.05 helps to relax the accuracy of encoder counts during calibration
 odrv0.axis0.motor.config.calibration_current = 10.0 sometimes needed if this is a large motor
 odrv0.axis0.motor.config.resistance_calib_max_voltage = 12.0 sometimes needed depending on motor
 odrv0.axis0.controller.config.vel_limit = 500000 low values result in the spinning motor stopping abruptly during calibration
```

4. Calib

 odrv0.axis0.encoder.config.pre_calibrated = False
- Chuyển mode calib ```odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION```

 odrv0.axis0.encoder.config.pre_calibrated = True 

5. Save Calib

Test

```
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 3000 let it loop a few times and then set:
odrv0.axis0.requested_state = AXIS_STATE_IDLE
```



odrv0.axis0.motor.config.current_lim = 3
odrv0.axis0.motor.config.calibration_current = 2
odrv0.axis0.encoder.config.pre_calibrated=False
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis0.encoder.config.pre_calibrated=true
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL 
odrv0.axis0.requested_state = AXIS_STATE_IDLE 

# PID

```
odrv0.axis0.controller.config.pos_gain = 2000.0 
odrv0.axis0.controller.config.vel_gain = 0.000001
odrv0.axis0.controller.config.vel_integrator_gain = 
```
odrv0.axis0.controller.input_pos = 10000