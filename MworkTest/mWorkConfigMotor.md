

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