# Mechanical Brake

Some systems employ mechanical brakes on motors as a safety feature. These brakes can also be engaged as a power-saving function if the motor is not moving, but still under load.

ODrive supports the use of its GPIO pins to connect to external brake drive electronics.

When the ODrive engages the drive electronics, the brake will be disabled. When the drive enters a fault or idle state, the brake will be re-engaged.

---

## Mechanical Brake Configuration
Each axis supports one mechanical brake. The following properties are accessible through `odrivetool`:

Name |  Type | Default
--- | -- | -- 
gpio_num | int | 0
enabled | boolean | false
is_active_low | boolean | true
pulldown | boolean | true

### gpio_num
The GPIO pin number, according to the silkscreen labels on ODrive. Set with these commands:
```
<odrv>.<axis>.mechanical_brake.config.gpio_num = <1, 2, 3, 4, 5, 6, 7, 8>
```

### enabled
Enables/disables the operation of the mechanical brake.
```
<odrv>.<axis>.mechanical_brake.config.enabled = <True, False>
```

### is_active_low
Most safety braking systems are active low, e.g. when the power is off, the brake is on. If the system uses brake drive electronics which use active high logic, flip this bit then reconsider the safety implications of your design...

### pulldown
If `true`, it enables the GPIO pulldown resistor.  If `false`, it enables the GPIO pullup resistor.

### Example

Let's say we're hacking away on an old ABB robotic arm. We've wired a 24V brake drive circuit triggered by GPIO5. When GPIO5 is driven, it will release the brakes on the axis we're moving.

```
<odrv>.<axis>.mechanical_brake.config.gpio_num = 5
<odrv>.<axis>.mechanical_brake.config.enabled = True
```

Don't forget to save and reboot:
```
<odrv>.save_configuration()
<odrv>.reboot()
```

### Testing The Mechanical Brakes
Depending on your system this could be a dangerous experiment. Ensure that you have taken all necessary precautions to confirm if the wrong brake were inadvertently released it would not lead to injury or damage to equipment.

```
<odrv>.<axis>.mechanical_brake.release()
```
Note: If a brake is enabled, it will be engaged/disengaged during the next state machine step. 

After you're satisfied with the testing, you can re-enable the brake using the command

```
<odrv>.<axis>.mechanical_brake.engage()
```
