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
is_active_low | boolean | true

### gpio_num
The GPIO pin number, according to the silkscreen labels on ODrive. Set with these commands:
```
<odrv>.<axis>.mechanical_brake.config.gpio_num = <1, 2, 3, 4, 5, 6, 7, 8>
```
After GPIO pin number is changed, you'll need to run `<odrv>.save_configuration()` and `<odrv>.reboot()` for changes to take effect.

### is_active_low
Most safety braking systems are active low, e.g. when the power is off, the brake is on. If the system uses brake drive electronics which use active high logic, flip this bit then reconsider the safety implications of your design...

### Enabling
The configuration of the mechanical brake will enable the brake functionality. There's no need to specifically 'enable' this feature.


### Example

Let's say we're hacking away on an old ABB robotic arm. We've wired a 24V brake drive circuit triggered by GPIO5. When GPIO5 is driven, it will release the brakes on the axis we're moving.

We need to notify the axis of the GPIO number we've attached our brake to, and configure the ODrive pin mode to `GPIO_MODE_MECH_BRAKE`:
```
<odrv>.<axis>.mechanical_brake.config.gpio_num = 5
<odrv>.config.gpio5_mode = GPIO_MODE_MECH_BRAKE
```

Pin configurations only take effect after a save/reboot so don't forget to run:
```
<odrv>.save_configuration()
<odrv>.reboot()
```

### Testing The Mechanical Brakes
Depending on your system this could be a dangerous experiment. Ensure that you have taken all necessary precautions to confirm if the wrong brake were inadvertently released it would not lead to injury or damage to equipment.

```
<odrv>.<axis>.mechanical_brake.release()
```
Note: If a brake is configured, it will be automatically engaged/disengaged during the next state machine step.

After you're satisfied with the testing, you can re-enable the brake using the command

```
<odrv>.<axis>.mechanical_brake.engage()
```
