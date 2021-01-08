import odrive
from odrive.enums import *
from odrive.utils import *

print("finding an odrive...")
odrv0 = odrive.find_any()
print('Odrive found')

odrv0.axis1.controller.config.vel_limit = 50000
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis1.encoder.config.cpr = 2400
odrv0.axis1.encoder.config.bandwidth = 1000
odrv0.axis1.motor.config.calibration_current = 5
odrv0.axis1.motor.config.current_lim = 5
odrv0.axis1.controller.config.homing_speed = 5000
odrv0.config.brake_resistance = 0

odrv0.axis0.min_endstop.config.gpio_num = 6
odrv0.axis0.min_endstop.config.enabled = True
odrv0.axis0.min_endstop.config.offset = -1000
odrv0.axis0.max_endstop.config.gpio_num = 5
odrv0.axis0.max_endstop.config.enabled = True

odrv0.axis1.min_endstop.config.gpio_num = 8
odrv0.axis1.min_endstop.config.enabled = True
odrv0.axis1.min_endstop.config.offset = -1000
odrv0.axis1.max_endstop.config.gpio_num = 7
odrv0.axis1.max_endstop.config.enabled = True

odrv0.axis1.config.startup_encoder_offset_calibration = True
odrv0.axis1.config.startup_motor_calibration = True
odrv0.axis1.config.startup_homing = True
odrv0.axis1.config.startup_closed_loop_control = True
