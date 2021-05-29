
import odrive
from odrive.utils import dump_errors
from odrive.enums import *
import time

print("Finding an odrive...")
odrv = odrive.find_any()

# axes = [odrv.axis0, odrv.axis1];
axes = [odrv.axis0];

flip_index_search_direction = False
save_and_reboot = True

print("Setting config...")
# Settings to protect battery
odrv.config.dc_bus_overvoltage_trip_level = 14.8
odrv.config.dc_bus_undervoltage_trip_level = 8.0
odrv.config.brake_resistance = 0
for ax in axes:
    ax.motor.config.requested_current_range = 25
    ax.motor.config.calibration_current = 10
    ax.motor.config.current_lim = 10
    ax.motor.config.resistance_calib_max_voltage = 4
    ax.motor.config.pole_pairs = 10

    ax.encoder.config.cpr = 4096
    ax.encoder.config.use_index = True
    ax.encoder.config.find_idx_on_lockin_only = True

    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.vel_limit = 10000
    ax.controller.config.vel_gain = 0.002205736003816127
    ax.controller.config.vel_integrator_gain = 0.022057360038161278
    ax.controller.config.pos_gain = 26

    ax.config.lockin.current = 10
    ax.config.lockin.ramp_distance = 3.14
    ax.config.lockin.vel = 15
    ax.config.lockin.accel = 10
    ax.config.lockin.finish_distance = 30

def wait_and_exit_on_error(ax):
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    if ax.error != AXIS_ERROR_NONE:
        dump_errors(odrv, True)
        exit()

for axnum, ax in enumerate(axes):
    print("Calibrating motor {}...".format(axnum))
    ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    wait_and_exit_on_error(ax)

    print("Checking motor {} direction...".format(axnum))
    ax.requested_state = AXIS_STATE_ENCODER_DIR_FIND
    wait_and_exit_on_error(ax)
    print("    Direction is {}".format(ax.motor.config.direction))

    if flip_index_search_direction:
        ax.config.lockin.ramp_distance = -ax.config.lockin.ramp_distance
        ax.config.lockin.vel = -ax.config.lockin.vel
        ax.config.lockin.accel = -ax.config.lockin.accel

    print("Searching for index on motor {}...".format(axnum))
    ax.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    wait_and_exit_on_error(ax)
    if (not ax.encoder.index_found):
        print("Failed finding index! Quitting.")
        exit()

    print("Calibrating encoder offset on motor {}...".format(axnum))
    ax.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    wait_and_exit_on_error(ax)
    if (not ax.encoder.is_ready):
        print("Failed to calibrate encoder! Quitting")
        exit()

    # If we get here there were no errors, so let's commit the values
    ax.motor.config.pre_calibrated = True
    ax.encoder.config.pre_calibrated = True

    # Uncomment this if you wish to automatically run index search and closed loop control on boot
    # ax.config.startup_encoder_index_search = True
    # ax.config.startup_closed_loop_control = True

#Everything should be good to go here, so let's save and reboot
print("")
print("All operations successful!")
if save_and_reboot:
    odrv.save_configuration()
    try:
        odrv.reboot()
    except odrive.fibre.ObjectLostError:
        pass
