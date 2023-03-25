#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import signal

def signal_handler(signum, frame):
    exit(1)

def main():

    while (1):

        print("Waiting for odrive")
        # Find a connected ODrive (this will block until you connect one)
        odrv0 = odrive.find_any()

        # Wait for index search to be done
        state = odrv0.axis0.current_state
        while state != 1:
            time.sleep(1)
            state = odrv0.axis0.current_state

        time.sleep(1)

        # Initialize
        # odrv0.axis0.controller.config.input_filter_bandwidth = 100
        # odrv0.axis0.controller.config.vel_limit = 4.0
        # odrv0.axis0.trap_traj.config.vel_limit = 0.75
        # odrv0.axis0.motor.config.current_control_bandwidth = 800
        # odrv0.axis0.trap_traj.config.accel_limit = 3.5
        # odrv0.axis0.trap_traj.config.decel_limit = 3.5
        # odrv0.axis0.controller.config.enable_current_mode_vel_limit = True

        # current_position = odrv0.axis0.encoder.pos_estimate
        # print("current_position " + str(current_position))

        odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        # odrv0.axis0.controller.input_vel = 0
        # odrv0.axis0.controller.input_pos = current_position

        # odrv0.axis0.controller.input_pos = current_position
        odrv0.axis0.controller.input_vel = 0.75

        # Enable motor
        print("Enabling motor")
        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # Let motor spin for a few seconds
        time.sleep(3)

        speed = odrv0.axis0.encoder.vel_estimate
        # vel_setpoint = odrv0.axis0.controller.input_vel
        # pos_setpoint = odrv0.axis0.controller.input_pos
        # input_mode = odrv0.axis0.controller.config.input_mode
        # control_mode = odrv0.axis0.controller.config.control_mode

        # print("speed -- " + str(speed));
        # print("vel_setpoint -- " + str(vel_setpoint))
        # print("pos_setpoint -- " + str(pos_setpoint))
        # print("input_mode -- " + str(input_mode))
        # print("control_mode -- " + str(control_mode))

        # TODO:
        # - count the number of cycles before issue occurs
        # - check error state of motor and correlate to LOC that sets the error
        # - change to: input_mode --> control_mode --> setpoint --> enable_motor

        # Reproduced!! The speed read back was 0.0
        # Reproduced!! The speed read back was -58.75651550292969
        # Reproduced!! The speed read back was 0 and the ODrive went into failsafe pretty much immediately
        # Reproduced!! The speed read back was 0.0 and the setpoint was 0.75

        if (speed > 1.0 or speed < 0.25):
            print("ERROR!")

            time.sleep(3)

            speed = odrv0.axis0.encoder.vel_estimate
            vel_setpoint = odrv0.axis0.controller.input_vel
            pos_setpoint = odrv0.axis0.controller.input_pos
            input_mode = odrv0.axis0.controller.config.input_mode
            control_mode = odrv0.axis0.controller.config.control_mode

            # TODO: also check  Iq_setpoint, torque_setpoint
            print("speed after error -- " + str(speed));
            print("vel_setpoint after error -- " + str(vel_setpoint))
            print("pos_setpoint after error -- " + str(pos_setpoint))
            print("input_mode after error -- " + str(input_mode))
            print("control_mode after error -- " + str(control_mode))

            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            exit(1)

        odrv0.axis0.requested_state = AXIS_STATE_IDLE

        try:
            odrv0.reboot()
        except Exception as e:
            print("Waiting for odrive to reboot")
            time.sleep(6)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    main()