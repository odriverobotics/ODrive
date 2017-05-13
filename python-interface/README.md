#### Usage:

1) Generate python api from low_level.c

<code>python generate_api.py</code>

2) Start ODrive.py

<code>python ODrive.py /dev/yourserial</code>

3) Open interface.html to access ODrive webinterface.


4) To change the default 127.0.0.1:12342 websocket, use "ODrive.py /dev/yourserial 1.1.2.3:9999"

##### iPython shell and API: 
1) Trigger selftest, selftest is required to enable motor control.

<code>odrive.motors[0].set_do_selftest(True)</code>


2) Wait for selftest to finish

<code>while(odrive.motors[0].get_do_selftest()):
    time.sleep(1)
</code>

3) Check if selftest was successful

<code>odrive.motors[0].get_selftest_ok()</code>

4) In case of error:

<code>odrive.motors[0].get_error()</code>

5) Set control mode to POSITION, VELOCITY or CURRENT

<code>odrive.motors[0].set_control_mode(ControlMode.POSITION)</code>
 
6) Enable Motor control, selftest must be successful to do this

<code>odrive.motors[0].set_enable_control(True)</code>

7) Move to some position with POSITION control active

<code>odrive.motors[0].set_pos_setpoint(0)</code>

8) Achieve some velocity with VELOCITY control active

<code>odrive.motors[0].set_vel_setpoint(0)</code>

9) Use some current with CURRENT control active

<code>odrive.motors[0].set_current_setpoint(0)</code>


