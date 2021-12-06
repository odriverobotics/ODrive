.. _ascii-protocol:

================================================================================
ASCII Protocol
================================================================================

.. contents::
   :depth: 1
   :local:

Sending Commands
-------------------------------------------------------------------------------

 * **Via USB:**
    * **Windows:** Use `PuTTY <https://www.chiark.greenend.org.uk/~sgtatham/putty/>`_ to manually send commands or open the COM port using your favorite programming language 
    * **Linux/macOS:** Run :code:`/dev/tty*` to list all serial ports. The ODrive will show up as :code:`/dev/ttyACM0` (or similar) on Linux and :code:`/dev/tty.usbmodem[...]` on macOS. 
      Once you know the name, you can use :code:`screen /dev/ttyACM0` (with the correct name) to send commands manually or open the device using your favorite programming language. 
      Serial ports on Unix can be opened, written to and read from like a normal file.
 * **Via UART:** Connect the ODrive's TX (GPIO1) to your host's RX. Connect your ODrive's RX (GPIO2) to your host's TX. See :ref:`UART <uart-doc>` for more info.
    * **Arduino:** You can use the `ODrive Arduino library <https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino>`_ to talk to the ODrive.
    * **Windows/Linux/macOS:** You can use an FTDI USB-UART cable to connect to the ODrive.

The ODrive does not echo commands. That means that when you type commands into a program like :code:`screen`, the characters you type won't show up in the console.

.. admonition:: Arduino

   There is an Arduino library that gives some examples on how to use the ASCII protocol to communicate with the ODrive. 
   Check it out `here <../Arduino/ODriveArduino>`.

Command Format
-------------------------------------------------------------------------------

The ASCII protocol is human-readable and line-oriented, with each line having the following format:

Format: :code:`command *42 ; comment [new line character]`

 * :code:`*42` stands for a GCode compatible checksum and can be omitted. If and only if a checksum is provided, the device will also include a checksum in the response, if any. 
   If the checksum is provided but is not valid, the line is ignored. The checksum is calculated as the bitwise xor of all characters before the asterisk (`*`). 
   Example of a valid checksum: :code:`r vbus_voltage *93`.
 * comments are supported for GCode compatibility
 * the command is interpreted once the new-line character is encountered


.. _acsii-cmd-ref:

Command Reference
-------------------------------------------------------------------------------

.. _motor_traj-cmd:

Motor Trajectory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Format: :code:`t motor destination`

* :code:`t` for trajectory.
* :code:`motor` is the motor number, :code:`0` or :code:`1`.
* :code:`destination` is the goal position, in [turns].

Example::
   
   t 0 -2

For general moving around of the axis, this is the recommended command.

This command updates the watchdog timer for the motor. 

Motor Position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For basic use where you send one setpoint at at a time, use the :code:`q` command.

Format: :code:`q motor position velocity_lim torque_lim`

* :code:`q` for position.
* :code:`motor` is the motor number, :code:`0` or :code:`1`.
* :code:`position` is the desired position, in [turns].
* :code:`velocity_lim` is the velocity limit, in [turns/s] (optional).
* :code:`torque_lim` is the torque limit, in [Nm] (optional).

Example:: 
   
   q 0 -2 1 0.1

If you have a realtime controller that is streaming setpoints and tracking a trajectory, use the :code:`p` command.

Format: :code:`p motor position velocity_ff torque_ff`

* :code:`p` for position
* :code:`motor` is the motor number, :code:`0` or :code:`1`.
* :code:`position` is the desired position, in [turns].
* :code:`velocity_ff` is the velocity feed-forward term, in [turns/s] (optional).
* :code:`torque_ff` is the torque feed-forward term, in [Nm] (optional).

Example::
   
   p 0 -2 0 0

This command updates the watchdog timer for the motor. 

.. note:: If you don't know what feed-forward is or what it's used for, simply omit it.
 
Motor Velocity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Format: :code:`v motor velocity torque_ff`

* :code:`v` for velocity
* :code:`motor` is the motor number, :code:`0` or :code:`1`.
* :code:`velocity` is the desired velocity in [turns/s].
* :code:`torque_ff` is the torque feed-forward term, in [Nm] (optional).

Example::
   
   v 0 1 0

This command updates the watchdog timer for the motor.


Motor Current
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Format: :code:`c motor torque`

* :code:`c` for torque
* :code:`motor` is the motor number, :code:`0` or :code:`1`.
* :code:`torque` is the desired torque in [Nm].

This command updates the watchdog timer for the motor. 


Request Feedback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

imput format: :code:`f motor`

response format: :code:`pos vel`

* :code:`f` for feedback.
* :code:`pos` is the encoder position in [turns] (float).
* :code:`vel` is the encoder velocity in [turns/s] (float).

Update Motor Watchdog
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Format: :code:`u motor`

* :code:`u` for /u/pdate.

* :code:`motor` is the motor number, :code:`0` or :code:`1`.

This command updates the watchdog timer for the motor, without changing any
setpoints. 

Parameter Reading/Writing
-------------------------------------------------------------------------------

Not all parameters can be accessed via the ASCII protocol but at least all parameters with float and integer type are supported.

 * Reading format: :code:`r [property]`
    
   * :code:`property` name of the property, as seen in ODrive Tool
   * response: text representation of the requested value
   * Example: :code:`r vbus_voltage` => response: :code:`24.087744`

 * Writing format: :code:`w [property] [value]`

   * :code:`property` name of the property, as seen in ODrive Tool
   * :code:`value` text representation of the value to be written
   * Example::

         w axis0.controller.input_pos -123.456

System Commands
-------------------------------------------------------------------------------

* :code:`ss` - Save config
* :code:`se` - Erase config
* :code:`sr` - Reboot
* :code:`sc` - Clear errors
