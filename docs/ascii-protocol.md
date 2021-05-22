
# ASCII Protocol

## How to send commands

 * **Via USB:**
    * **Windows:** Use [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/) to manually send commands or open the COM port using your favorite programming language 
    * **Linux/macOS:** Run `/dev/tty*` to list all serial ports. The ODrive will show up as `/dev/ttyACM0` (or similar) on Linux and `/dev/tty.usbmodem[...]` on macOS. Once you know the name, you can use `screen /dev/ttyACM0` (with the correct name) to send commands manually or open the device using your favorite programming language. Serial ports on Unix can be opened, written to and read from like a normal file.
 * **Via UART:** Connect the ODrive's TX (GPIO1) to your host's RX. Connect your ODrive's RX (GPIO2) to your host's TX. See [UART](uart) for more info.
    * **Arduino:** You can use the [ODrive Arduino library](https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino) to talk to the ODrive.
    * **Windows/Linux/macOS:** You can use an FTDI USB-UART cable to connect to the ODrive.

The ODrive does not echo commands. That means that when you type commands into a program like `screen`, the characters you type won't show up in the console.

### Arduino
There is an Arduino library that gives some examples on how to use the ASCII protocol to communicate with the ODrive. Check it out [here](../Arduino/ODriveArduino).

## Command format

The ASCII protocol is human-readable and line-oriented, with each line having the following format:

```
command *42 ; comment [new line character]
```

 * `*42` stands for a GCode compatible checksum and can be omitted. If and only if a checksum is provided, the device will also include a checksum in the response, if any. If the checksum is provided but is not valid, the line is ignored. The checksum is calculated as the bitwise xor of all characters before the asterisk (`*`). <br> Example of a valid checksum: `r vbus_voltage *93`.
 * comments are supported for GCode compatibility
 * the command is interpreted once the new-line character is encountered

## Command Reference

#### Motor trajectory command
```
t motor destination
```
* `t` for trajectory
* `motor` is the motor number, `0` or `1`.
* `destination` is the goal position, in [turns].

Example: `t 0 -2`

For general moving around of the axis, this is the recommended command.

This command updates the watchdog timer for the motor. 

#### Motor Position command
For basic use where you send one setpoint at at a time, use the `q` command.
If you have a realtime controller that is streaming setpoints and tracking a trajectory, use the `p` command.

```
q motor position velocity_lim torque_lim
```
* `q` for position
* `motor` is the motor number, `0` or `1`.
* `position` is the desired position, in [turns].
* `velocity_lim` is the velocity limit, in [turns/s] (optional).
* `torque_lim` is the torque limit, in [Nm] (optional).

Example: `q 0 -2 1 0.1`

```
p motor position velocity_ff torque_ff
```
* `p` for position
* `motor` is the motor number, `0` or `1`.
* `position` is the desired position, in [turns].
* `velocity_ff` is the velocity feed-forward term, in [turns/s] (optional).
* `torque_ff` is the torque feed-forward term, in [Nm] (optional).

Example: `p 0 -2 0 0`

Note that if you don't know what feed-forward is or what it's used for, simply omit it.

This command updates the watchdog timer for the motor. 

#### Motor Velocity command
```
v motor velocity torque_ff
```
* `v` for velocity
* `motor` is the motor number, `0` or `1`.
* `velocity` is the desired velocity in [turns/s].
* `torque_ff` is the torque feed-forward term, in [Nm] (optional).

Example: `v 0 1 0`

Note that if you don't know what feed-forward is or what it's used for, simply omit it.

This command updates the watchdog timer for the motor. 

#### Motor Current command
```
c motor torque
```
* `c` for torque
* `motor` is the motor number, `0` or `1`.
* `torque` is the desired torque in [Nm].

This command updates the watchdog timer for the motor. 

#### Request feedback
```
f motor

response:
pos vel
```
* `f` for feedback
* `pos` is the encoder position in [turns] (float)
* `vel` is the encoder velocity in [turns/s] (float)

#### Update motor watchdog
```
u motor
```
* `u` for /u/pdate.
* `motor` is the motor number, `0` or `1`.

This command updates the watchdog timer for the motor, without changing any
setpoints. 

#### Parameter reading/writing

Not all parameters can be accessed via the ASCII protocol but at least all parameters with float and integer type are supported.

 * Reading:
    ```
    r [property]
    ```
   * `property` name of the property, as seen in ODrive Tool
   * response: text representation of the requested value
   * Example: `r vbus_voltage` => response: `24.087744` &lt;new line&gt;
 * Writing:
    ```
    w [property] [value]
    ```
   * `property` name of the property, as seen in ODrive Tool
   * `value` text representation of the value to be written
   * Example: `w axis0.controller.input_pos -123.456`

#### System commands:
* `ss` - Save config
* `se` - Erase config
* `sr` - Reboot
* `sc` - Clear errors
