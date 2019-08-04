
# ASCII Protocol

## How to send commands

 * **Via USB:**
    * **Windows:** Use the Zadig utility to set the ODrive's driver to "usbser". Windows will then make the device available as COM port. You can use [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/) to manually send commands or  open the COM port using your favorite programming language 
    * **Linux/macOS:** Run `/dev/tty*` to list all serial ports. The ODrive will show up as `/dev/ttyACM0` on Linux and `/dev/tty.usbmodem[...]` on macOS. Once you know the name, you can use `screen /dev/ttyACM0` (with the correct name) to send commands manually or open the device using your favorite programming language. Serial ports on Unix can be opened, written to and read from like a normal file.
 * **Via UART:** Connect the ODrive's TX (GPIO1) to your host's RX. Connect your ODrive's RX (GPIO2) to your host's TX. The logic level of the ODrive is 3.3V.
    * **Arduino:** You can use the [ODrive Arduino library](https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino) to talk to the ODrive.
    * **Windows/Linux/macOS:** You can use an FTDI USB-UART cable to connect to the ODrive.

## Command format

The ASCII protocol is human-readable and line-oriented, with each line having the following format:

```
command *42 ; comment [new line character]
```

 * `*42` stands for a GCode compatible checksum and can be omitted. If and only if a checksum is provided, the device will also include a checksum in the response, if any.
 * comments are supported for GCode compatibility
 * the command is interpreted once the new-line character is encountered

## Command Reference

#### Motor trajectory command
```
t motor destination
```
* `t` for trajectory
* `motor` is the motor number, `0` or `1`.
* `destination` is the goal position, in encoder counts.

Example: `t 0 -20000`

For general moving around of the axis, this is the recommended command.

This command updates the watchdog timer for the motor. 

#### Motor Position command
For basic use where you send one setpoint at at a time, use the `q` command.
If you have a realtime controller that is streaming setpoints and tracking a trajectory, use the `p` command.

```
q motor position velocity_lim current_lim
```
* `q` for position
* `motor` is the motor number, `0` or `1`.
* `position` is the desired position, in encoder counts.
* `velocity_lim` is the velocity limit, in counts/s (optional).
* `current_lim` is the current limit, in A (optional).

Example: `q 0 -20000 10000 10`

```
p motor position velocity_ff current_ff
```
* `p` for position
* `motor` is the motor number, `0` or `1`.
* `position` is the desired position, in encoder counts.
* `velocity_ff` is the velocity feed-forward term, in counts/s (optional).
* `current_ff` is the current feed-forward term, in A (optional).

Example: `p 0 -20000 0 0`

Note that if you don't know what feed-forward is or what it's used for, simply omit it.

This command updates the watchdog timer for the motor. 

#### Motor Velocity command
```
v motor velocity current_ff
```
* `v` for velocity
* `motor` is the motor number, `0` or `1`.
* `velocity` is the desired velocity in counts/s.
* `current_ff` is the current feed-forward term, in A (optional).

Example: `v 0 1000 0`

Note that if you don't know what feed-forward is or what it's used for, simply omit it.

This command updates the watchdog timer for the motor. 

#### Motor Current command
```
c motor current
```
* `c` for current
* `motor` is the motor number, `0` or `1`.
* `current` is the desired current in A.

This command updates the watchdog timer for the motor. 

#### Request feedback
```
f motor

response:
pos vel
```
* `f` for feedback
* `pos` is the encoder position in counts (float)
* `vel` is the encoder velocity in counts/s (float)

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
   * Example: `r vbus_voltage` => response: `24.087744` <new line>
 * Writing:
    ```
    w [property] [value]
    ```
   * `property` name of the property, as seen in ODrive Tool
   * `value` text representation of the value to be written
   * Example: `w axis0.controller.pos_setpoint -123.456`

#### System commands:
* `ss` - Save config
* `se` - Erase config
* `sr` - Reboot
