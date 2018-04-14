
# ASCII Protocol

## How to send commands

 * **Via USB:**
    * **Windows:** Use the Zadig utility to set the ODrive's driver to "usbser". Windows will then make the device available as COM port. You can use [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/) to manually send commands or  open the COM port using your favorite programming language 
    * **Linux/macOS:** Run `/dev/tty*` to list all serial ports. The ODrive will show up as `/dev/ttyACM0` on Linux and `/dev/tty.usbmodem[...]` on macOS. Once you know the name, you can use `screen /dev/ttyACM0` (with the correct name) to send commands manually or open the device using your favorite programming language. Serial ports on Unix can be opened, written to and read from like a normal file.
 * **Via UART:** Connect the ODrive's TX (GPIO1) to your host's RX. Connect your ODrive's RX (GPIO2) to your host's TX. The logic level of the ODrive is 3.3V.
    * **Arduino:** You can use the [ODrive Arduino library](https://github.com/madcowswe/ODriveArduino) to talk to the ODrive.
    * **Windows/Linux/macOS:** You can use an FTDI USB-UART cable to connect to the ODrive.


## Command Reference

#### Motor Position command
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

#### Motor Current command
```
c motor current
```
* `c` for current
* `motor` is the motor number, `0` or `1`.
* `current` is the desired current in A.

#### Parameter reading/writing

This is currently not supported. Use the native protocol.
