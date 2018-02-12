
Warning: this protocol has [been replaced](https://github.com/madcowswe/ODrive/blob/devel/Firmware/protocol.md).
It's still operational but for new applications it's recommended to use the new protocol.

### Command set
The most accurate way to understand the commands is to read [the code](MotorControl/commands.c) that parses the commands. Also you can have a look at the [ODrive Arduino library](https://github.com/madcowswe/ODriveArduino) that makes it easy to use the UART interface on Arduino. You can also look at it as an implementation example of how to talk to the ODrive over UART.

#### UART framing
USB communicates with packets, so it is easy to frame a command as one command per packet. However, UART doesn't have any packeting, so we need a way to frame the commands. The start-of-packet symbol is `$` and the end-of-packet symbol is `!`, that is, something like this: `$command!`. An example of a valid UART position command:
```
$p 0 10000 0 0!
```

#### Motor Position command
```
p motor position velocity_ff current_ff
```
* `p` for position
* `motor` is the motor number, `0` or `1`.
* `position` is the desired position, in encoder counts.
* `velocity_ff` is the velocity feed-forward term, in counts/s.
* `current_ff` is the current feed-forward term, in A.

Note that if you don't know what feed-forward is or what it's used for, simply set it to 0.

#### Motor Velocity command
```
v motor velocity current_ff
```
* `v` for velocity
* `motor` is the motor number, `0` or `1`.
* `velocity` is the desired velocity in counts/s.
* `current_ff` is the current feed-forward term, in A.

Note that if you don't know what feed-forward is or what it's used for, simply set it to 0.

#### Motor Current command
```
c motor current
```
* `c` for current
* `motor` is the motor number, `0` or `1`.
* `current` is the desired current in A.

#### Variable getting and setting
```
g type index
s type index value
```
* `g` for get, `s` for set
* `type` is the data type as follows:
** `0` is float
** `1` is int
** `2` is bool
* `index` is the index in the corresponding [exposed variable table](MotorControl/commands.c).

For example
* `g 0 12` will return the phase resistance of M0
* `s 0 8 10000.0` will set the velocity limit on M0 to 10000 counts/s
* `g 1 3` will return the error status of M0
* `g 1 7` will return the error status of M1

The error status corresponds to the [Error_t enum in low_level.h](MotorControl/low_level.h).

Note that the links in this section are to a specific commits to make sure that the line numbers are accurate. That is, they don't link to the newest master, but to an old version. Please check the corresponding lines in the code you are using. This is especially important to get the correct indicies in the exposed variable tables, and the error enum values.

#### Continous monitoring of variables
You can set up variables in monitoring slots, and then have them (or a subset of them) repeatedly printed upon request. Please see the code for this.
