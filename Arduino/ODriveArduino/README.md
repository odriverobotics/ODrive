# ODriveArduino
Arduino library for the ODrive

To install the library, first clone this repository. In the Arduino IDE select: *Sketch -> Include Library -> Add .ZIP Library...*

Select the enclosing folder (e.g. ODriveArduino) to add it. Restarting the Arduino IDE may be necessary to see the examples in the *File* dropdown. Check the included example *ODriveArduinoTest* for basic usage. 

**Important Note: The Arduino library currently only supports the legacy UART protocol. This is selected in the firmware in [MotorControl/commands.h](https://github.com/madcowswe/ODrive/blob/master/Firmware/MotorControl/commands.h), with UART_PROTOCOL_LEGACY**

For most applications, this protocol is sufficient. Future versions of the Arduino library will be upgraded to support the current binary protocol.
