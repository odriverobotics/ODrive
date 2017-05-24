# ODrive
This project is all about accuratly driving brushless motors, for cheap. The aim is to make it possible to use inexpensive brushless motors in high performance robotics projects.
Like this:
[![Servo motor control demo](https://j.gifs.com/lYx7k6.gif)](https://www.youtube.com/watch?v=WT4E5nb3KtY)

This repository contains configuration and analysis scripts that runs on a PC. The other related repositories are:
* [ODriveFirmware](https://github.com/madcowswe/ODriveFirmware): Firmware that runs on the board.
* [ODriveHardware](https://github.com/madcowswe/ODriveHardware): Circuit board design.

There is also [ODriveFPGA](https://github.com/madcowswe/ODriveFPGA), which contains the FPGA logic and software that runs on the FPGA based ODrive. This is not currently in development, but may be resumed at some later date.

## Getting Started
*References to hardware is with respect to v3.2. Other versions may still apply, but component designators may differ*

It is perfectly fine, and even recommended, to start testing with just a single motor and encoder.
Make sure you have a good mechanical connection between the encdoer and the motor, slip can cause disasterous oscillations.
All non-power I/O is 3.3v output and 5v tolerant on input.

You need one or two brushless motor(s), qudrature incremental encoder(s), and likely a power resistor.
If you are powering from a battery that can absorb the energy from decellerating the load, and you wish to do regenerative breaking, then you do not need it. If your load is just the motor rotor, and you are testing at low speeds, it may not be required. If you are powering from a power supply, and have a load attached or are running at significant speeds/accelerations, you NEED to connect a power resistor.
The power resistor values you need depends on your motor setup, and peak/average decelleration power. A good starting point would be a [0.47 ohm, 50W resistor](https://www.digikey.com/product-detail/en/te-connectivity-passive-product/HSA50R47J/A102181-ND/2056131).

Wire up the motor phases into the 3-phase screw terminal(s), and the power resistor to the AUX terminal. Wire up the power source to the DC terminal, make sure to pay attention to the polarity. Do not apply power just yet.

Wire up the encoder(s) to J4. The A,B phases are required, and the Z (index pulse) is optional. The A,B and Z lines have 1k pull up resistors, for use with open-drain encoder outputs. For single ended push-pull signals with weak drive current (\<2mA), you may want to desolder the pull-ups.
