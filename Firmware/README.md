# ODriveFirmware

If you wish to use the latest release, please use the `master` branch (this is the default branch GitHub will present you with).

If you are a developer, you are encouraged to use the `devel` branch, as it contains the latest features.

The project is under active development, so make sure to check the [Changelog](CHANGELOG.md) to keep track of updates.

### Table of contents

<!-- MarkdownTOC depth=2 autolink=true bracket=round -->

- [Configuring parameters](#configuring-parameters)
- [Compiling and downloading firmware](#compiling-and-downloading-firmware)
- [Setting up an IDE](#setting-up-an-ide)
- [Continuing without an IDE](#no-ide-instructions)
- [Communicating over USB or UART](#communicating-over-usb-or-uart)
- [Generating startup code](#generating-startup-code)
- [Notes for Contributors](#notes-for-contributors)

<!-- /MarkdownTOC -->

## Configuring parameters
To correctly operate the ODrive, you need to supply some parameters. Some are mandatory, and if supplied incorrectly will cause the drive to malfunction. To get good performance you must also tune the drive.

The first thing to set is your board hardware version, located at the top of [Inc/main.h](Inc/main.h). If, for example, you are using the hardware: ODrive v3.2, then you should set it like this:
```C
#define HW_VERSION_MAJOR 3
#define HW_VERSION_MINOR 2
```
If you are using the 48V version of ODrive, you should also uncomment this line
```C
#define HW_VERSION_HIGH_VOLTAGE true
```

### Communication configuration
If want to use the example python scripts and connect the ODrive via USB, the defaults are fine for you and you can skip this step.

You can select what interface you want to run on USB and GPIO pins. See [Communicating over USB or UART](#communicating-over-usb-or-uart) for more information.
The following options are available in [MotorControl/commands.h](MotorControl/commands.h):

__USB__:
 - `USB_PROTOCOL_NATIVE`: Use the native protocol (recommended for new applications).
    The python library only understands the native protocol, so this is the way to go
    if you use that.
 - `USB_PROTOCOL_NATIVE_STREAM_BASED`: Use the native stream based protocol.
    On most platforms the device shows up as a serial port when connected over USB.
    So instead of using the python tool's direct USB access, you can use this option and then pretend you connected the device over serial.
    __On some platforms (specifically macOS), this is required__ because the kernel doesn't allow direct USB access.
 - `USB_PROTOCOL_LEGACY`: Use the human-readable legacy protocol
    Select this option if you already have an existing application. This option will be removed in the future.
 - `USB_PROTOCOL_NONE`: Ignore USB communication

__GPIO 1,2 pins__:
Note that UART is only supported on ODrive v3.3 and higher.
 - `UART_PROTOCOL_NATIVE`: Use the native protocol (see notes above).
 - `UART_PROTOCOL_LEGACY`: Use the human-readable legacy protocol
    Use this option if you control the ODrive with an Arduino. The ODrive Arduino library is not yet updated to the native protocol.
 - `UART_PROTOCOL_NONE`: Ignore UART communication
 - `USE_GPIO_MODE_STEP_DIR`: Step/direction control mode (use in conjunction with `UART_PROTOCOL_NONE`)

### Motor control parameters
The rest of all the parameters are at the top of the [MotorControl/low_level.c](MotorControl/low_level.c) file. Please note that many parameters occur twice, once for each motor.
In it's current state, the motor structs contain both tuning parameters, meant to be set by the developer, and static variables, meant to be modified by the software. Unfortunatly these are mixed together right now, but cleaning this up is a high priority task.

It may be helpful to know that the entry point of each of the motor threads is `void axis_thread_entry` at the top of [MotorControl/axis.cpp](MotorControl/axis.cpp). This is like `main` for each motor, and is probably where you should start reading the code.

### Mandatory parameters
You must set:
* `ENCODER_CPR`: Encoder Count Per Revolution (CPR). This is 4x the Pulse Per Revolution (PPR) value.
* `POLE_PAIRS`: This is the number of magnet poles in the rotor, divided by two. You can simply count the number of magnets in the rotor, if you can see them.
* `brake_resistance`: This is the resistance of the brake resistor. If you are not using it, you may set it to 0.0f.
* `motor_type`: This is the type of motor being used. Currently two types of motors are supported -- High-current motors (`MOTOR_TYPE_HIGH_CURRENT`) and Gimbal motors (`MOTOR_TYPE_GIMBAL`).

### Motor Modes
The firwmare currently supports two different types of motors, high-current motors, and Gimbal motors. If you're using a regular hobby brushless motor like [this](https://hobbyking.com/en_us/turnigy-aerodrive-sk3-5065-236kv-brushless-outrunner-motor.html) one, you should set `motor_mode` to `MOTOR_TYPE_HIGH_CURRENT`. For high-torque gimbal motors like [this](https://hobbyking.com/en_us/turnigy-hd-5208-brushless-gimbal-motor-bldc.html) one, you should choose `MOTOR_TYPE_GIMBAL`.

**Further detail:**

If 100's of mA of current noise is "small" for you, you can choose `MOTOR_TYPE_HIGH_CURRENT`.
If 100's of mA of current noise is "large" for you, and you do not intend to spin the motor very fast (omega * L << R), and the motor is fairly large resistance (1 ohm or larger), you can chose `MOTOR_TYPE_GIMBAL`.

If 100's of mA current noise is "large" for you, and you intend to spin the motor fast, then you need to replace the shunt resistors on the ODrive.

### Tuning parameters
The most important parameters are the limits:
* The current limit: `.current_lim = 75.0f, //[A] // Note: consistent with 40v/v gain`. The default current limit, for safety reasons, is set to 10A. This is quite weak, and good for making sure the drive is stable. Once you have tuned the drive, you can increase this to 75A to get some performance. Note that above 75A, you must change the current amplifier gains.
  * Note: The motor current and the current drawn from the power supply is not the same in general. You should not look at the power supply current to see what is going on with the motor current.
* The velocity limit: `.vel_limit = 20000.0f, // [counts/s]`. The motor will be limited to this speed; again the default value is quite slow.

The motion control gains are currently manually tuned:
* `.pos_gain = 20.0f, // [(counts/s) / counts]`
* `.vel_gain = 15.0f / 10000.0f, // [A/(counts/s)]`
* `.vel_integrator_gain = 10.0f / 10000.0f, // [A/(counts/s * s)]`

An upcoming feature will enable automatic tuning. Until then, here is a rough tuning procedure:
* Set the integrator gain to 0
* Make sure you have a stable system. If it is not, decrease all gains until you have one.
* Increase `vel_gain` by around 30% per iteration until the motor exhibits some vibration.
* Back down `vel_gain` to 50% of the vibrating value.
* Increase `pos_gain` by around 30% per iteration until you see some overshoot.
* Back down `pos_gain` until you do not have overshoot anymore.
* The integrator is not easily tuned, nor is it strictly required. Tune at your own discression.

### Optional parameters
By default both motors are enabled, and the default control mode is position control.
If you want a different mode, you can change `.control_mode`. To disable a motor, set `.enable_control` and `.do_calibration` to false.

<br><br>
## Compiling and downloading firmware
### Getting a programmer
Get a programmer that supports SWD (Serial Wire Debugging) and is ST-link v2 compatible. You can get them really cheap on [eBay](http://www.ebay.co.uk/itm/ST-Link-V2-Emulator-Downloader-Programming-Mini-Unit-STM8-STM32-with-20CM-Line-/391173940927?hash=item5b13c8a6bf:g:3g8AAOSw~OdVf-Tu) or many other places.

### Installing prerequisites
To compile the program, you first need to install the prerequisite tools:

#### Linux:
* `gcc-arm-none-eabi`: GCC compilation toolchain for ARM microcontrollers.
    * Installing on Ubuntu: `sudo apt-get install gcc-arm-none-eabi`
* `gdb-arm-none-eabi`: GNU project debugger for ARM microcontrollers.
    * Installing on Ubuntu: `sudo apt-get install gdb-arm-none-eabi`
* `OpenOCD`: Open On-Chip Debugging tools. This is what we use to flash the code onto the microcontroller.
    * Installing on Ubuntu: `sudo apt-get install openocd`
* No additional USB CDC driver should be required on Linux.

#### Mac:
* `brew cask install gcc-arm-embedded`:  GCC toolchain+debugger
* `brew install openocd`: Programmer

#### Windows:
Install the following:
* [Git for windows](https://git-scm.com/download/win). This intalls the Git Bash, which is a unix style command line interface that we will be using. 
* [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). The cross-compiler used to compile the code. Download and install the "Windows 32-bit" version. Make sure to tick the "add to path" option.
* [Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm). Make is used to script the compilation process. Download and run the complete package setup program. Add the path of the binaries to your PATH environment variable. For me this was at `C:\Program Files (x86)\GnuWin32\bin`. For details on how to set your path envirment in windows see [these instructions.](https://www.java.com/en/download/help/path.xml)
* OpenOCD. Follow the instructions at [GNU ARM Eclipse  - How to install the OpenOCD binaries](http://gnuarmeclipse.github.io/openocd/install/), including the part about ST-LINK/V2 drivers. Add the path of the binaries to your PATH environment variable. For me this was at `C:\Program Files\GNU ARM Eclipse\OpenOCD\0.10.0-201704182147-dev\bin`.

<br><br>
## Setting up an IDE
ODrive is a Makefile project.  It does not require an IDE, but the open-source IDE VSCode is recommended.  It is also possible to use Eclipse.  If you'd like to go that route, please see the respective configuration document:

* [Configuring VSCode](configuring-vscode.md)
* [Configuring Eclipse](configuring-eclipse.md)

<br><br>
## No IDE Instructions
After installing all of the above, open a Git Bash shell. Continue at section [Building the firmware](#building-the-firmware).

### Building the firmware
* Make sure you have cloned the repository.
* Navigate your terminal (bash/cygwin) to the ODrive/Firmware dir.
* Run `make` in the root of this repository.

### Flashing the firmware
* **Make sure you have [configured the parameters first](#configuring-parameters)**
* Connect `GND`, `SWD`, and `SWC` on connector J2 to the programmer. Note: Always plug in `GND` first!
* You need to power the board by only **ONE** of the following: VCC(3.3v), 5V, or the main power connection (the DC bus). The USB port (J1) does not power the board.
* Run `make flash` in the root of this repository.

If the flashing worked, you can start sending commands. If you want to do that now, you can go to [Communicating over USB or UART](#communicating-over-usb-or-uart).

### Debugging the firmware
* Run `make gdb`. This will reset and halt at program start. Now you can set breakpoints and run the program. If you know how to use gdb, you are good to go.

<br><br>
## Communicating over USB or UART
Warning: If testing USB or UART communication for the first time it is recommend that your motors are free to spin continuously and are not connected to a drivetrain with limited travel.
### From Linux/Windows/macOS
There are two example python scripts to help you get started with controlling the ODrive using python. One will drop you into an interactive shell to query settings, parameters, and variables, and let you send setpoints manually ([tools/explore_odrive.py](tools/explore_odrive.py)). The other is a demo application to show you how to control the ODrive programmatically ([tools/demo.py](tools/demo.py)). Below follows a step-by-step guide on how to run these.


* __Windows__: It is recommended to use a Unix style command prompt, such as Git Bash that comes with [Git for windows](https://git-scm.com/download/win).

1. [Install Python 3](https://www.python.org/downloads/), then install dependencies pyusb and pyserial:
```
pip install pyusb pyserial
```
* Note: If you have python2 and python3 installed concurrently then you must specifiy that we wish to target python3. This is done as follows:
  * __Linux__: Use `pip3` instead of `pip` in the above command.
  * __Windows__: Use the full path of the Python3 pip, yeilding something like:
 `C:\Users\YOUR_USERNAME\AppData\Local\Programs\Python\Python36-32\Scripts\pip install pyusb pyserial`
* If you have trouble with this step then refer to [this walkthrough.](https://www.youtube.com/watch?v=jnpC_Ib_lbc)

2. __Linux__: set up USB permissions
```
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/50-odrive.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger # until you reboot you may need to do this everytime you reset the ODrive
```
3. Power the ODrive board (as per the [Flashing the firmware](#flashing-the-firmware) step).
4. Plug in a USB cable into the microUSB connector on ODrive, and connect it to your PC.
5. __Windows__: Use the [Zadig](http://zadig.akeo.ie/) utility to set ODrive (not STLink!) driver to libusb-win32. 
  * If 'Odrive V3.x' is not in the list of devices upon opening Zadig, check 'List All Devices' from the options menu. With the Odrive selected in the device list choose 'libusb-win32' from the target driver list and select the large 'install driver' button.
6. Open the bash prompt in the `ODrive/tools/` folder.
7. Run `python3 demo.py` or `python3 explore_odrive.py`. 
- `demo.py` is a very simple script which will make motor 0 turn back and forth. Use this as an example if you want to control the ODrive yourself programatically.
- `explore_odrive.py` drops you into an interactive python shell where you can explore and edit the parameters that are available on your device. For instance `my_odrive.motor0.pos_setpoint = 10000` makes motor0 move to position 10000. To connect over serial instead of USB run `./tools/explore_odrive.py --discover serial`.

### From Arduino
[See ODrive Arduino Library](https://github.com/madcowswe/ODriveArduino)

### Other platforms
See the [protocol specification](protocol.md) or the [legacy protocol specification](legacy-protocol.md).

<br><br>
## Generating startup code
**Note:** You do not need to run this step to program the board. This is only required if you wish to update the auto generated code.

This project uses the STM32CubeMX tool to generate startup code and to ease the configuration of the peripherals.
You will likely want the pinout for this process. It is available [here](https://docs.google.com/spreadsheets/d/1QXDCs1IRtUyG__M_9WruWOheywb-GhOwFtfPcHuN2Fg/edit#gid=404444347)

### Installing prerequisites
* `stm32cubeMX`: Tool from STM to automatically generate setup routines and configure libraries, etc.
    * Available [here](http://www2.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stm32cubemx.html?icmp=stm32cubemx_pron_pr-stm32cubef2_apr2014&sc=stm32cube-pr2)

### Generate code
* Run stm32cubeMX and load the `stm32cubemx/Odrive.ioc` project file.
* Press `Project -> Generate code`
* You may need to let it download some drivers and such.

<br><br>
## Notes for Contributors
In general the project uses the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), except that the default indendtation is 4 spaces, and that the 80 character limit is not very strictly enforced, merely encouraged.

### Code maintenance notes
The cortex M4F processor has hardware single precision float unit. However double precision operations are not accelerated, and hence should be avoided. The following regex is helpful for cleaning out double constants:
find: `([-+]?[0-9]+\.[0-9]+(?:[eE][-+]?[0-9]+)?)([^f0-9e])`
replace: `\1f\2`
