# ODriveFirmware

If you wish to use the latest release, please use the `master` branch (this is the default branch GitHub will present you with).

If you are a developer, you are encouraged to use the `devel` branch, as it contains the latest features.

The project is under active development, so make sure to check the [Changelog](CHANGELOG.md) to keep track of updates.

### Table of contents

<!-- MarkdownTOC depth=2 autolink=true bracket=round -->

- [Configuring the build](#configuring-the-build)
- [Downloading and Installing Tools](#downloading-and-installing-tools)
- [Building and Flashing the Firmware](#building-and-flashing-the-firmware)
- [Setting up an IDE](#setting-up-an-ide)
- [Communicating over USB or UART](#communicating-over-usb-or-uart)
- [Configuring parameters](#configuring-parameters)
- [Encoder Calibration](#encoder-calibration)
- [Checking for error codes](#checking-for-error-codes)
- [Generating startup code](#generating-startup-code)
- [Notes for Contributors](#notes-for-contributors)

<!-- /MarkdownTOC -->

<br><br>
## Configuring the build
To correctly operate the ODrive, you need to supply some parameters. Some are mandatory, and if supplied incorrectly will cause the drive to malfunction.
In this section we will set the compile-time parameters, later we will also set the [run time parameters](#configuring-parameters).

To customize the compile time parameters, copy or rename the file `Firmware/tup.config.default` to `Firmware/tup.config` and edit the parameters in that file:

__CONFIG_BOARD_VERSION__: The board version you're using. Can be `v3.1`, `v3.2`, `v3.3`, `v3.4-24V` or `v3.4-48V`. Check for a label on the upper side of the ODrive to find out which version you have.

__CONFIG_USB_PROTOCOL__: Defines which protocol the ODrive should use on the USB interface.
 * `native`: The native ODrive protocol. Use this if you want to use the python tools in this repo.
 * `native-stream`: Like the native ODrive protocol, but the ODrive will treat the USB connection exactly as if it was a UART connection. __Use this if you're on macOS__. This is necessary because macOS doesn't grant our python tools sufficient low-level access to treat the device as the USB device that it is.
 * `ascii`: The ASCII protocol. This allows sending simple commands like position setpoints directly from the terminal to the ODrive without the use of intermediate utilities.
 * `none`: Disable USB. The device will still show up when plugged in but it will ignore any commands.

__CONFIG_UART_PROTOCOL__: Defines which protocol the ODrive should use on the UART interface (GPIO1 and GPIO2). Note that UART is only supported on ODrive v3.3 and higher.
 * `native`: The native ODrive protocol. Use this if you're connecting the ODrive to a PC using UART and want to use the python tools to control and setup the ODrive.
 * `ascii`: The ASCII protocol. Use this option if you control the ODrive with an Arduino. The ODrive Arduino library is not yet updated to the native protocol.
 * `none`: Disable UART.


<br><br>
## Downloading and Installing Tools
### Getting a programmer
__Note:__ If you don't plan to make major firmware modifications you can use the built-in DFU feature.
In this case you don't need an SWD programmer and you can skip OpenOCD related instructions.

Get a programmer that supports SWD (Serial Wire Debugging) and is ST-link v2 compatible. You can get them really cheap on [eBay](http://www.ebay.co.uk/itm/ST-Link-V2-Emulator-Downloader-Programming-Mini-Unit-STM8-STM32-with-20CM-Line-/391173940927?hash=item5b13c8a6bf:g:3g8AAOSw~OdVf-Tu) or many other places.

### Installing prerequisites
To compile the program, you first need to install the prerequisite tools:

#### Linux:
* `gcc-arm-none-eabi`: GCC compilation toolchain for ARM microcontrollers.
    * Installing on Ubuntu: `sudo apt-get install gcc-arm-none-eabi`
    * Installing on Arch Linux: `sudo pacman -S arm-none-eabi-gcc arm-none-eabi-binutils`
* `gdb-arm-none-eabi`: GNU project debugger for ARM microcontrollers.
    * Installing on Ubuntu: `sudo apt-get install gdb-arm-none-eabi`
    * Installing on Arch Linux: `sudo pacman -S arm-none-eabi-gdb`
* `OpenOCD`: Open On-Chip Debugging tools. This is what we use to flash the code onto the microcontroller.
    * Installing on Ubuntu: `sudo apt-get install openocd`
    * Installing on Arch Linux: build and install the [AUR package](https://aur.archlinux.org/packages/openocd/)
* `tup`: Used as a build tool
    * Installing on Ubuntu: `sudo add-apt-repository ppa:jonathonf/tup; sudo apt-get update; sudo apt-get install tup`
    * Installing on Arch Linux: `sudo pacman -S tup`
* No additional USB CDC driver should be required on Linux.

#### Mac:
* `brew cask install gcc-arm-embedded`:  GCC toolchain+debugger
* `brew cask install osxfuse; brew install tup`: Build tool
* `brew install openocd`: Programmer

#### Windows:
Install the following:
* [Git for windows](https://git-scm.com/download/win). This intalls the Git Bash, which is a unix style command line interface that we will be using. 
* [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). The cross-compiler used to compile the code. Download and install the "Windows 32-bit" version. Make sure to tick the "add to path" option.
* [Tup](http://gittup.org/tup/index.html) is used to script the compilation process. Unpack the zip-folder. Then add the path that contains the executable (something like `C:\Users\yourname\the\place\you\unzipped\tup-latest`) to your PATH environment variable. For details on how to set your path envirment in windows see [these instructions.](https://www.java.com/en/download/help/path.xml)
* [Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm). This is optional and mainly used because commands are short and developers are used to it. If you don't want to use it, you can look at the Makefile to get the corresponding commands. Download and run the complete package setup program. Add the path of the binaries to your PATH environment variable. For me this was at `C:\Program Files (x86)\GnuWin32\bin`.
* OpenOCD. Follow the instructions at [GNU ARM Eclipse  - How to install the OpenOCD binaries](http://gnuarmeclipse.github.io/openocd/install/), including the part about ST-LINK/V2 drivers. Add the path of the binaries to your PATH environment variable. For me this was at `C:\Program Files\GNU ARM Eclipse\OpenOCD\0.10.0-201704182147-dev\bin`.

After installing all of the above, open a Git Bash shell. Continue at section [Building and Flashing the Firmware](#building-and-flashing-the-firmware).

<br><br>
## Building and Flashing the Firmware

### Building the firmware
* Make sure you have cloned the repository. It is recommended that you use [git](https://help.github.com/articles/cloning-a-repository/) for this.
* Navigate your terminal (windows: Git Bash/cygwin) to the ODrive/Firmware dir.
* Run `make` in the `Firmware` directory.

### Flashing the firmware (standalone device)
Note: ODrive v3.4 and earlier require you to flash with the external programmer first (see below), before you can reflash in standalone mode.
* __Windows__: Use the [Zadig](http://zadig.akeo.ie/) utility to set ODrive (not STLink!) driver to libusb-win32. 
  * If 'Odrive version 3.x' is not in the list of devices upon opening Zadig, check 'List All Devices' from the options menu. With the Odrive selected in the device list choose 'libusb-win32' from the target driver list and select the large 'install driver' button.
* Run `make dfu` in the `Firmware` directory.
* __Windows__: During the update, a new device called "STM32 BOOTLOADER" will appear. Open the Zadig utility that you used when you first connected your ODrive and set the driver for "STM32 BOOTLOADER" to libusb-win32. After that the firmware upgrade will continue.
* On some machines you will need to unplug and plug back in the usb cable to make the PC understand that we switched from regular mode to bootloader mode.

If you have multiple ODrives connected, you should specify which one to upgrade.
* Run `(lsusb -d 1209:0d32 -v; lsusb -d 0483:df11 -v) 2>/dev/null | grep iSerial` to list the serial number of all flashable devices. Example output:
```
  iSerial                 3 385F324D3037
  iSerial                 3 306A396A3235
```
* The last column is the serial number you're looking for. You can unplug selected devices to track down the one you want to update.
* If you only get `iSerial     3`, then you can try to substitute `sudo lsusb` in place of `lsusb`.
* Run `make dfu SERIAL_NUMBER=385F324D3037`, where `385F324D3037` is the targeted serial number.

__Warning:__ Currently it is advised that you only do this to flash
official unmodified firmware. Also make sure you don't switch off
the device during upgrade. Otherwise, if something goes wrong, you need an external
programmer to recover the device. This will be fixed in the future.

### Flashing the firmware (external Programmer)
* **Make sure you have [configured the build first](#configuring-the-build)**
* Connect `GND`, `SWD`, and `SWC` on connector J2 to the programmer. Note: Always plug in `GND` first!
* You need to power the board by only **ONE** of the following: VCC(3.3v), 5V, or the main power connection (the DC bus). The USB port (J1) does not power the board.
* Run `make flash` in the `Firmware` directory.

If the flashing worked, you can start sending commands. If you want to do that now, you can go to [Communicating over USB or UART](#communicating-over-usb-or-uart).

### Debugging the firmware
* Run `make gdb`. This will reset and halt at program start. Now you can set breakpoints and run the program. If you know how to use gdb, you are good to go.

<br><br>
## Setting up an IDE
For working with the ODrive code you don't need an IDE, but the open-source IDE VSCode is recommended.  It is also possible to use Eclipse. If you'd like to go that route, please see the respective configuration document:

* [Configuring VSCode](configuring-vscode.md)
* [Configuring Eclipse](configuring-eclipse.md)

<br><br>
## Communicating over USB or UART
Warning: If testing USB or UART communication for the first time it is recommend that your motors are free to spin continuously and are not connected to a drivetrain with limited travel.
### From Linux/Windows/macOS
There are two example python scripts to help you get started with controlling the ODrive using python. One will drop you into an interactive shell to query settings, parameters, and variables, and let you send setpoints manually ([tools/explore_odrive.py](../tools/explore_odrive.py)). The other is a demo application to show you how to control the ODrive programmatically ([tools/demo.py](../tools/demo.py)). Below follows a step-by-step guide on how to run these.


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
  * If 'Odrive version 3.x' is not in the list of devices upon opening Zadig, check 'List All Devices' from the options menu. With the Odrive selected in the device list choose 'libusb-win32' from the target driver list and select the large 'install driver' button.
6. Open the bash prompt in the `ODrive/tools/` folder.
7. Run `python3 demo.py` or `python3 explore_odrive.py`. 
- `demo.py` is a very simple script which will make motor 0 turn back and forth. Use this as an example if you want to control the ODrive yourself programatically.
- `explore_odrive.py` drops you into an interactive python shell where you can explore and edit the parameters that are available on your device. For instance `my_odrive.motor0.pos_setpoint = 10000` makes motor0 move to position 10000. To connect over serial instead of USB run `./tools/explore_odrive.py --discover serial`.

### From Arduino
[See ODrive Arduino Library](https://github.com/madcowswe/ODriveArduino)

### Other platforms
See the [protocol specification](protocol.md) or the [ASCII protocol specification](ascii-protocol.md).

<br><br>
## Configuring parameters
The majority of the important parameters you would want to set after flashing the ODrive with firmware are configurable over the USB communication interface. These include some mandatory parameters that you must set for correct operation, as well as tuning and optional parameters.
To start the configuration session:

* Launch `./tools/explore_odrive.py`. This will give you a command prompt where you can modify using simple assignments.
* Configure parameters of the `my_odrive.[...].config` objects.
  * For example to adjust the position gain: `my_odrive.motor0.config.pos_gain = 30` <kbd>Enter</kbd>.
  * The complete list of configurable parameters is:
    * `my_odrive.motorN.config.*`
    * `my_odrive.axisN.config.*`
    * where N is a valid motor number (0 or 1).
* Save the configuration into non-volatile memory: `my_odrive.save_configuration()` <kbd>Enter</kbd>
  * This will save the properties of all the `[...].config` objects and no other parameters.
* Reboot the drive: `my_odrive.reboot()` <kbd>Enter</kbd>

Note that a firmware upgrade at this point will preserve the configuration if and only if the parameters of both firmware versions are identical. Should you need to reset the configuration, you can run `my_odrive.erase_configuration()`.

__Developers__: Be aware that you can also modify the compile-time defaults for all of these parameters. Most of them you will find at the top of [MotorControl/low_level.c](MotorControl/low_level.c#L50). Note that the configuration parameters there are somewhat intertwined with runtime variables and hardware specific configuration that should not be changed. Also note that all parameters occur twice.

### Mandatory parameters
You must set for every motor:
* `my_odrive.motorN.encoder.config.cpr`: Encoder Count Per Revolution (CPR). This is 4x the Pulse Per Revolution (PPR) value.
* `my_odrive.motorN.config.pole_pairs`: This is the number of magnet poles in the rotor, **divided by two**. You can simply count the number of permanent magnets in the rotor, if you can see them. Note: this is not the same as the number of coils in the stator.
* `my_odrive.config.brake_resistance` [Ohm]: This is the resistance of the brake resistor. If you are not using it, you may set it to 0.0f.
* `my_odrive.motorN.config.motor_type`: This is the type of motor being used. Currently two types of motors are supported -- High-current motors (`MOTOR_TYPE_HIGH_CURRENT`) and Gimbal motors (`MOTOR_TYPE_GIMBAL`).

#### Motor Modes
If you're using a regular hobby brushless motor like [this](https://hobbyking.com/en_us/turnigy-aerodrive-sk3-5065-236kv-brushless-outrunner-motor.html) one, you should set `motor_mode` to `MOTOR_TYPE_HIGH_CURRENT`. For low-current gimbal motors like [this](https://hobbyking.com/en_us/turnigy-hd-5208-brushless-gimbal-motor-bldc.html) one, you should choose `MOTOR_TYPE_GIMBAL`. Do not use `MOTOR_TYPE_GIMBAL` on a motor that is not a gimbal motor, as it may overheat the motor or the ODrive.

**Further detail:**
If 100's of mA of current noise is "small" for you, you can choose `MOTOR_TYPE_HIGH_CURRENT`.
If 100's of mA of current noise is "large" for you, and you do not intend to spin the motor very fast (omega * L << R), and the motor is fairly large resistance (1 ohm or larger), you can chose `MOTOR_TYPE_GIMBAL`.
If 100's of mA current noise is "large" for you, and you intend to spin the motor fast, then you need to replace the shunt resistors on the ODrive.

### Tuning parameters
The most important parameters are the limits:
* The current limit: `my_odrive.motorN.current_control.config.current_lim` [A]. The default current limit, for safety reasons, is set to 10A. This is quite weak, and good for making sure the drive is stable. Once you have tuned the drive, you can increase this to 75A to get some performance. Note that above 75A, you must change the current amplifier gains.
  * Note: The motor current and the current drawn from the power supply is not the same in general. You should not look at the power supply current to see what is going on with the motor current.
* The velocity limit: `my_odrive.motorN.config.vel_limit` [counts/s]. The motor will be limited to this speed; again the default value is quite slow.
* You can change `my_odrive.motorN.config.calibration_current` [A] to the largest value you feel comfortable leaving running through the motor continously when the motor is stationary.

The motion control gains are currently manually tuned:
* `my_odrive.motorN.config.pos_gain = 20.0f` [(counts/s) / counts]
* `my_odrive.motorN.config.vel_gain = 15.0f / 10000.0f` [A/(counts/s)]
* `my_odrive.motorN.config.vel_integrator_gain = 10.0f / 10000.0f` [A/(counts/s * s)]

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
If you want a different mode, you can change `my_odrive.motorN.config.control_mode`.
Possible values are:
* `CTRL_MODE_POSITION_CONTROL`
* `CTRL_MODE_VELOCITY_CONTROL`
* `CTRL_MODE_CURRENT_CONTROL`
* `CTRL_MODE_VOLTAGE_CONTROL` - this one is not normally used.

To disable a motor at startup, set `my_odrive.axisN.config.enable_control` and `my_odrive.axisN.config.do_calibration` to `False`.

<br><br>
## Encoder Calibration
By default the encoder-to-motor calibration will run on every startup. During encoder calibration the rotor must be allowed to rotate without any biased load during startup. That means mass and weak friction loads are fine, but gravity or spring loads are not okay.

### Encoder with Index signal
If you have an encoder with an index (Z) signal, you may avoid having to do the calibration on every startup, and instead use the index signal to re-sync the encoder to a stored calibration. Below are the steps to do the one-time calibration and configuration. Note that you can follow these steps with one motor at a time, or all motors together, as you wish.

* Since you will only do this once, it is recommended that you mechanically disengage the motor from anything other than the encoder, so it can spin freely.
* All the parameters we will be modifying are in the motor structs at the top of [MotorControl/low_level.c](MotorControl/low_level.c).
* Set `.encoder.use_index = true` and `.encoder.calibrated = false`.
* Flash this configuration, and let the motor scan for the index pulse and then complete the encoder calibration.
* Run `explore_odrive.py`, check [Communicating over USB or UART](#communicating-over-usb-or-uart) for instructions on how to do that.
* Enter the following to print out the calibration parameters (substitute the motor number you are calibrating for `<NUM>`):
  * `my_odrive.motor<NUM>.encoder.encoder_offset` - This should print a number, like -326 or 1364.
  * `my_odrive.motor<NUM>.encoder.motor_dir` - This should print 1 or -1.
* Copy these numbers to the corresponding entries in low_level.c: `.encoder.encoder_offset` and `.encoder.motor_dir`.
  * _Warning_: Please be careful to enter the correct numbers, and not to confuse the motor channels. Incorrect values may cause the motor to spin out of control.
* Set `.encoder.calibrated = true`.
* Flash this configuration and check that the motor scans for the index pulse but skips the encoder calibration.
* Congratulations, you are now done. You may now attach the motor to your mechanical load.
* If you wish to scan for the index pulse in the other direction (if for example your axis usually starts close to a hard-stop), you can set a negative value in `.encoder.idx_search_speed`.
* If your motor has problems reaching the index location due to the mechanical load, you can increase `.calibration_current`.

<br><br>
## Checking for error codes
`explore_odrive.py`can also be used to check error codes when your odrive is not working as expected. For example `my_odrive.motor0.error` will list the error code associated with motor 0.
<br><br>
The error nummber corresponds to the following:

0. `ERROR_NO_ERROR`
1. `ERROR_PHASE_RESISTANCE_TIMING`
2. `ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT`
3. `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE`
4. `ERROR_PHASE_INDUCTANCE_TIMING`
5. `ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT`
6. `ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE`
7. `ERROR_ENCODER_RESPONSE`
8. `ERROR_ENCODER_MEASUREMENT_TIMEOUT`
9. `ERROR_ADC_FAILED`
10. `ERROR_CALIBRATION_TIMING`
11. `ERROR_FOC_TIMING`
12. `ERROR_FOC_MEASUREMENT_TIMEOUT`
13. `ERROR_SCAN_MOTOR_TIMING`
14. `ERROR_FOC_VOLTAGE_TIMING`
15. `ERROR_GATEDRIVER_INVALID_GAIN`
16. `ERROR_PWM_SRC_FAIL`
17. `ERROR_UNEXPECTED_STEP_SRC`
18. `ERROR_POS_CTRL_DURING_SENSORLESS`
19. `ERROR_SPIN_UP_TIMEOUT`
20. `ERROR_DRV_FAULT`
21. `ERROR_NOT_IMPLEMENTED_MOTOR_TYPE`
22. `ERROR_ENCODER_CPR_OUT_OF_RANGE`
23. `ERROR_DC_BUS_BROWNOUT`

If you get an error code larger than this, it may be the case that someone added a code and forgot to update the documentation. In that case, please check [MotorControl/low_level.h](MotorControl/low_level.h) for the full enum.

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
* After generating/updating the code, some minor patches need to be applied. To do this, run:
  `git apply Firmware/Board/v3/*.patch`
* Run `git config --local core.autocrlf input`. This will tell git that all files should be checked in with LF endings (CubeMX generates CRLF endings).
* `git status` will still claim that many files are modified but the actual diff (using `git diff`) is empty (apart from all the line ending warnings).

### Generating patchfiles
If you made changes to CubeMX generated files outside of the `USER CODE BEGIN`...`USER CODE END` sections and contribute them back, please add a patch file so that the next person who runs CubeMX doesn't run into problems.

CubeMX will reset everything outside these sections to the original state; we will capturing into a patch file the changes required to undo this resetting.
* Make sure your current desired state is committed.
* Make a new temporary branch: `git checkout -b cubemx_temp`
* Run the CubeMX code generation as described in the previous section, including applying previous patches.
* The diff will now _not_ be empty since CubeMX reset your changes.
* Stage this state and commit it with a message like "CubeMX reset my changes".
* Run `git revert HEAD` to undo the resetting action CubeMX's regeneration had. This is the commit which you will export, so write a meaningful commit message.
* Run `git format-patch HEAD~1` to export the commit as patch file.
* Check out your previous branch and then force-delete the temporary branch: `git branch -D cubemx_temp`
* Move the patch file to `Board/v3/` and add it in a new commit.

<br><br>
## Notes for Contributors
In general the project uses the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), except that the default indendtation is 4 spaces, and that the 80 character limit is not very strictly enforced, merely encouraged.

### Code maintenance notes
The cortex M4F processor has hardware single precision float unit. However double precision operations are not accelerated, and hence should be avoided. The following regex is helpful for cleaning out double constants:
find: `([-+]?[0-9]+\.[0-9]+(?:[eE][-+]?[0-9]+)?)([^f0-9e])`
replace: `\1f\2`
