# Unreleased Features
Please add a note of your changes below this heading if you make a Pull Request.

### Added
* `make write_otp` command to burn the board version onto the ODrive's one-time programmable memory. If you have an ODrive v3.4 or older, you can run this once for a better firmware update user experience in the future. Run the command without any options for more details. Once set, the board version is exposed through the `board_version_[...]` properties.
* bake Git-derived firmware version into firmware binary. The firmware version is exposed through the `fw_version_[...]` properties.

### Changed
* The DFU script now verifies the flash after writing
* Set thread priority of USB pump thread above protocol thread
### Fixed
* Enums now transported with correct underlying type on native protocol

# Releases

## [0.3.6] - 2018-03-26

### Added
* **Storing of configuration parameters to Non Volatile Memory**
* **USB Bootloader**
* `make erase_config` to erase the configuration with an STLink (the configuration can also be erased from within explore_odrive.py, using `my_odrive.erase_configuration()`)
* Travis-CI builds firmware for all board versions and deploys the binaries when a tag is pushed to master

### Changed
* Most of the code from `lowlevel.c` moved to `axis.cpp`, `encoder.cpp`, `controller.cpp`, `sensorless_estimator.cpp`, `motor.cpp` and the corresponding header files
* Refactoring of the developer-facing communication protocol interface. See e.g. `axis.hpp` or `controller.hpp` for examples on how to add your own fields and functions
* Change of the user-facing field paths. E.g. `my_odrive.motor0.pos_setpoint` is now at `my_odrive.axis0.controller.pos_setpoint`. Names are mostly unchanged.
* Rewrite of the top-level per-axis state-machine
* The build is now configured using the `tup.config` file instead of editing source files. Make sure you set your board version correctly. See [here](README.md#configuring-the-build) for details.
* The toplevel directory for tup is now `Firmware`. If you used tup before, go to `Firmware` and run `rm -rd ../.tup; rm -rd build/*; make`.
* Update CubeMX generated STM platform code to version 1.19.0
* Remove `UUID_0`, `UUID_1` and `UUID_2` from USB protocol. Use `serial_number` instead.
* Freertos memory pool (task stacks, etc) now uses Core Coupled Memory.

### Fixed
* malloc now fails if we run out of memory (before it would always succeed even if we are out of ram...)

## [0.3.5] - 2018-03-04

### Added
* Reporting error if your encoder CPR is incorrect
* Ability to start anticogging calibration over USB protocol
* Reporting of DRV status/control registers and fault codes
* DRV status read script
* Microsecond delay function
* Travis-CI
* Firmware update over USB

### Changed
* Build system is now tup instead of make. Please check the [Readme](README.md#installing-prerequisites) for installation instructions.

## [0.3.4] - 2018-02-13

### Fixed
* Broken way to check for python 2. Python 2 not supported yet.

## [0.3.3] - 2018-02-12

### Added
* Liveplotter script
* Automatic recovery of USB halt/stall condition

### Changed
* Modified python code to be Python 2 compatible

### Fixed
* USB CSC (USB serial) now reports a sensible baud rate

## [0.3.2] - 2018-02-02

### Added
* Gimbal motor mode
* Encoder index pulse support
* `resistance_calib_max_voltage` parameter

## [0.3.1] - 2018-01-18

### Added
* UUID Endpoint
* Reporting of correct ODrive version on USB descriptor
* Getting started instructions for VSCode

### Changed
* USB Product ID to 0x0D32, as it is the only Pid we were allocated on [pid.codes](http://pid.codes/1209/0D32/)
* Recommended method to debug firmware from VSCode now uses Cortex-Debug extension instead of native-debug.
* Refactor IDE instructions into separate files

### Fixed
* Bug where the remote function calls from Python to the ODrive were not working properly.

## [0.3] - 2017-12-18
### Added
* **New binary communication protocol**
  * This is a much richer and more efficient binary protocol than the old human-readable protocol.
  * The old protocol is still available (but will be depricated eventually). You must manually chose to fall back on this protocol if you wish to still use it.
* Support for C++
* Demo scripts for getting started with commanding ODrive from python
* Protection from user setting current_lim higher than is measurable
* Current sense shunt values for HW v3.4
* Check DRV chip fault line

### Changed
* Shunt resistance values for v3.3 and earlier to include extra resistance of PCB
* Default HW revision to v3.4
* Refactoring of control code:
  * Lifted top layer of low_level.c into Axis.cpp

## [0.2.2] - 2017-11-17
### Fixed
* Incorrect TIM14 interrupt mapping on board v3.2 caused hard-fault

### Changed
* GPIO communication mode now defaults to NONE

## [0.2.1] - 2017-11-14
### Fixed
* USB communication deadlock
* EXTI handler redefiniton in V3.2

### Changed
* Resistance/inductance measurement now saved dispite errors, to allow debugging

## [0.2.0] - 2017-11-12
### Added
* UART communication
* Setting to select UART or Step/dir on GIPIO 1,2
* Basic Anti-cogging

## [0.1.0] - 2017-08-26
### Added
* Step/Dir interface
* this Changelog
* motor control interrupt timing diagram
* uint16 exposed variable type
* null termination to USB string parsing

### Changed
* Fixed Resistance measurement bug
* Simplified motor control adc triggers
* Increased AUX bridge deadtime
