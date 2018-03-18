# Unreleased Features
Please add a note of your changes below this heading if you make a Pull Request.

### Added
* **Storing of configuration parameters to Non Volatile Memory**
* **USB Bootloader**

### Changed
* The build is now configured using the `tup.config` file instead of editing source files. Make sure you set your board version correctly. See [here](README.md#configuring-the-build) for details.
* The toplevel directory for tup is now `Firmware`. If you used tup before, go to `Firmware` and run `rm -rd ../.tup; rm -rd build/*; make`.
* Update CubeMX generated STM platform code to version 1.19.0
* Remove `UUID_0`, `UUID_1` and `UUID_2` from USB protocol. Use `serial_number` instead.
* Freertos memory pool (task stacks, etc) now uses Core Coupled Memory.
* Refactor python tools
   * `explore_odrive.py` now supports controlling multiple ODrives concurrently (`odrv0`, `odrv1`, ...)
   * No need to restart `explore_odrive.py` when devices get disconnected and reconnected
   * The command line arguments of `explore_odrive.py` have changed. See `explore_odrive.py --help` for more details.
   * ODrive accesses from within python tools are now thread-safe
   * Liveplotter does no longer steal focus and closes as expected
   * (experimental: start liveplotter from `explore_odrive.py` by typing `start_liveplotter(lambda: odrv0.motor0.encoder.encoder_state)`)

### Fixed
* malloc now fails if we run out of memory (before it would always succeed even if we are out of ram...)

# Releases

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
