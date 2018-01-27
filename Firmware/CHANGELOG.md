## UNRELEASED

### Added
* Gimbal motor mode

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
