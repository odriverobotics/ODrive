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
