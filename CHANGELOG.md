# Unreleased Features
Please add a note of your changes below this heading if you make a Pull Request.

# Releases
## [0.5.1] - 2020-09-27
### Added
* Added motor `torque_constant`: units of torque are now [Nm] instead of just motor current.
* [Motor thermistors support](docs/thermistors.md)
* Enable/disable of thermistor thermal limits according `setting axis.<thermistor>.enabled`.
* Introduced `odrive-interface.yaml` as a root source for the ODrive's API. `odrivetool` connects much faster as a side effect.
* Added torque_constant and torque_lim to motor config

### Changed
* **`input_pos`, `input_vel`, `pos_estimate_linear`, `pos_estimate_circular`, are now in units of [turns] or [turns/s] instead of [counts] or [counts/s]**
* `axis.motor.thermal_current_lim` has been removed. Instead a new property is available `axis.motor.effective_current_lim` which contains the effective current limit including any thermal limits.
* `axis.motor.get_inverter_temp()`, `axis.motor.inverter_temp_limit_lower` and `axis.motor.inverter_temp_limit_upper` have been moved to seperate fet thermistor object under `axis.fet_thermistor`. `get_inverter_temp()` function has been renamed to `temp` and is now a read-only property.
* `axis.config.counts_per_step` is now `axis.config.turns_per_step`
* Outputs of `axis.sensorless_estimator` are now in turns/s instead of electrical rad/s

### Fixed
* Fixed bug of high current during lockin-ramp caused by `motor::update()` expecting a torque command instead of current
* Fixed bug where commanded velocity was extremely high just after sensorless ramp when using `input_mode` INPUT_MODE_VEL_RAMP caused by `vel_setpoint` and `axis.config.sensorless_ramp.vel` being in different units

## [0.5.0] - 2020-08-03
### Added
* AC Induction Motor support.
  * Tracking of rotor flux through rotor time constant
  * Automatic d axis current for Maximum Torque Per Amp (MTPA)
* ASCII "w" commands now execute write hooks.
* Simplified control interface ("Input Filter" branch)
    * New input variables: `input_pos`, `input_vel`, and `input_current`
    * New setting `input_mode` to switch between different input behaviours
      * Passthrough
      * Velocity Ramp
      * 2nd Order Position Filter
      * Trapezoidal Trajectory Planner
    * Removed `set_xxx_setpoint()` functions and made `xxx_setpoint` variables read-only
* [Preliminary support for Absolute Encoders](docs/encoders.md)
* [Preliminary support for endstops and homing](docs/endstops.md)
* [CAN Communication with CANSimple stack](can-protocol.md)
* Gain scheduling for anti-hunt when close to 0 position error
* Velocity Limiting in Current Control mode according to `vel_limit` and `vel_gain`
* Regen current limiting according to `max_regen_current`, in Amps
* DC Bus hard current limiting according to `dc_max_negative_current` and `dc_max_positive_current`
* Brake resistor logic now attempts to clamp voltage according to `odrv.config.dc_bus_overvoltage_ramp_start` and `odrv.config.dc_bus_overvoltage_ramp_end`
* Unit Testing with Doctest has been started for select algorithms, see [Firmware/Tests/test_runner.cpp](Firmware/Tests/test_runner.cpp)
* Added support for Flylint VSCode Extension for static code analysis
* Using an STM32F405 .svd file allows CortexDebug to view registers during debugging
* Added scripts for building via docker.
* Added ability to change uart baudrate via fibre

### Changed
* Changed ratiometric `motor.config.current_lim_tolerance` to absolute `motor.config.current_lim_margin`
* Moved `controller.vel_ramp_enable` to INPUT_MODE_VEL_RAMP.
* Anticogging map is temporarily forced to 0.1 deg precision, but saves with the config
* Some Encoder settings have been made read-only
* Cleaned up VSCode C/C++ Configuration settings on Windows with recursive includePath
* Now compiling with C++17
* Fixed a firmware hang that could occur from unlikely but possible user input
* Added JSON caching to Fibre. This drastically reduces the time odrivetool needs to connect to an ODrive (except for the first time or after firmware updates).
* Fix IPython `RuntimeWarning` that would occur every time `odrivetool` was started.
* Reboot on `erase_configuration()`. This avoids unexpected behavior of a subsequent `save_configuration()` call, since the configuration is only erased from NVM, not from RAM.
* Change `motor.get_inverter_temp()` to use a property which was already being sampled at `motor.inverter_temp`
* Fixed a numerical issue in the trajectory planner that could cause sudden jumps of the position setpoint

## [0.4.12] - 2020-05-06
### Fixed
* Fixed a numerical issue in the trajectory planner that could cause sudden jumps of the position setpoint

## [0.4.11] - 2019-07-25
### Added
* Separate lockin configs for sensorless, index search, and general.
* Check current limit violation: added `ERROR_CURRENT_UNSTABLE`, `motor.config.current_lim_tolerance`.

### Changed
* Ascii command for reboot changed from `sb` to `sr`.

# Releases
## [0.4.10] - 2019-04-24
### Fixed
* Index search would trigger in the wrong place.

## [0.4.9] - 2019-04-23
### Added
* A release target for ODrive v3.6
* Communication watchdog feature.
* `encoder.set_linear_count(count)` function.
* Configurable encoder offset calibration distance and speed:`calib_scan_distance` and `calib_scan_omega`
* Encoder offset calibration debug variable `calib_scan_response`
* Lock-in drive feature
* Script to enable using a hall signal as index edge.

### Changed
* Moved `traptraj.A_per_css` to `controller.inertia`
* Refactored velocity ramp mode into the new general input filtering structure
* Encoder index search now based on the new lock-in drive feature

### Fixed
* Encoder index interrupts now disabled when not searching

## [0.4.8] - 2019-02-25
### Added
* `dump_errors()` utility function in odrivetool to dump, decode and optionally clear errors.
* `f` command to ascii protocol to get encoder position and velocity feedback.
* `q` command to ascii protocol. It is like the old `p` command, but velocity and current mean limits, not feed-forward.
* `ss`, `se`, `sr` commands to ascii protocol, for save config, erase config and reboot.
* `move_incremental` function for relative trajectory moves.
* `encoder.config.ignore_illegal_hall_state` option.
* `encoder.config.enable_phase_interpolation` option. Setting to false may reduce jerky pulsations at low speed when using hall sensor feedback.
* Analog input. Used the same way as the PWM input mappings.
* Voltage limit soft clamping instead of ERROR_MODULATION_MAGNITUDE in gimbal motor closed loop.
* Thermal current limit with linear derating.

### Changed
* Unified lockin drive modes. Current for index searching and encoder offset calibration now moved to axis.lockin.current.

### Fixed
* Added required 1.5 cycle phase shift between ADC and PWM, lack thereof caused unstable current controller at high eRPM.

## [0.4.7] - 2018-11-28
### Added
* Overspeed fault
* Current sense saturation fault.
* Suppress startup transients by sampling encoder estimate into position setpoint when entering closed loop control.
* Make step dir gpio pins configurable.
* Configuration variable `encoder.config.zero_count_on_find_idx`, true by default. Set to false to leave the initial encoder count to be where the axis was at boot.
* Circular position setpoint mode: position setpoints wrapped [0, cpr). Useful for infinite incremental position control.
* Velocity setpoint ramping. Use velocity control mode, and set `controller.vel_ramp_enable` to true. This will ramp `controller.vel_setpoint` towards `controller.vel_ramp_target` at a ramp rate of `controller.config.vel_ramp_rate`.

### Changed
* Increased switching frequency from around 8kHz to 24kHz. Control loops still run at 8kHz.
* Renamed `axis.enable_step_dir` to `axis.step_dir_active`
* New process for working with STM32CubeMX.

### Fixed
* Would get ERROR_CONTROL_DEADLINE_MISSED along with every ERROR_PHASE_RESISTANCE_OUT_OF_RANGE.
* ODrive tool can now run interactive nested scripts with "%run -i script.py"

## [0.4.6] - 2018-10-07
### Fixed
* Broken printing of floats on ascii protocol

## [0.4.5] - 2018-10-06
### Added
* **Trapezoidal Trajectory Planner**
* Hook to execute protocol property written callback
* -Wdouble-promotion warning to compilation

### Changed
* Make python tools compatible with python 2.7 (so it can be used with ROS)
  * Threading API constructor can't take the daemon parameter, so all thread creation had to be expanded out.
  * `TimeoutError` isn't defined, but it makes for more readable code, so I defined it as an OSError subclass.
  * `ModuleNotFoundError` is replaced by the older ImportError.
  * Print function imported from future
* Using new hooks to calculate:
  * `motor.config.current_control_bandwidth`
    * This deprecates `motor.set_current_control_bandwidth()`
  * `encoder.config.bandwidth`
* Default value for `motor.resistance_calib_max_voltage` changed to 2.0

### Fixed
* An issue where the axis state machine would jump in and out of idle when there is an error
* There is a [bug](https://github.com/ARM-software/CMSIS_5/issues/267) in the arm fast math library, which gives spikes in the output of arm_cos_f32 for input values close to -pi/2. We fixed the bug locally, and hence are using "our_arm_cos_f32".

## [0.4.4] - 2018-09-18
### Fixed
* Serious reliability issue with USB communication where packets on Native and the CDC interface would collide with each other.

## [0.4.3] - 2018-08-30
### Added
* `min_endstop` and `max_endstop` objects can be configured on GPIO
* Axes can be homed if `min_endstop` is enabled
* Encoder position count "homed" to zero when index is found.

### Changed
* We now enforce encoder offset calibration must happen after index is found (if using index)
* Renaming of the velocity estimate `pll_vel` -> `vel_estimate`.
* Hardcoded maximum inductance now 2500 uH.

### Fixed
* Incorrect shifting of offset during index callback
* Once you got an axis error `ERROR_INVALID_STATE` you could never clear it
* Char to int conversion to read motornum on arduino example
* GPIO above #5 would not be used correctly in some cases

## [0.4.2] - 2018-08-04
### Added
* Hall sensor feedback
* Configurable RC PWM input
* Ability to read axis FET temperature
* Config settings for:
  * `motor.config.requested_current_range`
  * `motor.config.current_control_bandwidth` and `motor.set_current_control_bandwidth`. Latter required to invoke gain recalculation.
  * `encoder.config.bandwidth`
  * `sensorless_estimator.config.pm_flux_linkage`

## [0.4.1] - 2018-07-01
### Fixed
* Encoder errors would show up as Axis error `ERROR_MOTOR_FAILED` instead of `ERROR_ENCODER_FAILED`.
* Various pip install dependencies
* Ability for python tools threads to quit properly
* dfuse error prints now python3 compatible

## [0.4.0] - 2018-06-10
### Added
* Encoder can now go forever in velocity/torque mode due to using circular encoder space.
* Protocol supports function return values
* bake Git-derived firmware version into firmware binary. The firmware version is exposed through the `fw_version_[...]` properties.
* `make write_otp` command to burn the board version onto the ODrive's one-time programmable memory. If you have an ODrive v3.4 or older, you should run this once for a better firmware update user experience in the future. Run the command without any options for more details. Once set, the board version is exposed through the `hw_version_[...]` properties.
* infrastructure to publish the python tools to PyPi. See `tools/setup.py` for details.
* Automated test script `run_tests.py`
* System stats (e.g. stack usage) are exposed under `<odrv>.system_stats`

### Changed
* DFU script updates
  * Verify the flash after writing
  * Automatically download firmware from GitHub releases if no file is provided
  * Retain configuration during firmware updates
* Refactor python tools
  * The scripts `explore_odrive.py`, `liveplotter.py`, `drv_status.py` and `rate_test.py` have been merged into one single `odrivetool` script. Running this script without any arguments provides the shell that `explore_odrive.py` used to provide.
  * The command line options of `odrivetool` have changed compared to the original `explore_odrive.py`. See `odrivetool --help` for more details.
  * `odrivetool` (previously `explore_odrive.py`) now supports controlling multiple ODrives concurrently (`odrv0`, `odrv1`, ...)
  * No need to restart the `odrivetool` shell when devices get disconnected and reconnected
  * ODrive accesses from within python tools are now thread-safe. That means you can read from the same remote property from multiple threads concurrently.
  * The liveplotter (`odrivetool liveplotter`, formerly `liveplotter.py`) does no longer steal focus and closes as expected
  * Add commands `odrivetool backup-config` and `odrivetool restore-config`
  * (experimental: start liveplotter from `odrivetool` shell by typing `start_liveplotter(lambda: odrv0.motor0.encoder.encoder_state)`)
* Set thread priority of USB pump thread above protocol thread
* GPIO3 not sensitive to edges by default
* The device now appears as a composite device on USB. One subdevice is still a CDC device (virtual COM port), the other subdevice is a vendor specific class. This should resolve several issues that were caused by conflicting kernel drivers or OS services.
* Add WinUSB descriptors. This will tell Windows >= 8 to automatically load winusb.sys for the ODrive (only for the vendor specific subdevice). This makes it possible to use the ODrive from userspace via WinUSB with zero configuration. The Python tool currently still uses libusb so Zadig is still required.
* Add a configuration to enable the ASCII protocol on USB at runtime. This will only enable the ASCII protocol on the USB CDC subdevice, not the vendor specific subdevice so the python tools will still be able to talk to the ODrive.

### Fixed
* Enums now transported with correct underlying type on native protocol
* USB issue where the device would stop responding when the host script would quit abruptly or reset the device during operation

## [0.3.6] - 2018-03-26
### Added
* **Storing of configuration parameters to Non Volatile Memory**
* **USB Bootloader**
* `make erase_config` to erase the configuration with an STLink (the configuration can also be erased from within explore_odrive.py, using `odrv0.erase_configuration()`)
* Travis-CI builds firmware for all board versions and deploys the binaries when a tag is pushed to master
* General purpose ADC API. See function get_adc_voltage() in low_level.cpp for more detais.

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
