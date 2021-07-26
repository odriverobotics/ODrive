---
redirect_from:
 - /getting-started
permalink: /
---

# Getting Started

### Table of contents
<!-- TOC depthFrom:2 depthTo:2 -->

- [Getting Started](#getting-started)
    - [Table of contents](#table-of-contents)
  - [Hardware Requirements](#hardware-requirements)
    - [You will need:](#you-will-need)
  - [Wiring up the ODrive](#wiring-up-the-odrive)
    - [Wiring up the motors](#wiring-up-the-motors)
    - [Wiring up the encoders](#wiring-up-the-encoders)
    - [Safety & Power UP](#safety--power-up)
  - [Downloading and Installing Tools](#downloading-and-installing-tools)
    - [Windows](#windows)
    - [OSX](#osx)
    - [Linux](#linux)
  - [Firmware](#firmware)
  - [Start `odrivetool`](#start-odrivetool)
  - [Debugging](#debugging)
  - [Configure M0](#configure-m0)
    - [1. Set the limits:](#1-set-the-limits)
    - [2. Set other hardware parameters](#2-set-other-hardware-parameters)
    - [3. Save configuration](#3-save-configuration)
  - [Position control of M0](#position-control-of-m0)
  - [Other control modes](#other-control-modes)
  - [Watchdog Timer](#watchdog-timer)
  - [What's next?](#whats-next)
  - [Upgrading from 0.4.12](#upgrading-from-0412)

<!-- /TOC -->

## Hardware Requirements

### You will need:
* One or two [brushless motors](https://docs.google.com/spreadsheets/d/12vzz7XVEK6YNIOqH0jAz51F5VUpc-lJEs3mmkWP1H4Y). It is fine, even recommended, to start testing with just a single motor and encoder.
* One or two [encoder(s)](https://docs.google.com/spreadsheets/d/1OBDwYrBb5zUPZLrhL98ezZbg94tUsZcdTuwiVNgVqpU)
* A power resistor. A good starting point would be the 50W resistor included with your ODrive.
  <details><summary markdown="span">Do I really need a power resistor? What values to choose?</summary><div markdown="block">

  If you don't have a brake resistor, the ODrive will pump excess power back into the power supply during deceleration to achieve the desired deceleration torque. If your power supply doesn't eat that power (which it won't if it's not a battery), the bus voltage will inevitebly rise. If you're unlucky this will break the power supply.
  At some point, the ODrive's overvoltage protection will trip, after which both motors will be allowed to spin freely. Depending on your machine, this may or may not be a problem.

  The power resistor values you need depends on your motor setup, and peak/average deceleration power.

  To be on the safe side, think about what speed and current limits you want to set for the motor.

  When braking at max speed and with maximum motor current, the power that is dissipated in the power resistor can be calulated as: `P_brake = V_emf * I_motor` where `V_emf = motor_rpm / motor_kv`.

  </div></details>

* A power supply (12V-24V for the 24V board variant, 12V-56V for the 56V board variant). A battery is also fine. Some advice on choosing a power supply can be found [here](https://things-in-motion.blogspot.com/2018/12/how-to-select-right-power-source-for.html).
  <details><summary markdown="span">What voltage variant do I have?</summary><div markdown="block">
  On all ODrives shipped July 2018 or after have a silkscreen label clearly indicating the voltage variant.

  ODrives before this may or may not have this label. If you don't have a label, then you can look at the bus capacitors (8 gray cylinder components on the underside of the board). If they read 470uF, you have a 24V version; if they read 120uF you have a 48V version.
  </div></details>

## Wiring up the ODrive
<div class="alert">
Firmware, software, and documentation is intended for use with ODrive motor controllers purchased from odriverobotics.com. ODrive Robotics does not sell products through any channel other than odriverobotics.com. We do not provide support for ODrives purchased elsewhere.
</div>
<div class="alert">
Make sure you have a good mechanical connection between the encoder and the motor, slip can cause disastrous oscillations or runaway.
</div>

All non-power I/O is 3.3V output and 5V tolerant on input, on ODrive v3.3 and newer.

### Wiring up the motors
* Connect the motor phases into the 3-phase screw terminals. It is not recommended to use a clip-on connector such as an alligator clip, as this can cause issues with the phase resistance/inductance measurements. 

### Wiring up the encoders
Connect the encoder(s) to J4. The A,B phases are required, and the Z (index pulse) is optional. The A,B and Z lines have 3.3k pull up resistors, for use with open-drain encoder outputs. For single ended push-pull signals with weak drive current (\<4mA), you may want to desolder the pull-ups.

![Image of ODrive all hooked up](https://docs.google.com/drawings/d/e/2PACX-1vTpJziAisrkvV1kTL4vckAJkmJ-BAvTwN1GeZZNCNwpTHv47Cf8bpz-gJqK2Z3un6FCHT4E-rcuUg6c/pub?w=1716&h=1281)

### Safety & Power UP
<div class="alert">
 Always think safety before powering up the ODrive if motors are attached. Consider what might happen if the motor spins as soon as power is applied.
</div>

* Unlike some devices, the ODrive does not recieve power over the USB port so the 24/56 volt power input is required even just to communicate with it using USB. It is ok to power up the ODrive before or after connecting the USB cable.

* To power up the ODrive, connect the power source to the DC terminals. Make sure to pay attention to the polarity. Try to connect the power source first and then turn it on to avoid inrush current. If this can't be avoided then a small spark is normal. This is caused by the capacitors charging up.

* Make sure to avoid a ground loop! See the [ground loop page](ground-loops.md) for details.

## Downloading and Installing Tools
Most instructions in this guide refer to a utility called `odrivetool`, so you should install that first.

### Windows
1. Install Python 3. We recommend the Anaconda distribution because it packs a lot of useful scientific tools, however you can also install the standalone python.
  * __Anaconda__: Download the installer from [here](https://www.anaconda.com/download/#windows). Execute the downloaded file and follow the instructions.
  * __Standalone Python__: Download the installer for 3.8.6 from [here](https://www.python.org/downloads/release/python-386/). Execute the downloaded file and follow the instructions.  As of Oct 2020, Matplotlib (required by odrivetool) had not been updated to work with 3.9, so please use 3.8.6.
  * If you have Python 2 installed alongside Python 3, replace `pip` by `C:\Users\YOUR_USERNAME\AppData\Local\Programs\Python\Python36-32\Scripts\pip`. If you have trouble with this step then refer to [this walkthrough](https://www.youtube.com/watch?v=jnpC_Ib_lbc).
2. Launch the command prompt.
  * __Anaconda__: In the start menu, type `Anaconda Prompt` <kbd>Enter</kbd>
  * __Standalone Python__: In the start menu, type `cmd` <kbd>Enter</kbd>
3. Install the ODrive tools by typing `pip install --upgrade odrive` <kbd>Enter</kbd>


### OSX
We are going to run the following commands for installation in Terminal.
1. If you don't already have it, install homebrew:
```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
2. Install python:
```bash
brew install python
```
3. If you get the error: `Error: python 2.7.14_2 is already installed`, then upgrade to Python 3 by running:
```bash
brew upgrade python
```
4. The odrive tools uses libusb to communicate to the ODrive:
```bash
brew install libusb
```
5. Now that you have Python 3 and all the package managers, run:
```bash
pip3 install --upgrade odrive
```

__Troubleshooting__
1. Permission Errors: Just run the previous command in sudo
   ```bash
   sudo pip3 install --upgrade odrive
   ```

2. Dependency Errors: If the installer doesn't complete and you get a dependency
error (Ex. "No module..." or "module_name not found")
   ```bash
   sudo pip3 install module_name
   ```
   Try step 5 again

3. Other Install Errors: If the installer fails at installing dependencies, try
   ```bash
   sudo pip3 install odrive --no-deps
   ```
   If you do this, brace yourself for runtime errors when you run `odrivetool` (the basic functionality should work though).


### Linux
1. [Install Python 3](https://www.python.org/downloads/). (for example, on Ubuntu, `sudo apt install python3 python3-pip`)
2. Install the ODrive tools by opening a terminal and typing `sudo pip3 install --upgrade odrive` <kbd>Enter</kbd>
    * This should automatically add the udev rules. If this fails for some reason you can add them manually:
    ```bash
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/91-odrive.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
3. **Ubuntu**, **Raspbian**: If you can't invoke `odrivetool` at this point, try adding `~/.local/bin` to your `$PATH` ([see related bug](https://unix.stackexchange.com/a/392710/176715)). This is done for example by running `nano ~/.bashrc`, scrolling to the bottom, pasting `export PATH=$PATH:~/.local/bin`, and then saving and closing, and close and reopen the terminal window.

## Firmware
**ODrive v3.5 and later**<br>
Your board should come preflashed with firmware. If you run into problems, follow the instructions [here](odrivetool.md#device-firmware-update) on the DFU procedure before you continue.

**ODrive v3.4 and earlier**<br>
Your board does **not** come preflashed with any firmware. Follow the instructions [here](odrivetool.md#device-firmware-update) on the ST Link procedure before you continue.

## Start `odrivetool`
To launch the main interactive ODrive tool, type `odrivetool` <kbd>Enter</kbd>. Connect your ODrive and wait for the tool to find it. If it doesn't connect after a few seconds refer to the [troubleshooting page](troubleshooting.md#usb-connectivity-issues). Now you can, for instance type `odrv0.vbus_voltage` <kbd>Enter</kbd> to inpect the boards main supply voltage.
It should look something like this:

```text
ODrive control utility v0.5.1
Please connect your ODrive.
Type help() for help.

Connected to ODrive 306A396A3235 as odrv0
In [1]: odrv0.vbus_voltage
Out[1]: 11.97055721282959
```

The tool you're looking at is a fully capable Python command prompt, so you can type any valid python code.

You can read more about `odrivetool` [here](odrivetool.md).

## Debugging
If any of the following steps fail, print the errors by running `dump_errors(odrv0)` in `odrivetool`. You can clear errors by running `odrv0.clear_errors()`.

## Configure M0
<div class="alert" markdown="span">Read this section carefully, else you risk breaking something.</div>
<div class="note" markdown="span">There is a [separate guide](hoverboard.md) specifically for hoverboard motors.</div>

### 1. Set the limits:
<details><summary markdown="span">Wait, how do I set these?</summary><div markdown="block">
In the previous step we started `odrivetool`. In there, you can assign variables directly by name.

For instance, to set the current limit of M0 to 10A you would type: `odrv0.axis0.motor.config.current_lim = 10` <kbd>Enter</kbd>
</div></details>

**Current limit**<br>
`odrv0.axis0.motor.config.current_lim` [A].  
The default current limit, for safety reasons, is set to 10A. This is quite weak, but good for making sure the drive is stable. Once you have tuned the oDrive, you can increase this to 60A to increase performance. Note that above 60A, you must change the current amplifier gains. You do this by requesting a different current range. i.e. for 90A on M0: `odrv0.axis0.motor.config.requested_current_range = 90` [A], then save the configuration and reboot as the gains are written out to the DRV (MOSFET driver) only during startup.

*Note: The motor current and the current drawn from the power supply is not the same in general. You should not look at the power supply current to see what is going on with the motor current.*
<details><summary markdown="span">Ok, so tell me how it actually works then...</summary><div markdown="block">
The current in the motor is only connected to the current in the power supply _sometimes_ and other times it just cycles out of one phase and back in the other. This is what the modulation magnitude is (sometimes people call this duty cycle, but that's a bit confusing because we use SVM not straight PWM). When the modulation magnitude is 0, the average voltage seen across the motor phases is 0, and the motor current is never connected to the power supply. When the magnitude is 100%, it is always connected, and at 50% it's connected half the time, and cycled in just the motor half the time.

The largest effect on modulation magnitude is speed. There are other smaller factors, but in general: if the motor is still it's not unreasonable to have 50A in the motor from 5A on the power supply. When the motor is spinning close to top speed, the power supply current and the motor current will be somewhat close to each other.
</div></details>

**Velocity limit**<br>
`odrv0.axis0.controller.config.vel_limit` [turn/s].  
The motor will be limited to this speed. Again the default value is quite slow.

**Calibration current**<br>
You can change `odrv0.axis0.motor.config.calibration_current` [A] to the largest value you feel comfortable leaving running through the motor continuously when the motor is stationary. If you are using a small motor (i.e. 15A current rated) you may need to reduce `calibration_current` to a value smaller than the default.

### 2. Set other hardware parameters
`odrv0.config.enable_brake_resistor`
Set this to `True` if using a brake resistor. You need to save the ODrive configuration and reboot the ODrive for this to take effect.

`odrv0.config.brake_resistance` [Ohm]  
This is the resistance of the brake resistor. You can leave this at the default setting if you are not using a brake resistor. Note that there may be some extra resistance in your wiring and in the screw terminals, so if you are getting issues while braking you may want to increase this parameter by around 0.05 ohm.

`odrv0.config.dc_max_negative_current` [Amps]
This is the amount of current allowed to flow back into the power supply. The convention is that it is negative. By default, it is set to a conservative value of 10mA. If you are using a brake resistor and getting `DC_BUS_OVER_REGEN_CURRENT` errors, raise it slightly. If you are not using a brake resistor and you intend to send braking current back to the power supply, set this to a safe level for your power source. Note that in that case, it should be higher than your motor current limit + current limit margin.
 
`odrv0.axis0.motor.config.pole_pairs`  
This is the number of **magnet poles** in the rotor, **divided by two**. To find this, you can simply count the number of permanent magnets in the rotor, if you can see them.
**Note**: This is **not** the same as the number of coils in the stator.
A good way to find the number of pole pairs is with a current limited power supply. Connect any two of the three phases to a power supply outputting around 2A, spin the motor by hand, and count the number of detents. This will be the number of pole pairs. If you can't distinguish the detents from the normal cogging present when the motor is disconnected, increase the current.
Another way is sliding a loose magnet in your hand around the rotor, and counting how many times it stops. This will be the number of _pole pairs_. If you use a ferrous piece of metal instead of a magnet, you will get the number of _magnet poles_.

`odrv0.axis0.motor.config.torque_constant`  
This is the ratio of torque produced by the motor per Amp of current delivered to the motor. This should be set to **8.27 / (motor KV)**.
If you decide that you would rather command torque in units of Amps, you could simply set the torque constant to 1.

`odrv0.axis0.motor.config.motor_type`  
This is the type of motor being used. Currently two types of motors are supported: High-current motors (`MOTOR_TYPE_HIGH_CURRENT`) and gimbal motors (`MOTOR_TYPE_GIMBAL`).
<details><summary markdown="span">Which <code>motor_type</code> to choose?</summary><div markdown="block">

If you're using a regular hobby brushless motor like [this](https://hobbyking.com/en_us/turnigy-aerodrive-sk3-5065-236kv-brushless-outrunner-motor.html) one, you should set `motor_mode` to `MOTOR_TYPE_HIGH_CURRENT`. For low-current gimbal motors like [this](https://hobbyking.com/en_us/turnigy-hd-5208-brushless-gimbal-motor-bldc.html) one, you should choose `MOTOR_TYPE_GIMBAL`. Do not use `MOTOR_TYPE_GIMBAL` on a motor that is not a gimbal motor, as it may overheat the motor or the ODrive.

**Further detail:**
If 100's of mA of current noise is "small" for you, you can choose `MOTOR_TYPE_HIGH_CURRENT`.
If 100's of mA of current noise is "large" for you, and you do not intend to spin the motor very fast (Ω * L << R), and the motor is fairly large resistance (1 ohm or larger), you can chose `MOTOR_TYPE_GIMBAL`.
If 100's of mA current noise is "large" for you, _and_ you intend to spin the motor fast, then you need to replace the shunt resistors on the ODrive.
</div></details> <br> 

*Note: When using gimbal motors,* `current_lim` *and* `calibration_current` *actually mean "voltage limit" and "calibration voltage", since we don't use current feedback. This means that if you set it to 10, it means 10V, despite the name of the parameter.*

**If using encoder**<br>
`odrv0.axis0.encoder.config.cpr`: Encoder Count Per Revolution [CPR]  
This is 4x the Pulse Per Revolution (PPR) value. Usually this is indicated in the datasheet of your encoder.

**If not using encoder**<br>
* If you wish to run in sensorless mode, please see [Setting up sensorless](commands.md#setting-up-sensorless).
* If you are using hall sensor feedback, please see the [hoverboard motor example](hoverboard.md).

**If using motor thermistor**<br>
Please see the [Thermistors](thermistors.md) page for setup.

### 3. Save configuration
You can save all `.config` parameters to persistent memory so the ODrive remembers them between power cycles. This will reboot the board.
* `odrv0.save_configuration()` <kbd>Enter</kbd>. 


## Position control of M0
Let's get motor 0 up and running. The procedure for motor 1 is exactly the same, so feel free to substitute `axis1` wherever it says `axis0`.

1. Type `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE` <kbd>Enter</kbd>. After about 2 seconds should hear a beep. Then the motor will turn slowly in one direction for a few seconds, then back in the other direction.

  <details><summary markdown="span">What's the point of this?</summary><div markdown="block">
  This procedure first measures your motor's electrical properties (namely phase resistance and phase inductance) and then the offset between the motor's electrical phase and the encoder position.
  </div></details>

  The startup procedure is demonstrated [here](https://www.youtube.com/watch?v=VCX1bA2xnuY).

  *Note: the rotor must be allowed to rotate without any biased load during startup. That means mass and weak friction loads are fine, but gravity or spring loads are not okay. Also note that in the video, the motors spin after initialization, but in the current software the default behaviour is not like that.*

  <details><summary markdown="span">Help, something isn't working!</summary><div markdown="block">
  
  Check the encoder wiring and that the encoder is firmly connected to the motor. Check the value of `dump_errors(odrv0)` and then refer to the [error code documentation](troubleshooting.md#error-codes) for details.

  Once you understand the error and have fixed its cause, you may clear the error state with (`odrv0.clear_errors()` <kbd>Enter</kbd>) and retry.
  </div></details>

2. Type `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL` <kbd>Enter</kbd>. From now on the ODrive will try to hold the motor's position. If you try to turn it by hand, it will fight you gently. That is unless you bump up `odrv0.axis0.motor.config.current_lim`, in which case it will fight you more fiercely. If the motor begins to vibrate either immediately or after being disturbed you will need to [lower the controller gains](control.md).
3. Send the motor a new position setpoint. `odrv0.axis0.controller.input_pos = 1` <kbd>Enter</kbd>. The units are in turns.
4. At this point you will probably want to [Properly tune](control.md) the motor controller in order to maximize system performance.

## Other control modes
The default control mode is unfiltered position control in the absolute encoder reference frame. You may wish to use a controlled trajectory instead. Or you may wish to control position in a circular frame to allow continuous rotation forever without growing the numeric value of the setpoint too large.

You may also wish to control velocity (directly or with a ramping filter).
You can also directly control the current of the motor, which is proportional to torque.

- [Filtered position control](control-modes.md#filtered-position-control)
- [Trajectory control](control-modes.md#trajectory-control)
- [Circular position control](control-modes.md#circular-position-control)
- [Velocity control](control-modes.md#velocity-control)
- [Ramped velocity control](control-modes.md#ramped-velocity-control)
- [Torque control](control-modes.md#torque-control)


## Watchdog Timer
Each axis has a configurable watchdog timer that can stop the motors if the
control connection to the ODrive is interrupted.

Each axis has a configurable watchdog timeout: `axis.config.watchdog_timeout`,
measured in seconds. Set `axis.config.enable_watchdog = True` to turn on this feature.

The watchdog is fed using the `axis.watchdog_feed()` method of each axis. Some [ascii commands](ascii-protocol.md#command-reference) feed the watchdog automatically.

## What's next?
You can now:
* [Properly tune](control.md) the motor controller to unlock the full potential of the ODrive.
* See what other [commands and parameters](commands.md) are available, in order to better control the ODrive.
* Control the ODrive from your own program or hook it up to an existing system through one of it's [interfaces](pinout.md).
* See how you can improve the behavior during the startup procedure, like [bypassing encoder calibration](encoders.md#encoder-with-index-signal).
* The CAN communication is the most reliable way of talking to ODrive in a real application.  Check out the [CAN Guide](can-guide.md) and [CAN Protocol](can-protocol.md)

If you have any issues or any questions please get in touch. The [ODrive Community](https://discourse.odriverobotics.com/) warmly welcomes you.


## Upgrading from 0.4.12
A new version (0.5.1) of ODrive firmware has released, complete with a new odrivetool.  Follow the installation instructions, making sure to add the `--upgrade` flag to pip commands, and check out the [Changelog](../CHANGELOG.md) for changes!

The odrivetool will stage and restore your configuration.  This probably isn't wise for the 0.4.12 -> 0.5.1 upgrade, so we suggest using `odrv0.erase_configuration()` immediately after connecting the first time.
