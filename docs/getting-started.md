---
redirect_from:
 - /getting-started
permalink: /
---

# Getting Started

### Table of contents

<!-- MarkdownTOC depth=2 autolink=true bracket=round -->

- [Hardware Requirements](#hardware-requirements)
- [Wiring up the ODrive](#wiring-up-the-odrive)
- [Downloading and Installing Tools](#downloading-and-installing-tools)
- [Start `odrivetool`](#start-odrivetool)
- [Configure M0](#configure-m0)
- [Position control of M0](#position-control-of-m0)
- [What's next?](#whats-next)

<!-- /MarkdownTOC -->

## Hardware Requirements

You will need:

* One or two [brushless motors](https://docs.google.com/spreadsheets/d/12vzz7XVEK6YNIOqH0jAz51F5VUpc-lJEs3mmkWP1H4Y). It is fine, even recommended, to start testing with just a single motor and encoder.
* One or two [quadrature incremental encoder(s)](encoders)
* A power resistor. A good starting point would be the 50W resistor included with your ODrive.
  <details><summary markdown="span">Do I really need a power resistor? What values to choose?</summary><div markdown="block">

  If you don't have a brake resistor, the ODrive will pump excess power back into the power supply during deceleration to achieve the desired deceleration torque. If your power supply doesn't eat that power (which it won't if it's not a battery), the bus voltage will inevitebly rise. If you're unlucky this will break the power supply.
  At some point, the ODrive's overvoltage protection will trip, after which both motors will be allowed to spin freely. Depending on your machine, this may or may not be a problem.

  The power resistor values you need depends on your motor setup, and peak/average deceleration power.

  To be on the safe side, think about what speed and current limits you want to set for the motor.

  When braking at max speed and with maximum motor current, the power that is dissipated in the power resistor can be calulated as: `P_brake = V_emf * I_motor` where `V_emf = motor_rpm / motor_kv`.

  </div></details>

* A power supply (12V-24V for the 24V board variant, 12V-48V for the 48V board variant). A battery is also fine.
   <details><summary markdown="span">What voltage variant do I have?</summary><div markdown="block">
   On all ODrives shipped July 2018 or after have a silkscreen label clearly indicating the voltage variant.

   ODrives before this may or may not have this label. If you don't have a label, then you can look at the bus capacitors (8 gray cylinder components on the underside of the board). If they read 470uF, you have a 24V version; if they read 120uF you have a 48V version.
   </div></details>

## Wiring up the ODrive

<div class="alert">
Make sure you have a good mechanical connection between the encoder and the motor, slip can cause disasterous oscillations or runaway.
</div>

All non-power I/O is 3.3V output and 5V tolerant on input, on ODrive v3.3 and newer.

1. Wire up the motor phases into the 3-phase screw terminals, and the power resistor to the AUX terminal. Wire up the power source to the DC terminal, make sure to pay attention to the polarity. Do not apply power just yet.

2. Wire up the encoder(s) to J4. The A,B phases are required, and the Z (index pulse) is optional. The A,B and Z lines have 3.3k pull up resistors, for use with open-drain encoder outputs. For single ended push-pull signals with weak drive current (\<4mA), you may want to desolder the pull-ups.

![Image of ODrive all hooked up](https://docs.google.com/drawings/d/e/2PACX-1vTCD0P40Cd-wvD7Fl8UYEaxp3_UL81oI4qUVqrrCJPi6tkJeSs2rsffIXQRpdu6rNZs6-2mRKKYtILG/pub?w=1716&h=1281)

## Downloading and Installing Tools

Most instructions in this guide refer to a utility called `odrivetool`, so you should install that first.

### Windows

1. Install Python 3. We recommend the Anaconda distribution because it packs a lot of useful scientific tools, however you can also install the standalone python.
   * __Anaconda__: Download the installer from [here](https://www.anaconda.com/download/#windows). Execute the downloaded file and follow the instructions.
   * __Standalone Python__: Download the installer from [here](https://www.python.org/downloads/). Execute the downloaded file and follow the instructions.
   * If you have Python 2 installed alongside Python 3, replace `pip` by `C:\Users\YOUR_USERNAME\AppData\Local\Programs\Python\Python36-32\Scripts\pip`. If you have trouble with this step then refer to [this walkthrough](https://www.youtube.com/watch?v=jnpC_Ib_lbc).
2. Launch the command prompt.
   * __Anaconda__: In the start menu, type `Anaconda Prompt` <kbd>Enter</kbd>
   * __Standalone Python__: In the start menu, type `cmd` <kbd>Enter</kbd>
3. Install dependencies by typing `pip install pywin32==222` <kbd>Enter</kbd>
3. Install the ODrive tools by typing `pip install odrive` <kbd>Enter</kbd>
4. Plug in a USB cable into the microUSB connector on ODrive, and connect it to your PC.
5. Use the [Zadig](http://zadig.akeo.ie/) utility to set ODrive driver to libusb-win32.
  * Check 'List All Devices' from the options menu, and select 'ODrive 3.x Native Interface (Interface 2)'. With that selected in the device list choose 'libusb-win32' from the target driver list and then press the large 'install driver' button.


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
pip3 install odrive
```

__Troubleshooting__
1. Permission Errors: Just run the previous command in sudo
```bash
sudo pip3 install odrive
```

2. Dependency Errors: If the installer doesn't complete and you get a dependency
error (Ex. "No module..." or "module_name not found")
```bash
sudo pip3 install module_name
```
Try step 5 again


### Linux

1. [Install Python 3](https://www.python.org/downloads/).
2. Install the ODrive tools by opening a terminal and typing `pip install odrive` <kbd>Enter</kbd>
3. __Linux__: set up USB permissions
```bash
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/50-odrive.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger # until you reboot you may need to do this everytime you reset the ODrive
```

## Start `odrivetool`

<div class="note" markdown="span">__ODrive v3.5 and later:__ Your board should come preflashed with firmware. If you run into problems, follow the instructions [here](odrivetool#device-firmware-update) on the DFU procedure before you continue.</div>

<div class="note" markdown="span">__ODrive v3.4 and earlier:__ Your board does __not__ come preflashed with any firmware. Follow the instructions [here](odrivetool#device-firmware-update) on the STP Link procedure before you continue.</div>

To launch the main interactive ODrive tool, type `odrivetool` <kbd>Enter</kbd>. Connect your ODrive and wait for the tool to find it. Now you can for instance type `odrv0.vbus_voltage` <kbd>Enter</kbd> to inpect the boards main supply voltage.
It should look something like this:

```text
ODrive control utility v0.4.0
Please connect your ODrive.
Type help() for help.

Connected to ODrive 306A396A3235 as odrv0
In [1]: odrv0.vbus_voltage
Out[1]: 11.97055721282959
```

The tool you're looking at is a fully capable Python command prompt, so you can type any valid python code.

You can read more about the odrivetool [here](odrivetool.md).

## Configure M0

<div class="alert">Read this section carefully, else you risk breaking something.</div>

1. Set the limits:

   <details><summary markdown="span">Wait, how do I set these?</summary><div markdown="block">

   In the previous step we started `odrivetool`. In there, you can assign variables directly by name.

   For instance, to set the current limit of M0 to 10A you would type: `odrv0.axis0.motor.config.current_lim = 10` <kbd>Enter</kbd>

   </div></details>

   * The current limit: `odrv0.axis0.motor.config.current_lim` [A]. The default current limit, for safety reasons, is set to 10A. This is quite weak, and good for making sure the drive is stable. Once you have tuned the drive, you can increase this to 75A to get some performance. Note that above 75A, you must change the current amplifier gains.
     * Note: The motor current and the current drawn from the power supply is not the same in general. You should not look at the power supply current to see what is going on with the motor current.
       <details><summary markdown="span">Ok so tell me how it actually works then...</summary><div markdown="block">
       The current in the motor is only connected to the current in the power supply _sometimes_ and other times it just cycles out of one phase and back in the other. This is what the modulation magnitude is (sometimes people call this duty cycle, but that's a bit confusing because we use SVM not straight PWM). When the modulation magnitude is 0, the average voltage seen across the motor phases is 0, and the motor current is never connected to the power supply. When the magnitude is 100%, it is always connected, and at 50% it's connected half the time, and cycled in just the motor half the time.

       The largest effect on modulation magnitude is speed. There are other smaller factors, but in general: if the motor is still it's not unreasonable to have 50A in the motor from 5A on the power supply. When the motor is spinning close to top speed, the power supply current and the motor current will be somewhat close to each other.
       </div></details>
   * The velocity limit: `odrv0.axis0.controller.config.vel_limit` [counts/s]. The motor will be limited to this speed; again the default value is quite slow.
   * You can change `odrv0.axis0.motor.config.calibration_current` [A] to the largest value you feel comfortable leaving running through the motor continously when the motor is stationary.

2. Set other hardware parameters:

   * `odrv0.config.brake_resistance` [Ohm]: This is the resistance of the brake resistor. If you are not using it, you may set it to `0`.
   * `odrv0.axis0.motor.config.pole_pairs`: This is the number of **magnet poles** in the rotor, **divided by two**. You can simply count the number of permanent magnets in the rotor, if you can see them. _Note: this is not the same as the number of coils in the stator._
   * `odrv0.axis0.motor.config.motor_type`: This is the type of motor being used. Currently two types of motors are supported: High-current motors (`MOTOR_TYPE_HIGH_CURRENT`) and Gimbal motors (`MOTOR_TYPE_GIMBAL`).

     <details><summary markdown="span">Which `motor_type` to choose?</summary><div markdown="block">

     If you're using a regular hobby brushless motor like [this](https://hobbyking.com/en_us/turnigy-aerodrive-sk3-5065-236kv-brushless-outrunner-motor.html) one, you should set `motor_mode` to `MOTOR_TYPE_HIGH_CURRENT`. For low-current gimbal motors like [this](https://hobbyking.com/en_us/turnigy-hd-5208-brushless-gimbal-motor-bldc.html) one, you should choose `MOTOR_TYPE_GIMBAL`. Do not use `MOTOR_TYPE_GIMBAL` on a motor that is not a gimbal motor, as it may overheat the motor or the ODrive.

     **Further detail:**
     If 100's of mA of current noise is "small" for you, you can choose `MOTOR_TYPE_HIGH_CURRENT`.
     If 100's of mA of current noise is "large" for you, and you do not intend to spin the motor very fast (omega * L << R), and the motor is fairly large resistance (1 ohm or larger), you can chose `MOTOR_TYPE_GIMBAL`.
     If 100's of mA current noise is "large" for you, _and_ you intend to spin the motor fast, then you need to replace the shunt resistors on the ODrive.

     </div></details>

   * `odrv0.axis0.encoder.config.cpr`: Encoder Count Per Revolution (CPR). This is 4x the Pulse Per Revolution (PPR) value. Usually this is indicated in the datasheet of your encoder.

3. Save configuration. You can save all `.config` parameters to persistent memory such that the ODrive remembers them between power cycles.
* `odrv0.save_configuration()` <kbd>Enter</kbd>

## Position control of M0

Let's get motor 0 up and running. The procedure for motor 1 is exactly the same, so feel free to replace read "axis1" wherever it says "axis0".

1. Type `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE` <kbd>Enter</kbd>. After about 2 seconds should hear a beep. Then the motor will turn slowly in one direction for a few seconds, then back in the other direction.

   <details><summary markdown="span">What's the point of this?</summary><div markdown="block">
   This procedure first measures your motor's electrical properties (namely phase resistance and phase inductance) and then the offset between the motor's electrical phase and the encoder position.
   </div></details>

   The startup procedure is demonstrated [here](https://www.youtube.com/watch?v=VCX1bA2xnuY).

   **Note**: the rotor must be allowed to rotate without any biased load during startup. That means mass and weak friction loads are fine, but gravity or spring loads are not okay. Also note that in the video, the motors spin after initalisation, but in the current software the default behaviour is not like that.

   <details><summary markdown="span">My motor doesn't beep or doesn't turn</summary><div markdown="block">
   Make sure the motor wires are connected firmly. Check the value of `odrv0.axis0.error` and then refer to the [error code documentation](troubleshooting.md#error-codes) for details.

   Once you have understood the error and fixed its cause, you may clear the error state (`odrv0.axis0.error = 0` <kbd>Enter</kbd>) and retry. You may also need to clear the error state of other subcomponents (e.g. `odrv0.axis0.motor.error`).
   </div></details>

2. Type `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL` <kbd>Enter</kbd>. From now on the ODrive will try to hold the motor's position. If you try to turn it by hand, it will fight you gently. That is unless you bump up `odrv0.axis0.motor.config.current_lim`, in which case it will fight you more fiercely.
3. Send the motor a new position setpoint. `odrv0.axis0.controller.pos_setpoint = 10000` <kbd>Enter</kbd>. The units are in encoder counts.

### Other control modes
The ODrive also supports velocity control and current (torque) control.
* **Velocity control**: Set `odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL`. You can now control the velocity with `odrv0.axis0.controller.vel_setpoint = 5000`. Units are counts/s.
* **Current control**: Set `odrv0.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL`. You can now control the current with `odrv0.axis0.controller.vel_setpoint = 3`. Units are A. **NOTE**: There is no velocity limiting in current control mode. Make sure that you don't overrev the motor, or exceed the max speed for your encoder.

## What's next?

You can now:

 * See what other [commands and parameters](commands.md) are available, including setting tuning parameters for better performance.
 * Control the ODrive from your own program or hook it up to an existing system through one of it's [interfaces](interfaces).
 * See how you can improve the behavior during the startup procedure, like [bypassing encoder calibration](encoders.md#encoder-with-index-signal).

If you have any issues or any questions please get in touch. The [ODrive Community](https://discourse.odriverobotics.com/) warmly welcomes you.
