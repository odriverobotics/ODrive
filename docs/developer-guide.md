# ODrive Firmware Developer Guide

This guide is intended for developers who wish to modify the firmware of the ODrive.
As such it assumes that you know things like how to use Git, what a compiler is, etc. If that sounds scary, turn around now.

The official releases are maintained on the `master` branch. However since you are a developer, you are encouraged to use the `devel` branch, as it contains the latest features.

The project is under active development, so make sure to check the [Changelog](../CHANGELOG.md) to keep track of updates.

### Table of contents

<!-- MarkdownTOC depth=2 autolink=true bracket=round -->

- [Prerequisites](#prerequisites)
- [Configuring the build](#configuring-the-build)
- [Building and flashing the Firmware](#building-and-flashing-the-firmware)
- [Debugging](#debugging)
- [Testing](#testing)
- [Setting up an IDE](#setting-up-an-ide)
- [STM32CubeMX](#stm32cubemx)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)
- [Releases](#releases)
- [Notes for Contributors](#notes-for-contributors)

<!-- /MarkdownTOC -->

<br><br>
## Prerequisites

The recommended tools for ODrive development are:

 * **make**: Used to invoke tup
 * **Tup**: The build system used to invoke the compile commands
 * **ARM GNU Compiler**: For cross-compiling code
 * **ARM GDB**: For debugging the code and stepping through on the device
 * **OpenOCD**: For flashing the ODrive with the STLink/v2 programmer
 * **Python**: For running the Python tools

See below for specific installation instructions for your OS.

Depending on what you're gonna do, you may not need all of the components.

Once you have everything, you can verify the correct installation by running:
```bash
$ arm-none-eabi-gcc --version
$ arm-none-eabi-gdb --version
$ openocd --version             # should be 0.10.0 or later
$ tup --version                 # should be 0.7.5 or later
$ python --version              # should be 3.7 or later
```

#### Linux (Ubuntu)
```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
sudo apt-get install openocd
sudo add-apt-repository ppa:jonathonf/tup && sudo apt-get update && sudo apt-get install tup
```

#### Arch Linux
```bash
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-binutils
sudo pacman -S arm-none-eabi-gdb
sudo pacman -S tup
```
* [OpenOCD AUR package](https://aur.archlinux.org/packages/openocd/)

#### Mac
First install [Homebrew](https://brew.sh/). Then you can run these commands in Terminal:
```bash
brew install armmbed/formulae/arm-none-eabi-gcc
brew cask install osxfuse && brew install tup
brew install openocd
```

#### Windows
__Note__: make sure these programs are not only installed but also added to your `PATH`.

Some instructions in this document may assume that you're using a bash command prompt, such as the Windows 10 built-in bash or [Git](https://git-scm.com/download/win) bash.

* [ARM compiler](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)  
  * __Note 1__: After installing, create an environment variable named `ARM_GCC_ROOT` whose value is the path you installed to.  e.g. `C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update`.  This variable is used to locate include files for the c/c++ Visual Studio Code extension.
  * __Note 2__: 8-2018-q4-major seems to have a bug on Windows.  Please use 7-2018-q2-update.
* [Tup](http://gittup.org/tup/index.html)
* [GNU MCU Eclipse's Windows Build Tools](https://github.com/gnu-mcu-eclipse/windows-build-tools/releases)
* [OpenOCD](http://gnuarmeclipse.github.io/openocd/install/). 
* [ST-Link/V2 Drivers](http://www.st.com/web/en/catalog/tools/FM147/SC1887/PF260219)

<br>

## Configuring the build

To customize the compile time parameters, copy or rename the file `Firmware/tup.config.default` to `Firmware/tup.config` and edit the parameters in that file:

__CONFIG_BOARD_VERSION__: The board version you're using. Can be `v3.1`, `v3.2`, `v3.3`, `v3.4-24V`, `v3.4-48V`, `v3.5-24V`, `v3.5-48V`, etc. Check for a label on the upper side of the ODrive to find out which version you have. Some ODrive versions don't specify the voltage: in that case you can read the value of the main capacitors: 120uF are 48V ODrives, 470uF are 24V ODrives.

__CONFIG_USB_PROTOCOL__: Defines which protocol the ODrive should use on the USB interface.
 * `native`: The native ODrive protocol. Use this if you want to use the python tools in this repo. Can maybe work with macOS.
 * `native-stream`: Like the native ODrive protocol, but the ODrive will treat the USB connection exactly as if it was a UART connection. __ Maybe need to use this if you're on macOS__. This is necessary because macOS doesn't grant our python tools sufficient low-level access to treat the device as the USB device that it is.
 * `none`: Disable USB. The device will still show up when plugged in but it will ignore any commands.
 
 **Note**: There is a second USB interface that is always a serial port.

__CONFIG_UART_PROTOCOL__: Defines which protocol the ODrive should use on the UART interface (GPIO1 and GPIO2). Note that UART is only supported on ODrive v3.3 and higher.
 * `native`: The native ODrive protocol. Use this if you're connecting the ODrive to a PC using UART and want to use the python tools to control and setup the ODrive.
 * `ascii`: The ASCII protocol. Use this option if you control the ODrive with an Arduino. The ODrive Arduino library is not yet updated to the native protocol.
 * `none`: Disable UART.

You can also modify the compile-time defaults for all `.config` parameters. You will find them if you search for `AxisConfig`, `MotorConfig`, etc.

<br><br>
## Building and flashing the Firmware

1. Run `make` in the `Firmware` directory.
2. Connect the ODrive via USB and power it up.
3. Flash the firmware using [odrivetool dfu](odrivetool#device-firmware-update).

### Flashing using an STLink/v2 programmer

* Connect `GND`, `SWD`, and `SWC` on connector J2 to the programmer. Note: Always plug in `GND` first!
* You need to power the board by only **ONE** of the following: VCC(3.3v), 5V, or the main power connection (the DC bus). The USB port (J1) does not power the board.
* Run `make flash` in the `Firmware` directory.
__Note__: If you receive the error `can't find target interface/stlink-v2.cfg` or similar, create and set an environment variable named `OPENOCD_SCRIPTS` to the location of the openocd scripts directory.

If the flashing worked, you can connect to the board using the [odrivetool](getting-started#start-odrivetool).

<br><br>
## Testing
The script `tools/run_tests.py` runs a sequence of automated tests for several firmware features as well as high power burn-in tests. Some tests only need one ODrive and one motor/encoder pair while other tests need a back-to-back test rig such as [this one](https://cad.onshape.com/documents/026bda35ad5dff4d73c1d37f/w/ae302174f402737e1fdb3783/e/5ca143a6e5e24daf1fe8e434). In any case, to run the tests you need to provide a YAML file that lists the parameters of your test setup. An example can be found at [`tools/test-rig-parallel.yaml`](tools/test-rig-parallel.yaml`). The programmer serial number can be found by running `Firmware/find_programmer.sh` (make sure it has the latest formware from STM).

<div class="alert" markdown="span">The test script commands the ODrive to high currents and high motor speeds so if your ODrive is connected to anything other than a stirdy test-rig (or free spinning motors), it will probably break your machine.</div>

Example usage: `./run_tests.py --test-rig-yaml ../tools/test-rig-parallel.yaml`

<br><br>
## Debugging
If you're using VSCode, make sure you have the Cortex Debug extension, OpenOCD, and the STLink.  You can verify that OpenOCD and STLink are working by ensuring you can flash code.  Open the ODrive_Workspace.code-workspace file, and start a debugging session (F5).  VSCode will pick up the correct settings from the workspace and automatically connect.  Breakpoints can be added graphically in VSCode.

* Run `make gdb`. This will reset and halt at program start. Now you can set breakpoints and run the program. If you know how to use gdb, you are good to go.

<br><br>
## Setting up an IDE
For working with the ODrive code you don't need an IDE, but the open-source IDE VSCode is recommended.  It is also possible to use Eclipse. If you'd like to go that route, please see the respective configuration document:

* [Configuring VSCode](configuring-vscode.md)
* [Configuring Eclipse](configuring-eclipse.md)

<br><br>
## STM32CubeMX
This project uses the STM32CubeMX tool to generate startup code and to ease the configuration of the peripherals. You can download it from [here](http://www2.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stm32cubemx.html?icmp=stm32cubemx_pron_pr-stm32cubef2_apr2014&sc=stm32cube-pr2). All CubeMX related files are in `Firmware/Board/v3`.

You will likely want the pinout for this process. It is available [here](https://docs.google.com/spreadsheets/d/1QXDCs1IRtUyG__M_9WruWOheywb-GhOwFtfPcHuN2Fg/edit#gid=404444347).

### Maintaining modified generated code
When generating the code, STM32CubeMX will nuke everything except some special sections that they provide. These sections are marked like `USER CODE BEGIN`...`USER CODE END`.
We used to try to make sure all edits we made to the generated code would only go in these sections, so some code structrure may reflect that.
However over time we realized this will not be tenable, so instead we use git to rebase all changes of the generated code whenever we need to regenerate it.
We use two special branches that will help us to do this, they are `STM32CubeMX-start` and `STM32CubeMX-end`.
How to use these is shown in the following example.

**Note**: Due to how this rebasing is done, all development that changes the generated code should be done directly on `STM32CubeMX-end`, and not based on `devel`, then follow step 4 below to carry them over to your feature branch. If you did some changes to the generated code based from `devel`, you need to cherry pick just those changes over to `STM32CubeMX-end`.

### 1. Ensuring a clean slate
* We do all changes to the STM32CubeMX config and regenerate the code on top of `STM32CubeMX-start`.
  * `git checkout STM32CubeMX-start`
* Run stm32cubeMX and load the `Firmware/Board/v3/Odrive.ioc` project file.
  * If the tool asks if you wish to migrate to a new version, choose to migrate.
* Without changing any settings, press `Project -> Generate code`.
* You may need to let it download some drivers and such.
* STM32CubeMX may now have a newer version of some of the libraries, so there may be changes to the generated code even though we didn't change any settings. We need to check that everything is still working, and hence check in the changes:
* `git config --local core.autocrlf input` - This will tell git that all files should be checked in with LF endings (CubeMX generates CRLF endings).
* `git diff` - Ignore the pile of line ending warnings.
* If you feel qualified: you can now ispect if CubeMX introduced something stupid. If there were any changes, and they look acceptable, we should commit them:
  * `git commit -am "Run STM32CubeMX v1.21"` - Replace with actual version of CubeMX

### 2. Making changes to the STM32CubeMX config
* After completing the above steps, make sure the working directory is clean:
  * `git status` should include "nothing to commit, working tree clean"
* Make your changes in STM32CubeMX, save the project and generate the code. (`Project -> Generate code`)
* `git diff` - Check that the introduced changes are as expected
* If everything looks ok, you can commit your changes.

### 3. Rebasing the modifications to the generated code
* `git checkout STM32CubeMX-end`
* `git rebase STM32CubeMX-start`
* Make sure the rebase finishes, fixing any conflicts that may arise

### 4. Merge new STM32CubeMX code to your feature branch
Simply merge the new state at `STM32CubeMX-end` into your feature branch.
* `git checkout your-feature`
* `git merge STM32CubeMX-end`

### 5. Pushing back upstream
* Generate a PR like normal for your feature.
* Make sure youhave pushed to the `STM32CubeMX-start` and `STM32CubeMX-end` branches on your fork.
* Make a note in your PR to the maintainer that they need to update the STM32CubeMX branches when they merge the PR.

<br><br>
## Troubleshooting

### `LIBUSB_ERROR_IO` when flashing with the STLink/v2

**Problem:** when I try to flash the ODrive with the STLink using `make flash` I get this error:
```
Open On-Chip Debugger 0.10.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
adapter speed: 2000 kHz
adapter_nsrst_delay: 100
none separate
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : clock speed 1800 kHz
Error: libusb_open() failed with LIBUSB_ERROR_IO
Error: open failed
in procedure 'init' 
in procedure 'ocd_bouncer'
```

**Solution:**
This happens from time to time.
1. Unplug the STLink and all ODrives from your computer
2. Power off the ODrive you're trying to flash
3. Plug in the STLink into your computer
4. Power on the ODrive
5. Run `make flash` again

## Documentation

All *.md files in the `docs/` directory of the master branch are served up by GitHub Pages on [this domain](https://docs.odriverobotics.com).

 * Theme: [minimal](https://github.com/pages-themes/minimal) by [orderedlist](https://github.com/orderedlist)
 * HTML layout: `docs/_layouts/default.html`
 * CSS style: `docs/assets/css/styles.scss`
 * Site index: `docs/_data/index.yaml`

To run the docs server locally:

```bash
cd docs
gem install bundler
bundle install --path ruby-bundle
bundle exec jekyll serve --host=0.0.0.0
```

## Releases

We use GitHub Releases to provide firmware releases.

1. Cut off the changelog to reflect the new release
2. Merge the release candidate into master.
3. Push a (lightweight) tag to the master branch. Follow the existing naming convention.
4. Push the python tools to PyPI.
5. Edit the release on GitHub to add a title and description (copy&paste from changelog).

## Other code maintenance notes
The cortex M4F processor has hardware single precision float unit. However double precision operations are not accelerated, and hence should be avoided. The following regex is helpful for cleaning out double constants:
find: `([-+]?[0-9]+\.[0-9]+(?:[eE][-+]?[0-9]+)?)([^f0-9e])`
replace: `\1f\2`

<br><br>
## Notes for Contributors
In general the project uses the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), except that the default indendtation is 4 spaces, and that the 80 character limit is not very strictly enforced, merely encouraged.
