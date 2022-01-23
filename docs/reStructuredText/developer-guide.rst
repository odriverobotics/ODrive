.. _developer-guide-doc:

================================================================================
ODrive Firmware Developer Guide
================================================================================

.. contents::
   :depth: 1
   :local:


This guide is intended for developers who wish to modify the firmware of the ODrive.
As such it assumes that you know things like how to use Git, what a compiler is, etc. If that sounds scary, turn around now.

The official releases are maintained on the `master` branch. However since you are a developer, you are encouraged to use the `devel` branch, as it contains the latest features.

The project is under active development, so make sure to check the `Changelog <https://github.com/odriverobotics/ODrive/tree/master/CHANGELOG.md>`_ to keep track of updates.

.. _dev-prereq:

Prerequisites
-------------------------------------------------------------------------------

The recommended tools for ODrive development are:

 * **make**: Used to invoke tup
 * **Tup**: The build system used to invoke the compile commands
 * **ARM GNU Compiler**: For cross-compiling code
 * **ARM GDB**: For debugging the code and stepping through on the device
 * **OpenOCD**: For flashing the ODrive with the STLink/v2 programmer
 * **Python 3**, along with the packages :code:`PyYAML`, :code:`Jinja2` and :code:`jsonschema`: For running the Python tools (:code:`odrivetool`). Also required for compiling firmware.

See below for specific installation instructions for your OS.

Depending on what you're gonna do, you may not need all of the components.

Once you have everything, you can verify the correct installation by running:

.. code:: Bash

  arm-none-eabi-gcc --version
  arm-none-eabi-gdb --version
  openocd --version             # should be 0.10.0 or later
  tup --version                 # should be 0.7.5 or later
  python --version              # should be 3.7 or later

**Installing Prerequisites**

.. tabs::
  .. tab:: Linux (Ubuntu < 20.04)

    .. code:: Bash
      
      sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa

    .. code:: Bash
      
      sudo apt-get update

    .. code:: Bash
      
      sudo apt-get install gcc-arm-embedded

    .. code:: Bash
      
      sudo apt-get install openocd

    .. code:: Bash
      
      sudo apt-get install git-lfs

    .. code:: Bash
      
      sudo add-apt-repository ppa:jonathonf/tup && sudo apt-get update && sudo apt-get install tup

    .. code:: Bash
      
      sudo apt-get install python3 python3-yaml python3-jinja2 python3-jsonschema

  .. tab:: Linux (Ubuntu >= 20.04)

    .. code:: Bash
      
      sudo apt install gcc-arm-none-eabi

    .. code:: Bash
      
      sudo apt install openocd

    .. code:: Bash
      
      sudo apt install git-lfs

    .. code:: Bash
      
      sudo apt install tup

    .. code:: Bash
      
      sudo apt install python3 python3-yaml python3-jinja2 python3-jsonschema

  .. tab:: Arch Linux

    .. code:: Bash
      
      sudo pacman -S arm-none-eabi-gcc arm-none-eabi-binutils

    .. code:: Bash
      
      sudo pacman -S arm-none-eabi-gdb

    .. code:: Bash
      
      sudo pacman -S git-lfs

    .. code:: Bash
      
      sudo pacman -S tup

    .. code:: Bash
      
      sudo pacman -S python python-yaml python-jinja python-jsonschema

    * `OpenOCD AUR package <https://aur.archlinux.org/packages/openocd/>`__

  .. tab:: Mac
    First install `Homebrew <https://brew.sh/>`__ Then you can run these commands in Terminal:

    .. code:: Bash
      
      brew install armmbed/formulae/arm-none-eabi-gcc

    .. code:: Bash
      
      brew install --cask osxfuse && brew install tup

    .. code:: Bash
      
      brew install openocd

    .. code:: Bash
      
      brew install git-lfs

    .. code:: Bash
      
      pip3 install PyYAML Jinja2 jsonschema

  .. tab:: Windows

    .. note:: make sure these programs are not only installed but also added to your `PATH`.

    Some instructions in this document may assume that you're using a bash command prompt, such as the Windows 10 built-in bash or `Git <https://git-scm.com/download/win>`__ bash.

    * `ARM compiler <https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads>`__

      .. note:: 

        After installing, create an environment variable named `ARM_GCC_ROOT` whose value is the path you installed to.  e.g. :code:`C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update`.  
        This variable is used to locate include files for the c/c++ Visual Studio Code extension.

      .. note:: 8-2018-q4-major seems to have a bug on Windows.  Please use 7-2018-q2-update.
    * `Tup <http://gittup.org/tup/index.html>`__
    * `GNU MCU Eclipse's Windows Build Tools <https://github.com/gnu-mcu-eclipse/windows-build-tools/releases>`__
    * `Python 3 <https://www.python.org/downloads/>`__
      * Install Python packages: :code:`pip install PyYAML Jinja2 jsonschema`
    * `OpenOCD <https://github.com/xpack-dev-tools/openocd-xpack/releases/>`__
    * `ST-Link/V2 Drivers <https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html>`__


Configuring the Build
--------------------------------------------------------------------------------

To customize the compile time parameters, copy or rename the file :code:`Firmware/tup.config.default` to :code:`Firmware/tup.config` and edit the following parameters in that file:

* **CONFIG_BOARD_VERSION** The board version you're using. Can be `v3.1`, `v3.2`, `v3.3`, `v3.4-24V`, `v3.4-48V`, `v3.5-24V`, `v3.5-48V`, etc. Check for a label on the upper side of the ODrive to find out which version you have. 
  Some ODrive versions don't specify the voltage: in that case you can read the value of the main capacitors: 120uF are 48V ODrives, 470uF are 24V ODrives.

* **CONFIG_DEBUG** Defines whether debugging will be enabled when compiling the firmware; specifically the :code:`-g -gdwarf-2` flags. 
  Note that printf debugging will only function if your tup.config specifies the :code:`USB_PROTOCOL` or :code:`UART_PROTOCOL` as stdout and :code:`DEBUG_PRINT` is defined. 
  See the IDE specific documentation for more information.

You can also modify the compile-time defaults for all :code:`.config` parameters. 
You will find them if you search for :code:`AxisConfig`, :code:`MotorConfig`, etc.

.. _build-and-flash:

Building and Flashing the Firmware
--------------------------------------------------------------------------------

#. Run :code:`make` in the :code:`Firmware` directory.
#. Connect the ODrive via USB and power it up.
#. Flash the firmware using :ref:`odrivetool dfu <firmware-update>`.

.. _flashing-with-an-stlink: 

Flashing using an STLink/v2 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Connect `GND`, `SWD`, and `SWC` on connector J2 to the programmer. 

  .. note:: Always plug in `GND` first!

* You need to power the board by only **ONE** of the following: VCC(3.3v), 5V, or the main power connection (the DC bus). The USB port (J1) does not power the board.
* Run :code:`make flash` in the :code:`Firmware` directory.

.. note:: If you receive the error `can't find target interface/stlink-v2.cfg` or similar, create and set an environment variable named :code:`OPENOCD_SCRIPTS` to the location of the openocd scripts directory.

If the flashing worked, you can connect to the board using the :ref:`odrivetool <odrivetool-startup>`.

Testing
--------------------------------------------------------------------------------

.. include:: testing.rst

Debugging
--------------------------------------------------------------------------------

If you're using VSCode, make sure you have the Cortex Debug extension, OpenOCD, and the STLink.  
You can verify that OpenOCD and STLink are working by ensuring you can flash code.  
Open the ODrive_Workspace.code-workspace file, and start a debugging session (F5).  
VSCode will pick up the correct settings from the workspace and automatically connect.  
Breakpoints can be added graphically in VSCode.

* Run :code:`make gdb`. This will reset and halt at program start. Now you can set breakpoints and run the program. If you know how to use gdb, you are good to go.

Setting up an IDE
--------------------------------------------------------------------------------

For working with the ODrive code you don't need an IDE, but the open-source IDE VSCode is recommended.  
It is also possible to use Eclipse. If you'd like to go that route, please see the respective configuration document:

* :ref:`Configuring VSCode <configuring-vscode>`
* :ref:`Configuring Eclipse <configuring-eclipse>`


STM32CubeMX
--------------------------------------------------------------------------------

This project uses the STM32CubeMX tool to generate startup code and to ease the configuration of the peripherals. 
You can download it from `here <http://www2.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stm32cubemx.html?icmp=stm32cubemx_pron_pr-stm32cubef2_apr2014&sc=stm32cube-pr2>`___. 
All CubeMX related files are in :code:`Firmware/Board/v3`.

You will likely want the pinout for this process. It is available `here <https://docs.google.com/spreadsheets/d/1QXDCs1IRtUyG__M_9WruWOheywb-GhOwFtfPcHuN2Fg/edit#gid=404444347>`__.

Maintaining Modified Generated Code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When generating the code, STM32CubeMX will nuke everything except some special sections that they provide. 
These sections are marked like `USER CODE BEGIN`...`USER CODE END`.
We used to try to make sure all edits we made to the generated code would only go in these sections, so some code structrure may reflect that.
However over time we realized this will not be tenable, so instead we use git to rebase all changes of the generated code whenever we need to regenerate it.
We use two special branches that will help us to do this, they are :code:`STM32CubeMX-start` and :code:`STM32CubeMX-end`.
How to use these is shown in the following example.

.. note:: 
  Due to how this rebasing is done, all development that changes the generated code should be done directly on :code:`STM32CubeMX-end`, and not based on :code:`devel`, then follow step 4 below to carry them over to your feature branch. If you did some changes to the generated code based from :code:`devel`, you need to cherry pick just those changes over to :code:`STM32CubeMX-end`.

#. **Ensuring a clean slate**

  * We do all changes to the STM32CubeMX config and regenerate the code on top of :code:`STM32CubeMX-start`.
    * :code:`git checkout STM32CubeMX-start`

  * Run stm32cubeMX and load the :code:`Firmware/Board/v3/Odrive.ioc` project file.
    * If the tool asks if you wish to migrate to a new version, choose to download the old firmware package (unless you want to use the latest libraries)
  
  * Without changing any settings, press :code:`Project -> Generate code`.
  * You may need to let it download some drivers and such.
  * STM32CubeMX may now have a newer version of some of the libraries, so there may be changes to the generated code even though we didn't change any settings. We need to check that everything is still working, and hence check in the changes:
  * :code:`git config --local core.autocrlf input` - This will tell git that all files should be checked in with LF endings (CubeMX generates CRLF endings).
  * :code:`git diff` - Ignore the pile of line ending warnings.
  * If you feel qualified: you can now ispect if CubeMX introduced something stupid. If there were any changes, and they look acceptable, we should commit them:
    
    * :code:`git commit -am "Run STM32CubeMX v1.21"` - Replace with actual version of CubeMX

#. **Making Changes to the STM32CubeMX Config**

  * After completing the above steps, make sure the working directory is clean:
    * :code:`git status` should include "nothing to commit, working tree clean"

  * Make your changes in STM32CubeMX, save the project and generate the code. (:code:`Project -> Generate code`)
  * :code:`git diff` - Check that the introduced changes are as expected
  * If everything looks ok, you can commit your changes.

#. **Rebasing the Modifications to the Generated Code**

  * :code:`git checkout STM32CubeMX-end`
  * :code:`git rebase STM32CubeMX-start`
  * Make sure the rebase finishes, fixing any conflicts that may arise

#. **Merge New STM32CubeMX Code to your Feature Branch**

  Simply merge the new state at:code: `STM32CubeMX-end` into your feature branch.
  * :code:`git checkout your-feature`
  * :code:`git merge STM32CubeMX-end`

#. **Pushing back Upstream**

  * Generate a PR like normal for your feature.
  * Make sure youhave pushed to the :code:`STM32CubeMX-start` and :code:`STM32CubeMX-end` branches on your fork.
  * Make a note in your PR to the maintainer that they need to update the STM32CubeMX branches when they merge the PR.

Troubleshooting
--------------------------------------------------------------------------------

:code:`LIBUSB_ERROR_IO` when flashing with the STLink/v2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Problem:** when I try to flash the ODrive with the STLink using :code:`make flash` I get this error:

.. code:: Bash

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

**Solution:**
This happens from time to time.

#. Unplug the STLink and all ODrives from your computer
#. Power off the ODrive you're trying to flash
#. Plug in the STLink into your computer
#. Power on the ODrive
#. Run :code:`make flash` again

:code:`Cannot identify target as a STM32 family` when flashing using openocd
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Problem:** When I try to flash ODrive v4.1 with :code:`make flash` then I get:

.. code:: Bash

  [...]
  ** Programming Started **
  auto erase enabled
  Info : device id = 0x10006452
  Warn : Cannot identify target as a STM32 family.
  Error: auto_probe failed
  embedded:startup.tcl:487: Error: ** Programming Failed **
  in procedure 'program' 
  in procedure 'program_error' called at file "embedded:startup.tcl", line 543
  at file "embedded:startup.tcl", line 487


**Solution:**
Compile and install a recent version of openocd from source. 
The latest official release (0.10.0 as of Nov 2020) doesn't support the STM32F722 yet.

.. code:: Bash

  sudo apt-get install libtool libusb-1.0
  git clone https://git.code.sf.net/p/openocd/code openocd
  cd openocd/
  ./bootstrap
  ./configure --enable-stlink
  make
  sudo make install


Documentation
--------------------------------------------------------------------------------
.. admonition:: In Progress 

  ODrive documentation is now built using `Sphinx <https://www.sphinx-doc.org/en/master/>`_.

  * Prerequisites:

    * Install Sphinx: :code:`pip install -U sphinx`
    * Install packages: :code:`pip install sphinx-copybutton sphinx-panels sphinx-rtd-theme`


  * Run :code:`make html` within the :kbd:`./docs/reStructuredText` folder 
  * Open :kbd:`./docs/reStructuredText/_build/index.html` to view

.. _modifying-libfibre:

Modifying libfibre
--------------------------------------------------------------------------------

If you need to modify libfibre add :code:`CONFIG_BUILD_LIBFIBRE=true` to your tup.config and rerun :code:`make`. 
After this you can start :code:`odrivetool` (on your local PC) and it will use the updated libfibre.

To cross-compile libfibre for the Raspberry Pi, run :code:`make libfibre-linux-armhf` or :code:`make libfibre-all`. 
This will require a docker container. See ;:ref:`fibre-cpp readme <../Firmware/fibre-cpp/README.md>` for details.

.. code:: Bash

  docker run -it -v "$(pwd)":/build -v /tmp/build:/build/build -w /build fibre-compiler configs/linux-armhf.config

If you're satisfied with the changes don't forget to generate binaries for all
supported systems using :code:`make libfibre-all`.

Releases
--------------------------------------------------------------------------------

We use GitHub Releases to provide firmware releases.

#. Cut off the changelog to reflect the new release
#. Merge the release candidate into master.
#. Push a (lightweight) tag to the master branch. Follow the existing naming convention.
#. If you changed something in libfibre, regenerate the binaries using :code:`make libfibre-all`. 
   See :ref:`Modifying libfibre <modifying-libfibre>` for details.
#. Push the python tools to PyPI (see setup.py for details).
#. Edit the release on GitHub to add a title and description (copy&paste from changelog).

Code Maintenance Notes
--------------------------------------------------------------------------------

The cortex M4F processor has hardware single precision float unit. However double precision operations are not accelerated, and hence should be avoided. 
The following regex is helpful for cleaning out double constants:

find: 

.. code::

  ([-+]?[0-9]+\.[0-9]+(?:[eE][-+]?[0-9]+)?)([^f0-9e])

replace: 

  .. code::

    \1f\2

Notes for Contributors
--------------------------------------------------------------------------------

In general the project uses the `Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`__ with a few exceptions:

 * The default indentation is 4 spaces.
 * The 80 character limit is not very strictly enforced, merely encouraged.
 * The file extensions `*.cpp` and `*.hpp` are used instead of `*.cc` and `*.h`.

Your help is welcome! However before you start working on a feature/change that will take you a non-negligible amount of time and that you plan to upstream please discuss your plans with us on GitHub or Discord. 
This will ensure that your implementation is in line with the direction that ODrive is going.

When filing a PR please go through this checklist:

 * Make sure you adhere to the same coding style that we use (see note above).
 * Update CHANGELOG.md.
 * If you removed/moved/renamed things in :code:`odrive-interface.yaml` make sure to add corresponding bullet points tp the "API migration notes" section in the changelog. Use git to compare against the :code:`devel` branch.
 * Also, for each removed/moved/renamed API item use your IDE's search feature to search for occurrences of this name. Update the places you found (this will usually be documentation and test scripts).
 * If you added things to :code:`odrive-interface.yaml` make sure the new things have decent documentation in the YAML file. We don't expect 100% coverage but use good sense of what to document.
 * Make sure your PR doesn't contain spurious changes that unnecessarily add or remove whitespace. These add noise and make the reviewer's lifes harder.
 * If you changed any enums in :code:`odrive-interface.yaml`, make sure you update `enums.py <https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py>`_ and `ODriveEnums.h <https://github.com/odriverobotics/ODrive/blob/master/Arduino/ODriveArduino/ODriveEnums.h>`_. 
   The file includes instructions on how to do this. Check the diff to verify that none of the existing enumerators changed their value.
