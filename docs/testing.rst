
Automated Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. contents::
   :depth: 1
   :local:

This section describes how to use the automated testing facilities.
You don't have to do this as an end user.

The testing facility consists of the following components:

 * **Test rig:** In the simplest case this can be a single ODrive optionally with a single motor and encoder pair. Can also be multiple ODrives with multiple axes, some of which may be mechanically coupled.
 * **Test host:** The PC on which the test script runs. All ODrives must be connected to the test host via USB.
 * **test-rig.yaml:** Describes your test rig. Make sure all values are correct. Incorrect values may physically break or fry your test setup.
 * **test_runner.py:** This is the main script that runs all the tests.
 * **..._test.py** The actual tests

The Tests
********************************************************************************

 * :code:`analog_input_test.py`: Analog Input
 * :code:`calibration_test.py`: Motor calibration, encoder offset calibration, encoder direction find, encoder index search
 * :code:`can_test.py`: Partial coverage of the commands described in :ref:`CAN Protocol. <can-protocol>`
 * :code:`closed_loop_test.py`: Velocity control, position control (TODO: sensorless control), brake regen current hard limit, current control with velocity limiting
 * :code:`encoder_test.py`: Incremental encoder, hall effect encoder, sin/cos encoder, SPI encoders (AMS, CUI)
 * :code:`fibre_test.py`: General USB protocol tests
 * :code:`nvm_test.py`: Configuration storage
 * :code:`pwm_input_test.py`: PWM input
 * :code:`step_dir_test.py`: Step/dir input
 * :code:`uart_ascii_test.py`: Partial coverage of the commands described in :ref:`ASCII Protocol <ascii-protocol>`

All tests in a file can be run with e.g.:

.. code:: Bash

    python3 uart_ascii_test.py --test-rig-yaml ../../test-rig-rpi.yaml

See the following sections for a more detailed test flow description.

Our Test Rig
**************************************************************************

Our test rig essentially consists of the following components:

 * an ODrive as the test subject
 * a Teensy 4.0 to emulate external hardware such as encoders
 * a Motor + Encoder pair for closed loop control tests
 * a Raspberry Pi 4.0 as test host
 * a CAN hat for the Raspberry Pi for CAN tests

This document is therefore centered around this test rig layout.
If your test rig differs, you may be able to run some but not all of the tests.

How to set up a Raspberry Pi as testing host
**************************************************************************

#. Install Raspbian Lite on a Raspberry Pi 4.0. This is easiest if you have a keyboard, mouse and screen (micro-HDMI!). 
I used the `NOOBS Lite installer <https://www.raspberrypi.org/downloads/noobs/>`_ for this. Paste the ZIP-file's contents onto a FAT32 formatted SD card (fs type `0b` in `fdisk`) and boot it. Then follow the on-screen instructions.

#. Prepare the installation:
 
       .. code:: Bash

              sudo systemctl enable ssh
              sudo systemctl start ssh
              # Transfer your public key for passwordless SSH. All subsequent steps can be done via SSH.
              sudo apt-get update
              sudo apt-get upgrade
              # Change /etc/hostname to something meaningful

#. Add the following lines to :code:`/boot/config.txt`:

       * :code:`enable_uart=1`
       * :code:`dtparam=spi=on`
       * :code:`dtoverlay=spi-bcm2835-overlay`
       * :code:`dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25` 
              .. note:: These oscillator and interrupt GPIO settings here are for the "RS485 CAN HAT" I have. There appear to be multiple versions, so they may be different from yours. Check the marking on the oscillator and the schematics.

#. Remove the following arguments from :code:`/boot/cmdline.txt`:
       * :code:`console=serial0,115200`

#. Append :code:`ODRIVE_TEST_RIG_NAME=[test-rig-name]` to :code:`/etc/environment`. The HWIL tests use this to look up the the file :code:`[test-rig-name].yaml` which is supposed to describe your test rig.

#. Reboot.

#. Install the prerequisites:

       .. code:: Bash

              sudo apt-get install ipython3 python3-appdirs python3-yaml python3-jinja2 python3-usb python3-serial python3-can python3-scipy python3-matplotlib python3-ipdb git openocd
              # Optionally, to be able to compile the firmware:
              sudo apt-get install gcc-arm-none-eabi

#. Install Teensyduino and teensy-loader-cli:

       .. code:: Bash

              sudo apt-get install libfontconfig libxft2 libusb-dev

       .. code:: Bash

              wget https://downloads.arduino.cc/arduino-1.8.13-linuxarm.tar.xz

       .. code:: Bash

             tar -xf arduino-1.8.13-linuxarm.tar.xz

       .. code:: Bash

             wget https://www.pjrc.com/teensy/td_153/TeensyduinoInstall.linuxarm

       .. code:: Bash

             chmod +x TeensyduinoInstall.linuxarm

       .. code:: Bash

             ./TeensyduinoInstall.linuxarm --dir=arduino-1.8.13

       .. code:: Bash

             sudo cp -R arduino-1.8.13 /usr/share/arduino

       .. code:: Bash

             sudo ln -s /usr/share/arduino/arduino /usr/bin/arduino
        

       .. code:: Bash

             git clone https://github.com/PaulStoffregen/teensy_loader_cli

       .. code:: Bash

             pushd teensy_loader_cli

       .. code:: Bash

             make

       .. code:: Bash

             sudo cp teensy_loader_cli /usr/bin/

       .. code:: Bash

             sudo ln -s /usr/bin/teensy_loader_cli /usr/bin/teensy-loader-cli

       .. code:: Bash

             popd

       .. code:: Bash

             curl https://www.pjrc.com/teensy/49-teensy.rules | sudo tee /etc/udev/rules.d/49-teensy.rules

#. Add the following lines to :code:`/etc/udev/rules.d/49-stlinkv2.rules`:

       .. code:: Bash

              SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE:="0666"
              SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE:="0666"

#. :code:`sudo mkdir /opt/odrivetest && sudo chown $USER /opt/odrivetest`

#. At this point you need the ODrive repository. See next section to sync it from your main PC. We assume now that you navigated to `tools/odrive/tests/`.

#. :code:`sudo ../../odrivetool udev-setup`

#. `sudo udevadm trigger`

#. Run once after every reboot: :code:`sudo -E ipython3 --pdb test_runner.py -- --setup-host`

SSH Testing Flow
**************************************************************************

Here's one possible workflow for developing on the local host and testing on a remote SSH host.

We assume that the ODrive repo is at :code:`/path/to/ODriveFirmware` and your testing host is configured under the SSH name :code:`odrv`.

To flash and start remote debugging:

#. Start OpenOCD remotely, along with a tunnel to localhost: 

      .. code:: Bash
            
            ssh -t odrv -L3333:localhost:3333 bash -c "\"openocd '-f' 'interface/stlink-v2.cfg' '-f' 'target/stm32f4x_stlink.cfg'\""
      
      
      You can keep this open for multiple debug sessions. Press :kbd:`Ctrl` **+** :kbd:`C` to quit.

#. Compile the firmware.
#. In VSCode, select the run configuration "Debug ODrive v3.x/v4.x - Remote" and press Run. This will flash the new firmware before dropping you into the debugger.

To run a test:

       .. code:: Bash

              rsync -avh -e ssh /path/to/ODriveFirmware/ odrv:/opt/odrivetest --exclude="Firmware/build" --exclude="Firmware/.tup" --exclude=".git" --exclude="GUI" --delete

       .. code:: Bash

              ssh odrv

       .. code:: Bash

              cd /opt/odrivetest/tools/odrive/tests/
              ipython3 --pdb uart_ascii_test.py

