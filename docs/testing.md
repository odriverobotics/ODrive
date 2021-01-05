# Automated Testing

This section describes how to use the automated testing facilities.
You don't have to do this as an end user.

The testing facility consists of the following components:
 * **Test rig:** In the simplest case this can be a single ODrive optionally with a single motor and encoder pair. Can also be multiple ODrives with multiple axes, some of which may be mechanically coupled.
 * **Test host:** The PC on which the test script runs. All ODrives must be connected to the test host via USB.
 * **test-rig.yaml:** Describes your test rig. Make sure all values are correct. Incorrect values may physically break or fry your test setup.
 * **test_runner.py:** This is the main script that runs all the tests.
 * **..._test.py** The actual tests

## The Tests

 - `analog_input_test.py`: Analog Input
 - `calibration_test.py`: Motor calibration, encoder offset calibration, encoder direction find, encoder index search
 - `can_test.py`: Partial coverage of the commands described in [CAN Protocol](can-protocol)
 - `closed_loop_test.py`: Velocity control, position control (TODO: sensorless control), brake regen current hard limit, current control with velocity limiting
 - `encoder_test.py`: Incremental encoder, hall effect encoder, sin/cos encoder, SPI encoders (AMS, CUI)
 - `fibre_test.py`: General USB protocol tests
 - `nvm_test.py`: Configuration storage
 - `pwm_input_test.py`: PWM input
 - `step_dir_test.py`: Step/dir input
 - `uart_ascii_test.py`: Partial coverage of the commands described in [ASCII Protocol](ascii-protocol)

All tests in a file can be run with e.g.:

    python3 uart_ascii_test.py --test-rig-yaml ../../test-rig-rpi.yaml

See the following sections for a more detailed test flow description.

## Our test rig

Our test rig essentially consists of the following components:

 - an ODrive as the test subject
 - a Teensy 4.0 to emulate external hardware such as encoders
 - a Motor + Encoder pair for closed loop control tests
 - a Raspberry Pi 4.0 as test host
 - a CAN hat for the Raspberry Pi for CAN tests

This document is therefore centered around this test rig layout.
If your test rig differs, you may be able to run some but not all of the tests.

## How to set up a Raspberry Pi as testing host

 1. Install Raspbian Lite on a Raspberry Pi 4.0. This is easiest if you have a keyboard, mouse and screen (micro-HDMI!). I used the [NOOBS Lite installer](https://www.raspberrypi.org/downloads/noobs/) for this. Paste the ZIP-file's contents onto a FAT32 formatted SD card (fs type `0b` in `fdisk`) and boot it. Then follow the on-screen instructions.
 2. Prepare the installation:
 
        sudo systemctl enable ssh
        sudo systemctl start ssh
        # Transfer your public key for passwordless SSH. All subsequent steps can be done via SSH.
        sudo apt-get update
        sudo apt-get upgrade
        # Change /etc/hostname to something meaningful

 3. Add the following lines to `/boot/config.txt`:
    - `enable_uart=1`
    - `dtparam=spi=on`
    - `dtoverlay=spi-bcm2835-overlay`
    - `dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25` - Note: These oscillator and interrupt GPIO settings here are for the "RS485 CAN HAT" I have. There appear to be multiple versions, so they may be different from yours. Check the marking on the oscillator and the schematics.

 4. Remove the following arguments from `/boot/cmdline.txt`:
    - `console=serial0,115200`

 5. Append `ODRIVE_TEST_RIG_NAME=[test-rig-name]` to `/etc/environment`. The HWIL tests use this to look up the the file `[test-rig-name].yaml` which is supposed to describe your test rig.

 6. Reboot.

 7. Install the prerequisites:

        sudo apt-get install ipython3 python3-appdirs python3-yaml python3-jinja2 python3-usb python3-serial python3-can python3-scipy python3-matplotlib python3-ipdb git openocd
        # Optionally, to be able to compile the firmware:
        sudo apt-get install gcc-arm-none-eabi

 8. Install Teensyduino and teensy-loader-cli:

        sudo apt-get install libfontconfig libxft2 libusb-dev

        wget https://downloads.arduino.cc/arduino-1.8.13-linuxarm.tar.xz
        tar -xf arduino-1.8.13-linuxarm.tar.xz
        wget https://www.pjrc.com/teensy/td_153/TeensyduinoInstall.linuxarm
        chmod +x TeensyduinoInstall.linuxarm
        ./TeensyduinoInstall.linuxarm --dir=arduino-1.8.13
        sudo cp -R arduino-1.8.13 /usr/share/arduino
        sudo ln -s /usr/share/arduino/arduino /usr/bin/arduino
        
        git clone https://github.com/PaulStoffregen/teensy_loader_cli
        pushd teensy_loader_cli
        make
        sudo cp teensy_loader_cli /usr/bin/
        sudo ln -s /usr/bin/teensy_loader_cli /usr/bin/teensy-loader-cli
        popd
        curl https://www.pjrc.com/teensy/49-teensy.rules | sudo tee /etc/udev/rules.d/49-teensy.rules

 9. Add the following lines to `/etc/udev/rules.d/49-stlinkv2.rules`:

        SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE:="0666"
        SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE:="0666"

 10. `sudo mkdir /opt/odrivetest && sudo chown $USER /opt/odrivetest`

 11. At this point you need the ODrive repository. See next section to sync it from your main PC. We assume now that you navigated to `tools/odrive/tests/`.

 12. `sudo ../../odrivetool udev-setup`

 13. `sudo udevadm trigger`

 14. Run once after every reboot: `sudo -E ipython3 --pdb test_runner.py -- --setup-host`

## SSH testing flow

Here's one possible workflow for developing on the local host and testing on a remote SSH host.

We assume that the ODrive repo is at `/path/to/ODriveFirmware` and your testing host is configured under the SSH name `odrv`.

To flash and start remote debugging:

 1. Start OpenOCD remotely, along with a tunnel to localhost: `ssh -t odrv -L3333:localhost:3333 bash -c "\"openocd '-f' 'interface/stlink-v2.cfg' '-f' 'target/stm32f4x_stlink.cfg'\""`
    You can keep this open for multiple debug sessions. Press <kbd>Ctrl</kbd>+<kbd>C</kbd> to quit.
 2. Compile the firmware.
 3. In VSCode, select the run configuration "Debug ODrive v3.x/v4.x - Remote" and press Run. This will flash the new firmware before dropping you into the debugger.

To run a test:

    rsync -avh -e ssh /path/to/ODriveFirmware/ odrv:/opt/odrivetest --exclude="Firmware/build" --exclude="Firmware/.tup" --exclude=".git" --exclude="GUI" --delete

    ssh odrv
    > cd /opt/odrivetest/tools/odrive/tests/
    > ipython3 --pdb uart_ascii_test.py

