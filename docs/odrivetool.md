# ODrive Tool

The ODrive Tool is the accompanying PC program for the ODrive. It's main purpose is to provide an interactive shell to control the device manually, as well as some supporting functions like firmware update.

## Installation

Refer to the [Getting Started guide](getting-started#downloading-and-installing-tools).

Type `odrivetool --help` to see what features are available.

## Multiple ODrives

By default, `odrivetool` will connect to any ODrive it finds. If this is not what you want, you can select a specific ODrive.

To find the serial number of your ODrive, run `odrivetool`, connect exactly one ODrive and power it up. You should see this:
```
Connected to ODrive 306A396A3235 as odrv0
In [1]:
```
`306A396A3235` is the serial number of this particular ODrive. If you want ODrive Tool to ignore all other devices you would close it and then run `odrivetool --serial-number 306A396A3235`.

<details><summary markdown="span">My ODrive is stuck in DFU mode, can I still find the serial number?</summary><div markdown="block">
Yes, the serial number is part of the USB descriptors.

In Linux you can find it by running:
```
$ (sudo lsusb -d 1209:0d32 -v; sudo lsusb -d 0483:df11 -v) 2>/dev/null | grep iSerial
  iSerial                 3 385F324D3037
  iSerial                 3 306A396A3235
```
Here, two ODrives are connected.
</div></details>

## Configuration Backup

You can use ODrive Tool to back up and restore device configurations or transfer the configuration of one ODrive to another one.

 * To save the configuration to a file on the PC, run `odrivetool backup-config my_config.json`.
 * To restore the configuration form such a file, run `odrivetool restore-config my_config.json`.

## Device Firmware Update

<div class="note" markdown="span">__ODrive v3.4 or earlier__: DFU is not supported on these devices. You need to [flash with the external programmer](#flashing-with-an-stlink) instead.</div>

To update the ODrive to the newest firmware release, simply open up a terminal and run the following command:

```
~ $ odrivetool dfu
ODrive control utility v0.3.7.dev
Waiting for ODrive...
Found ODrive 308039673235 (v3.5-24V) with firmware v0.3.7-dev
Checking online for newest firmware... found v0.3.7
Downloading firmware...
Putting device 308039673235 into DFU mode...
Erasing... done            
Flashing... done            
Verifying... done            
```

Note that this command will connect to GitHub servers to retrieve the latest firmware.

If you have a non-default configuration saved on the device, ODrive Tool will try to carry over the configuration across the firmware update. If any of the settings are removed or renamed, you will get warning messages.

<details><summary markdown="span">How to flash a custom firmware</summary><div markdown="block">
If you want to flash a specific firmware file instead of automatically downloading one, you can run `odrivetool dfu [path/to/firmware/file.hex]`.

You can download one of the officially released firmware files from [here](https://github.com/madcowswe/ODrive/releases). You will need one of the __.hex__ files (not the __.elf__ file). Make sure you select the file that matches your board version.

To compile firmware from source, refer to the [developer guide](developer-guide).
</div></details>


### Troubleshooting

* __Windows__: During the update, a new device called "STM32 BOOTLOADER" will appear. Open the [Zadig utility](http://zadig.akeo.ie/) and set the driver for "STM32 BOOTLOADER" to libusb-win32. After that the firmware update will continue.
* On some machines you will need to unplug and plug back in the USB cable to make the PC understand that we switched from regular mode to bootloader mode.
* If the DFU script can't find the device, try forcing it into DFU mode.

  <details><summary markdown="span">How to force DFU mode (ODrive v3.5)</summary><div markdown="block">
  Flick the DIP switch that "DFU, RUN" to "DFU" and power cycle the board. If that alone doesn't work, also connect the  After you're done, put the switch back into the "RUN" position and power cycle the board again.
  </div></details>

  <details><summary markdown="span">How to force DFU mode (ODrive v3.1, v3.2)</summary><div markdown="block">
  Connect the pin "BOOT0" to "3.3V" and power cycle the board. If that alone doesn't work, also connect the pin "GPIO1" to "GND". After you're done, remove the wires and power cycle the board again.
  </div></details>


## Flashing with an STLink

This procedure is only necessary for ODrive v3.4 or earlier. You will need an STLink/v2 or compatible programmer. You should have received one with your ODrive.

1. Install OpenOCD
   * **Windows:** [instructions](http://gnuarmeclipse.github.io/openocd/install/) (also follow the instructions on the ST-LINK/V2 drivers)
   * **Linux:** `sudo apt-get install openocd`
   * **macOS:** `brew install openocd`
2. Download the latest firmware release form [here](https://github.com/madcowswe/ODrive/releases). You will need the __.elf__ file. Make sure you select the file that matches your board version.
3. Wire up the ODrive and STLink/v2 programmer as shown in this picture:<br>
     ![stlink-wiring](stlink-wiring-cropped.jpg)
     Power up the ODrive.
4. Open up a terminal and navigate to the directory where the firmware is.
5. Run the following command (replace `ODriveFirmware_v3.4-24V.elf` with the name of your firmware file):
  ```
~/Downloads $ openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c reset\ halt -c flash\ write_image\ erase\ ODriveFirmware_v3.4-24V.elf -c reset\ run -c exit
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
Info : STLINK v2 JTAG v17 API v2 SWIM v4 VID 0x0483 PID 0x3748
Info : using stlink api v2
Info : Target voltage: 3.236027
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
adapter speed: 2000 kHz
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08009224 msp: 0x20020000
auto erase enabled
Info : device id = 0x10076413
Info : flash size = 1024kbytes
target halted due to breakpoint, current mode: Thread
xPSR: 0x61000000 pc: 0x20000046 msp: 0x20020000
Warn : no flash bank found for address 10000000
wrote 262144 bytes from file ODriveFirmware_v3.4-24V.elf in 10.194110s (25.113 KiB/s)
adapter speed: 2000 kHz
  ```

If something doesn't work, make sure `openocd` is in your `PATH` variable, check that the wires are connected properly and try with elevated privileges.
