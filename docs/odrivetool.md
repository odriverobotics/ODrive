# ODrive Tool

### Multiple ODrives
By default, `odrivetool` will connect to any ODrive it finds. If this is not what you want, you can select a specific ODrive.

If you have multiple ODrives connected, you should specify which one to connect to.
* Run `(lsusb -d 1209:0d32 -v; lsusb -d 0483:df11 -v) 2>/dev/null | grep iSerial` to list the serial number of all flashable devices. Example output:
```
  iSerial                 3 385F324D3037
  iSerial                 3 306A396A3235
```
* The last column is the serial number you're looking for. You can unplug selected devices to track down the one you want to update.
* Run `odrivetool --serial-number=385F324D3037`, where `385F324D3037` is the targeted serial number.
* To connect over serial, run `odrivetool --path serial`.

## Device Firmware Update

Note: ODrive v3.4 and earlier require you to flash with the external programmer first (see below), before you can reflash in standalone mode.

* Run `make dfu` in the `Firmware` directory.
* __Windows__: During the update, a new device called "STM32 BOOTLOADER" will appear. Open the [Zadig utility](http://zadig.akeo.ie/) and set the driver for "STM32 BOOTLOADER" to libusb-win32. After that the firmware update will continue.
* On some machines you will need to unplug and plug back in the USB cable to make the PC understand that we switched from regular mode to bootloader mode.
* Currently a firmware update will preserve the configuration if and only if the parameters of both firmware versions are identical. This will change in the future.
