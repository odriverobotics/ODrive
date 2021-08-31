#!/usr/bin/env python
"""
Tool for flashing .hex files to the ODrive via the STM built-in USB DFU mode.
"""

from __future__ import print_function
import argparse
import sys
import time
import threading
import platform
import struct
import requests
import re
import io
import os
import usb.core
import fibre
import odrive
from odrive.utils import Event, OperationAbortedException
from odrive.dfuse import *

if sys.version_info < (3, 0):
    _print = print
    def print(*vals, **kwargs):
        _print(*vals)
        if kwargs.get('flush', False):
            sys.stdout.flush()

try:
    from intelhex import IntelHex
except:
    sudo_prefix = "" if platform.system() == "Windows" else "sudo "
    print("You need intelhex for this ({}pip install IntelHex)".format(sudo_prefix), file=sys.stderr)
    sys.exit(1)


def get_fw_version_string(fw_version):
    if (fw_version[0], fw_version[1], fw_version[2]) == (0, 0, 0):
        return "[unknown version]"
    else:
        return "v{}.{}.{}{}".format(fw_version[0], fw_version[1], fw_version[2], "-dev" if fw_version[3] else "")

def get_hw_version_string(hw_version):
    if hw_version == (0, 0, 0):
        return "[unknown version]"
    else:
        return "v{}.{}{}".format(hw_version[0], hw_version[1], ("-" + str(hw_version[2]) + "V") if hw_version[2] > 0 else "")

def populate_sectors(sectors, hexfile):
    """
    Checks for which on-device sectors there is data in the hex file and
    returns a (sector, data) tuple for each touched sector where data
    is a byte array of the same size as the sector.
    """
    for sector in sectors:
        addr = sector['addr']
        size = sector['len']
        # check if any segment from the hexfile overlaps with this sector
        touched = False
        for (start, end) in hexfile.segments():
            if start < addr and end > addr:
                touched = True
                break
            elif start >= addr and start < addr + size:
                touched = True
                break

        if touched:
            # TODO: verify if the section is writable
            yield (sector, hexfile.tobinarray(addr, addr + size - 1))


def get_first_mismatch_index(array1, array2):
    """
    Compares two arrays and returns the index of the
    first unequal item or None if both arrays are equal
    """
    if len(array1) != len(array2):
        raise Exception("arrays must be same size")
    for pos in range(len(array1)):
        if (array1[pos] != array2[pos]):
            return pos
    return None

def dump_otp(dfudev):
    """
    Dumps the contents of the one-time-programmable
    memory for debugging purposes.
    The OTP is used to determine the board version.
    """
    # 512 Byte OTP
    otp_sector = [s for s in dfudev.sectors if s['name'] == 'OTP Memory' and s['addr'] == 0x1fff7800][0]
    data = dfudev.read_sector(otp_sector)
    print(' '.join('{:02X}'.format(x) for x in data))

    # 16 lock bytes
    otp_lock_sector = [s for s in dfudev.sectors if s['name'] == 'OTP Memory' and s['addr'] == 0x1fff7A00][0]
    data = dfudev.read_sector(otp_lock_sector)
    print(' '.join('{:02X}'.format(x) for x in data))

class Firmware():
    def __init__(self):
        self.fw_version = (0, 0, 0, True)
        self.hw_version = (0, 0, 0)

    @staticmethod
    def is_newer(a, b):
        a_num = (a[0], a[1], a[2])
        b_num = (b[0], b[1], b[2])
        if a_num == (0, 0, 0) or b_num == (0, 0, 0):
            return False # Cannot compare unknown versions
        return a_num > b_num or (a_num == b_num and not a[3] and b[3])

    def __gt__(self, other):
        """
        Compares two firmware versions. If both versions are equal, the
        prerelease version is considered older than the release version.
        """
        if not isinstance(other, tuple):
            other = other.fw_version
        return Firmware.is_newer(self.fw_version, other)

    def __lt__(self, other):
        """
        Compares two firmware versions. If both versions are equal, the
        prerelease version is considered older than the release version.
        """
        if not isinstance(other, tuple):
            other = other.fw_version
        return Firmware.is_newer(other, self.fw_version)

    def is_compatible(self, hw_version):
        """
        Determines if this firmware is compatible
        with the specified hardware version
        """
        return self.hw_version == hw_version

class FirmwareFromGithub(Firmware):
    """
    Represents a firmware asset
    """
    def __init__(self, release_json, asset_json):
        Firmware.__init__(self)
        if release_json['draft'] or release_json['prerelease']:
            release_json['tag_name'] += "*"
        self.fw_version = odrive.version.version_str_to_tuple(release_json['tag_name'])

        hw_version_regex = r'.*v([0-9]+).([0-9]+)(-(?P<voltage>[0-9]+)V)?.hex'
        hw_version_match = re.search(hw_version_regex, asset_json['name'])
        self.hw_version = (int(hw_version_match.group(1)),
                          int(hw_version_match.group(2)),
                          int(hw_version_match.groupdict().get('voltage') or 0))
        self.github_asset_id = asset_json['id']
        self.hex = None
        # no technical reason to fetch this - just interesting
        self.download_count = asset_json['download_count']
    
    def get_as_hex(self):
        """
        Returns the content of the firmware in as a binary array in Intel Hex format
        """
        if self.hex is None:
            print("Downloading firmware {}...".format(get_fw_version_string(self.fw_version)))
            response = requests.get('https://api.github.com/repos/odriverobotics/ODrive/releases/assets/' + str(self.github_asset_id),
                                    headers={'Accept': 'application/octet-stream'})
            if response.status_code != 200:
                raise Exception("failed to download firmware")
            self.hex = response.content
        return io.StringIO(self.hex.decode('utf-8'))

class FirmwareFromFile(Firmware):
    def __init__(self, file):
        Firmware.__init__(self)
        self._file = file
    def get_as_hex(self):
        return self._file

def get_all_github_firmwares():
    response = requests.get('https://api.github.com/repos/odriverobotics/ODrive/releases')
    if response.status_code != 200:
        raise Exception("could not fetch releases")
    response_json = response.json()
    
    for release_json in response_json:
        for asset_json in release_json['assets']:
            try:
                if asset_json['name'].lower().endswith('.hex'):
                    fw = FirmwareFromGithub(release_json, asset_json)
                    yield fw
            except Exception as ex:
                print(ex)

def get_newest_firmware(hw_version):
    """
    Returns the newest available firmware for the specified hardware version
    """
    firmwares = get_all_github_firmwares()
    firmwares = filter(lambda fw: not fw.fw_version[3], firmwares) # ignore prereleases
    firmwares = filter(lambda fw: fw.hw_version == hw_version, firmwares)
    firmwares = list(firmwares)
    firmwares.sort()
    return firmwares[-1] if len(firmwares) else None

def show_deferred_message(message, cancellation_token):
    """
    Shows a message after 10s, unless cancellation_token gets set.
    """
    def show_message_thread(message, cancellation_token):
        for _ in range(1,10):
            if cancellation_token.is_set():
                return
            time.sleep(1)
        if not cancellation_token.is_set():
            print(message)
    t = threading.Thread(target=show_message_thread, args=(message, cancellation_token))
    t.daemon = True
    t.start()

def put_into_dfu_mode(device, cancellation_token):
    """
    Puts the specified device into DFU mode
    """
    if not hasattr(device, "enter_dfu_mode"):
        print("The firmware on device {:08X} cannot soft enter DFU mode.\n"
              "Please remove power, put the DFU switch into DFU mode,\n"
              "then apply power again. Then try again.\n"
              "If it still doesn't work, you can try to use the DeFuse app or \n"
              "dfu-util, see the odrive documentation.\n"
              "You can also flash the firmware using STLink (`make flash`)"
              .format(device.serial_number))
        return
        
    print("Putting device {:08X} into DFU mode...".format(device.serial_number))
    try:
        device.enter_dfu_mode()
    except fibre.ObjectLostError:
        pass # this is expected because the device reboots
    if platform.system() == "Windows":
        show_deferred_message("Still waiting for the device to reappear.\n"
                            "Use the Zadig utility to set the driver of 'STM32 BOOTLOADER' to libusb-win32.",
                            cancellation_token)

def find_device_in_dfu_mode(serial_number, cancellation_token):
    """
    Polls libusb until a device in DFU mode is found
    """
    while not cancellation_token.is_set():
        params = {} if serial_number == None else {'serial_number': serial_number}
        stm_device = usb.core.find(idVendor=0x0483, idProduct=0xdf11, **params)
        if stm_device != None:
            return stm_device
        time.sleep(1)
    return None

def get_hw_version_in_dfu_mode(dfudev):
    """
    Reads the hardware version from one-time-programmable memory.
    This is written on all ODrives sold since Summer 2018.
    """
    otp_sector = [s for s in dfudev.sectors if s['name'] == 'OTP Memory' and s['addr'] == 0x1fff7800][0]
    otp_data = dfudev.read_sector(otp_sector)
    if otp_data[0] == 0:
        otp_data = otp_data[16:]
    if otp_data[0] == 0xfe:
        return (otp_data[3], otp_data[4], otp_data[5])
    else:
        return None

def unlock_device(serial_number, cancellation_token):
    print("Looking for ODrive in DFU mode...")
    print("If the program hangs at this point, try to set the DFU switch to \"DFU\" and power cycle the ODrive.")

    stm_device = find_device_in_dfu_mode(serial_number, cancellation_token)
    dfudev = DfuDevice(stm_device)

    print("Unlocking device (this may take a few seconds)...")
    dfudev.unprotect()
    print("done")
    print("")
    print("Now do the following:")
    print(" 1. Put the DFU switch on the ODrive to \"DFU\"")
    print(" 2. Power-cycle the ODrive")
    print(" 3. Run \"odrivetool dfu\" (or any third party DFU tool)")
    print(" 4. Put the DFU switch on the ODrive to \"RUN\"")


def update_device(device, firmware, logger, cancellation_token):
    """
    Updates the specified device with the specified firmware.
    The device passed to this function can either be in
    normal mode or in DFU mode.
    The firmware should be an instance of Firmware or None.
    If firmware is None, the newest firmware for the device is
    downloaded from GitHub releases.
    """

    if isinstance(device, usb.core.Device):
        found_in_dfu = True
        serial_number = device.serial_number
        dfudev = DfuDevice(device)
        if (logger._verbose):
            logger.debug("OTP:")
            dump_otp(dfudev)
        hw_version = get_hw_version_in_dfu_mode(dfudev) or (0, 0, 0)

    else:
        found_in_dfu = False
        serial_number = "{:08X}".format(device.serial_number)
        dfudev = None

        # Read hardware version as reported from firmware
        hw_version_major = device.hw_version_major if hasattr(device, 'hw_version_major') else 0
        hw_version_minor = device.hw_version_minor if hasattr(device, 'hw_version_minor') else 0
        hw_version_variant = device.hw_version_variant if hasattr(device, 'hw_version_variant') else 0
        hw_version = (hw_version_major, hw_version_minor, hw_version_variant)

    if hw_version < (3, 5, 0):
        print("  DFU mode is not supported on board version 3.4 or earlier.")
        print("  This is because entering DFU mode on such a device would")
        print("  break the brake resistor FETs under some circumstances.")
        print("Warning: DFU mode is not supported on ODrives earlier than v3.5 unless you perform a hardware mod.")
        if not odrive.utils.yes_no_prompt("Do you still want to continue?", False):
            raise OperationAbortedException()

    fw_version_major = device.fw_version_major if hasattr(device, 'fw_version_major') else 0
    fw_version_minor = device.fw_version_minor if hasattr(device, 'fw_version_minor') else 0
    fw_version_revision = device.fw_version_revision if hasattr(device, 'fw_version_revision') else 0
    fw_version_prerelease = device.fw_version_unreleased != 0 if hasattr(device, 'fw_version_unreleased') else True
    fw_version = (fw_version_major, fw_version_minor, fw_version_revision, fw_version_prerelease)

    print("Found ODrive {} ({}) with firmware {}{}".format(
                serial_number,
                get_hw_version_string(hw_version),
                get_fw_version_string(fw_version),
                " in DFU mode" if dfudev is not None else ""))

    if firmware is None:
        if hw_version == (0, 0, 0):
            if dfudev is None:
                suggestion = 'You have to manually flash an up-to-date firmware to make automatic checks work. Run `odrivetool dfu --help` for more info.'
            else:
                suggestion = 'Run "make write_otp" to program the board version.'
            raise Exception('Cannot check online for new firmware because the board version is unknown. ' + suggestion)
        print("Checking online for newest firmware...", end='')
        firmware = get_newest_firmware(hw_version)
        if firmware is None:
            raise Exception("could not find any firmware release for this board version")
        print(" found {}".format(get_fw_version_string(firmware.fw_version)))

    if firmware.fw_version <= fw_version:
        print()
        if firmware.fw_version < fw_version:
            print("Warning: you are about to flash firmware {} which is older than the firmware on the device ({}).".format(
                    get_fw_version_string(firmware.fw_version),
                    get_fw_version_string(fw_version)))
        else:
            print("You are about to flash firmware {} which is the same version as the firmware on the device ({}).".format(
                    get_fw_version_string(firmware.fw_version),
                    get_fw_version_string(fw_version)))
        if not odrive.utils.yes_no_prompt("Do you want to flash this firmware anyway?", False):
            raise OperationAbortedException()

    # load hex file
    # TODO: Either use the elf format or pack a custom format with a manifest.
    # This way we can for instance verify the target board version and only
    # have to publish one file for every board (instead of elf AND hex files).
    hexfile = IntelHex(firmware.get_as_hex())

    logger.debug("Contiguous segments in hex file:")
    for start, end in hexfile.segments():
        logger.debug(" {:08X} to {:08X}".format(start, end - 1))

    # Back up configuration
    do_backup_config = False
    if dfudev is None:
        do_backup_config = device.user_config_loaded if hasattr(device, 'user_config_loaded') else False
        if do_backup_config:
            odrive.configuration.backup_config(device, None, logger)
    elif not odrive.utils.yes_no_prompt("The configuration cannot be backed up because the device is already in DFU mode. The configuration may be lost after updating. Do you want to continue anyway?", True):
        raise OperationAbortedException()

    # Put the device into DFU mode if it's not already in DFU mode
    if dfudev is None:
        find_odrive_cancellation_token = Event(cancellation_token)
        put_into_dfu_mode(device, find_odrive_cancellation_token)
        stm_device = find_device_in_dfu_mode(serial_number, cancellation_token)
        find_odrive_cancellation_token.set()
        dfudev = DfuDevice(stm_device)

    hw_version = get_hw_version_in_dfu_mode(dfudev)
    if hw_version is None:
        logger.error("Could not determine hardware version. Flashing precompiled "
                     "firmware could lead to unexpected results. Please use an "
                     "STLink/2 to force-update the firmware anyway. Refer to "
                     "https://docs.odriverobotics.com/developer-guide for details.")
        # Jump to application
        dfudev.jump_to_application(0x08000000)
        return

    logger.debug("Sectors on device: ")
    for sector in dfudev.sectors:
        logger.debug(" {:08X} to {:08X} ({})".format(
            sector['addr'],
            sector['addr'] + sector['len'] - 1,
            sector['name']))

    # fill sectors with data
    touched_sectors = list(populate_sectors(dfudev.sectors, hexfile))

    logger.debug("The following sectors will be flashed: ")
    for sector,_ in touched_sectors:
        logger.debug(" {:08X} to {:08X}".format(sector['addr'], sector['addr'] + sector['len'] - 1))

    # Erase
    try:
        internal_flash_sectors = [sector for sector in dfudev.sectors if sector['name'] == 'Internal Flash']
        for i, sector in enumerate(dfudev.sectors):
            if sector['name'] == 'Internal Flash':
                print("Erasing... (sector {}/{})  \r".format(i, len(internal_flash_sectors)), end='', flush=True)
                dfudev.erase_sector(sector)
        print('Erasing... done            \r', end='', flush=True)
    finally:
        print('', flush=True)

    # Flash
    try:
        for i, (sector, data) in enumerate(touched_sectors):
            print("Flashing... (sector {}/{})  \r".format(i, len(touched_sectors)), end='', flush=True)
            dfudev.write_sector(sector, data)
        print('Flashing... done            \r', end='', flush=True)
    finally:
        print('', flush=True)

    # Verify
    try:
        for i, (sector, expected_data) in enumerate(touched_sectors):
            print("Verifying... (sector {}/{})  \r".format(i, len(touched_sectors)), end='', flush=True)
            observed_data = dfudev.read_sector(sector)
            mismatch_pos = get_first_mismatch_index(observed_data, expected_data)
            if not mismatch_pos is None:
                mismatch_pos -= mismatch_pos % 16
                observed_snippet = ' '.join('{:02X}'.format(x) for x in observed_data[mismatch_pos:mismatch_pos+16])
                expected_snippet = ' '.join('{:02X}'.format(x) for x in expected_data[mismatch_pos:mismatch_pos+16])
                raise RuntimeError("Verification failed around address 0x{:08X}:\n".format(sector['addr'] + mismatch_pos) +
                                   "  expected: " + expected_snippet + "\n"
                                   "  observed: " + observed_snippet)
        print('Verifying... done            \r', end='', flush=True)
    finally:
        print('', flush=True)


    # If the flash operation failed for some reason, your device is bricked now.
    # You can unbrick it as long as the device remains powered on.
    # (or always with an STLink)
    # So for debugging you should comment this last part out.

    # Jump to application
    dfudev.jump_to_application(0x08000000)

    if not found_in_dfu:
        logger.info("Waiting for the device to reappear...")
        device = odrive.find_any(odrive.default_usb_search_path, serial_number,
                        cancellation_token, timeout=30)

        if do_backup_config:
            temp_config_filename = odrive.configuration.get_temp_config_filename(device)
            odrive.configuration.restore_config(device, None, logger)
            os.remove(temp_config_filename)
        
        logger.success("Device firmware update successful.")
    else:
        logger.success("Firmware upload successful.")
        logger.info("To complete the firmware update, set the DFU switch to \"RUN\" and power cycle the board.")
    

def launch_dfu(args, logger, cancellation_token):
    """
    Waits for a device that matches args.path and args.serial_number
    and then upgrades the device's firmware.
    """

    serial_number = args.serial_number
    find_odrive_cancellation_token = Event(cancellation_token)

    logger.info("Waiting for ODrive...")

    devices = [None, None]

    # Start background thread to scan for ODrives in DFU mode
    def find_device_in_dfu_mode_thread():
        devices[0] = find_device_in_dfu_mode(serial_number, find_odrive_cancellation_token)
        find_odrive_cancellation_token.set()
    t = threading.Thread(target=find_device_in_dfu_mode_thread)
    t.daemon = True
    t.start()
    

    # Scan for ODrives not in DFU mode
    # We only scan on USB because DFU is only implemented over USB
    devices[1] = odrive.find_any(odrive.default_usb_search_path, serial_number,
        find_odrive_cancellation_token)
    find_odrive_cancellation_token.set()
    
    device = devices[0] or devices[1]
    firmware = FirmwareFromFile(args.file) if args.file else None

    update_device(device, firmware, logger, cancellation_token)



# Note: the flashed image can be verified using: (0x12000 is the number of bytes to read)
# $ openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c flash\ read_bank\ 0\ image.bin\ 0\ 0x12000 -c exit
# $ hexdump -C image.bin > image.bin.txt
#
# If you compare this with a reference image that was flashed with the STLink, you will see
# minor differences. This is because this script fills undefined sections with 0xff.
# $ diff image_ref.bin.txt image.bin.txt
# 21c21
# < *
# ---
# > 00000180  d9 47 00 08 d9 47 00 08  ff ff ff ff ff ff ff ff  |.G...G..........|
# 2553c2553
# < 00009fc0  9e 46 70 47 00 00 00 00  52 20 96 3c 46 76 50 76  |.FpG....R .<FvPv|
# ---
# > 00009fc0  9e 46 70 47 ff ff ff ff  52 20 96 3c 46 76 50 76  |.FpG....R .<FvPv|


