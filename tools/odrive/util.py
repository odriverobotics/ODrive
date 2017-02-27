# requires pyusb
#   pip install --pre pyusb

import time
import usb.core
import usb.util

# Exceptions
class ODriveError(Exception):
  pass

class ODriveNotConnectedError(ODriveError):
  pass

# ODrive generic        1209:0D31
USBID_VID_ODRIVE        = 0x1209
USBID_PID_ODRIVE        = 0x0D31
# ODrive rev 3.1        1209:0D31
USBID_VID_ODRIVE_3_1    = USBID_VID_ODRIVE
USBID_PID_ODRIVE_3_1    = USBID_PID_ODRIVE
USB_DEV_ODRIVE_3_1      = (USBID_VID_ODRIVE_3_1, USBID_PID_ODRIVE_3_1)
# all devices
all_devices         = [USB_DEV_ODRIVE_3_1]

# device messages
DEVICE_NOT_FOUND    = 'ODrive Not Found'
DEVICE_FOUND        = 'ODrive Found!'

def noprint(x):
  pass

def poll_odrive_device(devices=all_devices, intv=1, printer=noprint):
  while 1:
    time.sleep(intv)
    for device in devices:
      try:
        idVendor  = device[0]
        idProduct = device[1]
        dev = usb.core.find(idVendor=idVendor, idProduct=idProduct)
        if dev is not None:
          printer(DEVICE_FOUND)
          return device
      except:
        printer(DEVICE_NOT_FOUND)
