# requires pyusb
#   pip install --pre pyusb


# Exceptions
class ODriveError(Exception):
  pass

class ODriveNotConnectedError(ODriveError):
  pass

USB_DEV_ODRIVE_3_1 = (0x1209, 0x0D31)
USB_DEV_ODRIVE_3_2 = (0x1209, 0x0D32)
USB_DEV_ODRIVE_3_3 = (0x1209, 0x0D33)
# all devices
USB_VID_PID_PAIRS = [
  USB_DEV_ODRIVE_3_1,
  USB_DEV_ODRIVE_3_2,
  USB_DEV_ODRIVE_3_3,
  ]

def noprint(x):
  pass
