# requires pyusb
#   pip install --pre pyusb

import usb.core
import usb.util
import sys
import time

from odrive.util import ODriveNotConnectedError
from odrive.util import USBID_VID_ODRIVE, USBID_PID_ODRIVE

BULK_DEVICE_NOT_FOUND     = 'ODrive BulkDevice Not Found'
BULK_DEVICE_FOUND         = 'ODrive BulkDevice Found!'

def noprint(x):
  pass

def poll_odrive_bulk_device(intv=1, printer=noprint):
  # look for device
  while 1:
    time.sleep(intv)
    try:
      dev = ODriveBulkDevice()
      break
    except:
      printer(BULK_DEVICE_NOT_FOUND)
  # found device
  printer(BULK_DEVICE_FOUND)
  return dev

class ODriveBulkDevice():
  def __init__(self, idVendor=None, idProduct=None):
    # ODrive idVendor
    if idVendor is None:
      idVendor = USBID_VID_ODRIVE
    # ODrive idProduct
    if idProduct is None:
      idProduct = USBID_PID_ODRIVE
    # find our device
    self.dev = usb.core.find(idVendor=idVendor, idProduct=idProduct)
    # was it found?
    if self.dev is None:
      raise ODriveNotConnectedError()

  ##
  # information about the connected device
  ##
  def info(self):
    # loop through configurations
    string = ""
    for cfg in self.dev:
      string += "ConfigurationValue {0}\n".format(cfg.bConfigurationValue)
      for intf in cfg:
        string += "\tInterfaceNumber {0},{0}\n".format(intf.bInterfaceNumber, intf.bAlternateSetting)
        for ep in intf:
          string += "\t\tEndpointAddress {0}\n".format(ep.bEndpointAddress)
    return string

  def init(self):
    try:
      string = ""
      # detach kernel driver
      try:
        if self.dev.is_kernel_driver_active(1):
          self.dev.detach_kernel_driver(1)
          string += "Detached Kernel Driver\n"
      except NotImplementedError:
        pass #is_kernel_driver_active not implemented on Windows
      # set the active configuration. With no arguments, the first
      # configuration will be the active one
      self.dev.set_configuration()
      # get an endpoint instance
      self.cfg = self.dev.get_active_configuration()
      self.intf = self.cfg[(1,0)]
      # write endpoint
      self.epw = usb.util.find_descriptor(self.intf,
          # match the first OUT endpoint
          custom_match = \
          lambda e: \
              usb.util.endpoint_direction(e.bEndpointAddress) == \
              usb.util.ENDPOINT_OUT
      )
      assert self.epw is not None
      string += "EndpointAddress for writing {}\n".format(self.epw.bEndpointAddress)
      # read endpoint
      self.epr = usb.util.find_descriptor(self.intf,
          # match the first IN endpoint
          custom_match = \
          lambda e: \
              usb.util.endpoint_direction(e.bEndpointAddress) == \
              usb.util.ENDPOINT_IN
      )
      assert self.epr is not None
      string += "EndpointAddress for reading {}\n".format(self.epr.bEndpointAddress)
      return string
    except usb.core.USBError:
      #return -1
      raise

  def shutdown(self):
    return 0

  def send(self, usbBuffer):
    try:
      ret = self.epw.write(usbBuffer, 0)
      return ret
    except usb.core.USBError:
      #return -1
      raise

  def receive(self, bufferLen):
    try:
      ret = self.epr.read(bufferLen, 100)
      return ret
    except usb.core.USBError:
      #return -1
      raise

  def send_max(self):
    return 64

  def receive_max(self):
    return 64
