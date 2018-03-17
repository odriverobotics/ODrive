# requires pyusb
#   pip install --pre pyusb

import sys
import time
import usb.core
import usb.util
import odrive.protocol

ODRIVE_VID_PID_PAIRS = [
  (0x1209, 0x0D31),
  (0x1209, 0x0D32), # <== TODO: this is the only official ODrive PID, remove the other ones
  (0x1209, 0x0D33)
]

class USBBulkTransport(odrive.protocol.PacketSource, odrive.protocol.PacketSink):
  def __init__(self, dev, printer):
    self._printer = printer
    self.dev = dev
    self._name = "USB device {}:{}".format(dev.idVendor, dev.idProduct)
    self._was_damaged = False

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
    # Resetting device to start init from a known state
    # self.dev.reset()
    # time.sleep(1)
    # detach kernel driver
    try:
      if self.dev.is_kernel_driver_active(1):
        self.dev.detach_kernel_driver(1)
        self._printer("Detached Kernel Driver\n")
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
    self._printer("EndpointAddress for writing {}\n".format(self.epw.bEndpointAddress))
    # read endpoint
    self.epr = usb.util.find_descriptor(self.intf,
        # match the first IN endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN
    )
    assert self.epr is not None
    self._printer("EndpointAddress for reading {}\n".format(self.epr.bEndpointAddress))

  def shutdown(self):
    return 0

  def process_packet(self, usbBuffer):
    try:
      ret = self.epw.write(usbBuffer, 0)
      if self._was_damaged:
        self._printer("Recovered from USB halt/stall condition")
        self._was_damaged = False
      return ret
    except usb.core.USBError as ex:
      if ex.errno == 19: # "no such device"
        raise odrive.protocol.ChannelBrokenException()
      else:
        # Try resetting halt/stall condition
        try:
          self.epw.clear_halt()
        except usb.core.USBError:
          raise odrive.protocol.ChannelBrokenException()
        # Retry transfer
        self._was_damaged = True
        raise odrive.protocol.ChannelDamagedException()

  def get_packet(self, deadline):
    try:
      bufferLen = self.epr.wMaxPacketSize
      timeout = max(int((deadline - time.monotonic()) * 1000), 0)
      ret = self.epr.read(bufferLen, timeout)
      if self._was_damaged:
        self._printer("Recovered from USB halt/stall condition")
        self._was_damaged = False
      return bytearray(ret)
    except usb.core.USBError as ex:
      if ex.errno == 19: # "no such device"
        raise odrive.protocol.ChannelBrokenException()
      else:
        # Try resetting halt/stall condition
        try:
          self.epw.clear_halt()
        except usb.core.USBError:
          raise odrive.protocol.ChannelBrokenException()
        # Retry transfer
        self._was_damaged = True
        raise odrive.protocol.ChannelDamagedException()

  def send_max(self):
    return 64

  def receive_max(self):
    return 64


def discover_channels(path, serial_number, callback, cancellation_token, printer):
  """
  Scans for USB devices that match the path spec.
  This function blocks until cancellation_token is set.
  """
  if path == None or path == "":
    bus = None
    address = None
  else:
    try:
      bus = int(path.split(":")[0])
      address = int(path.split(":")[1])
    except (ValueError, IndexError):
      raise Exception("{} is not a valid USB path specification. "
                      "Expected a string of the format BUS:DEVICE where BUS "
                      "and DEVICE are integers.".format(path))
  
  known_devices = []
  def device_matcher(device):
    #print("  test {:04X}:{:04X}".format(device.idVendor, device.idProduct))
    if (device.bus, device.address) in known_devices:
      return False
    if bus != None and device.bus != bus:
      return False
    if address != None and device.address != address:
      return False
    if serial_number != None and device.serial_number != serial_number:
      return False
    if (device.idVendor, device.idProduct) not in ODRIVE_VID_PID_PAIRS:
      return False
    return True

  while not cancellation_token.is_set():
    printer("USB discover loop")
    devices = usb.core.find(find_all=True, custom_match=device_matcher)
    for usb_device in devices:
      try:
        bulk_device = USBBulkTransport(usb_device, printer)
        printer(bulk_device.info())
        bulk_device.init()
        channel = odrive.protocol.Channel(
                "USB device bus {} device {}".format(usb_device.bus, usb_device.address),
                bulk_device, bulk_device)
        channel.usb_device = usb_device # for debugging only
      except usb.core.USBError as ex:
        if ex.errno == 13:
          printer("USB device access denied. Did you set up your udev rules correctly?")
          continue
        raise
      callback(channel)
      known_devices.append((usb_device.bus, usb_device.address))
    time.sleep(1)
