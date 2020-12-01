# ## Resources ##
#
#  - Overview over the most important commands:
#    https://btbm.ch/rigol-dp832-lxi-commands-with-python/
#
#  - Programming Guide:
#    http://beyondmeasure.rigoltech.com/acton/attachment/1579/f-03a1/1/-/-/-/-/DP800%20Programming%20Guide.pdf
#

class Dp800(object):
    """
    Provides an command and control interface to power supplies implementing
    the USB Test and Measurement Class (TMC) protocol.

    It was tested on a Rigol DP832 but is likely to work with other USB TMC
    devices too.
    """
    def __init__(self, **device_filter):
        # If something doesn't work it sometimes helps to reset the USB device
        #import usb.core
        #dev = usb.core.find(idVendor=0x1ab1, idProduct=0x0e11, **device_filter)
        #dev.reset()
        #cfg = dev.get_active_configuration()
        #intf = cfg.interfaces()[0]
        #usb.util.claim_interface(dev, intf)
        #ep_in = intf.endpoints()[0]
        #ep_in = intf.endpoints()[1]
        #ep_out = intf.endpoints()[2]
        ##ep_out.write(b"*IDN?\n")
        #import ipdb; ipdb.set_trace()
        
        import usbtmc
        self._dev = usbtmc.Instrument(0x1ab1, 0x0e11)
        self._id = self._dev.ask("*IDN?")

    def _get_setpoint(self, channel):
        """
        Returns the currently configured voltage and current setpoints.
        channel: The channel number (starting at 1).
        Returns: A tuple of the form (voltage, current).
        """
        response = self._dev.ask(f":SOUR{channel}:VOLT?;:SOUR{channel}:CURR?")
        return tuple((float(v) for v in response.split(";")))

    def _apply(self, channel, voltage, current):
        """
        Sets the voltage and current setpoint for the specified channel.
        channel: The channel number (starting at 1).
        """
        self._dev.write(f":APPL CH{channel},{voltage},{current}")

    def _measure(self, channel):
        """
        Measures the voltage, current and power on the selected channel.
        channel: The channel number (starting at 1).
        Returns: A tuple of the form (voltage, current, power).
        """
        response = self._dev.ask(f":MEAS:ALL? CH{channel}")
        return tuple((float(v) for v in response.split(",")))

class Dp800Channel(object):
    """
    Represents one logical power channel on a TMC power supply. This usually
    corresponds to one real channel on the device but can also bundle multiple
    real channels that are connected in series or in parallel.
    """
    def __init__(self, device, channels, conn_type):
        """
        channels: A list of channel numbers.
        conn_type: "series": the channels are connected in series.
                   "parallel": the channels are connected in parallel.
                   This has no effect if channels has length 1.
        """
        self._dev = device
        self._channels = channels
        self._in_series = {'series': True, 'parallel': False}[conn_type]

    def get_setpoint(self):
        setpoints = [self._dev._get_setpoint(c) for c in self._channels]
        setpoint = sum(v for v, _ in setpoints), sum(c for _, c in setpoints)
        if self._in_series:
            return setpoint[0], setpoint[1] / len(self._channels)
        else:
            return setpoint[0] / len(self._channels), setpoint[1]

    def apply(self, voltage, current):
        if self._in_series:
            voltage /= len(self._channels)
        else:
            current /= len(self._channels)
        for ch in self._channels:
            self._dev._apply(ch, voltage, current)

    def measure(self):
        setpoints = [self._dev._measure(c) for c in self._channels]
        setpoint = sum(v for v, _, _ in setpoints), sum(c for _, c, _ in setpoints), sum(p for _, _, p in setpoints)
        if self._in_series:
            return setpoint[0], setpoint[1] / len(self._channels), setpoint[2]
        else:
            return setpoint[0] / len(self._channels), setpoint[1], setpoint[2]
        

mydev = Dp800()
mychannel = Dp800Channel(mydev, [1, 2], 'series')
