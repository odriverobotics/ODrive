"""
Provides classes that implement the StreamSource/StreamSink and
PacketSource/PacketSink interfaces for serial ports.
"""

import serial
import time
import fibre

def noprint(x):
  pass

class SerialStreamTransport(fibre.protocol.StreamSource, fibre.protocol.StreamSink):
    def __init__(self, port, baud):
        self._dev = serial.Serial(port, baud, timeout=1)

    def process_bytes(self, bytes):
        self._dev.write(bytes)

    def get_bytes(self, n_bytes, deadline):
        """
        Returns n bytes unless the deadline is reached, in which case the bytes
        that were read up to that point are returned. If deadline is None the
        function blocks forever. A deadline before the current time corresponds
        to non-blocking mode.
        """
        if deadline is None:
            self._dev.timeout = None
        else:
            self._dev.timeout = max(deadline - time.monotonic(), 0)
        return self._dev.read(n_bytes)

    def get_bytes_or_fail(self, n_bytes, deadline):
        result = self.get_bytes(n_bytes, deadline)
        if len(result) < n_bytes:
            raise fibre.protocol.TimeoutException("expected {} bytes but got only {}", n_bytes, len(result))
        return result

# TODO: provide SerialPacketTransport



def channel_from_serial_port(port, baud, packet_based, printer=noprint, device_stdout=noprint):
    """
    Inits a Fibre Protocol channel from a serial port name and baudrate.
    """
    if packet_based == True:
        # TODO: implement packet based transport over serial
        raise NotImplementedError("not supported yet")
    serial_device = fibre.serial_transport.SerialStreamTransport(port, baud)
    input_stream = fibre.protocol.PacketFromStreamConverter(serial_device, device_stdout)
    output_stream = fibre.protocol.StreamBasedPacketSink(serial_device)
    return fibre.protocol.Channel(
            "serial port {}@{}".format(port, baud),
            input_stream, output_stream,
            device_stdout)

def find_dev_serial_ports(search_regex):
    try:
        return ['/dev/' + x for x in filter(re.compile(search_regex).search, os.listdir('/dev'))]
    except FileNotFoundError:
        return []

def find_pyserial_ports():
    return [x.name for x in serial.tools.list_ports.comports()]

def find_serial_channels(printer=noprint, device_stdout=noprint):
    """
    Scans for serial ports.
    Returns a generator of fibre.protocol.Channel objects.
    Not every returned object necessarily represents a compatible device.
    """

    # Real serial ports or USB-Serial converters (tested on Linux and Windows)
    real_serial_ports = find_pyserial_ports()

    # Serial devices that are exposed by the platform
    # for the device's USB connection
    linux_usb_serial_ports = find_dev_serial_ports(r'^ttyACM')
    macos_usb_serial_ports = find_dev_serial_ports(r'^tty\.usbmodem')

    for port in real_serial_ports + linux_usb_serial_ports + macos_usb_serial_ports:
        try:
            yield channel_from_serial_port(port, 115200, False, printer, device_stdout=device_stdout)
        except serial.serialutil.SerialException:
            printer("could not open " + port)
            continue

def open_serial(port_name, printer=noprint, device_stdout=noprint):
    channel = channel_from_serial_port(port_name, 115200, False, printer, device_stdout)
    return object_from_channel(channel, printer)
