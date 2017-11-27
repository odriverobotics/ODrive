"""
Provides classes that implement the StreamSource/StreamSink and
PacketSource/PacketSink interfaces for serial ports.
"""

import odrive
import serial
import time

class SerialStreamTransport(odrive.protocol.StreamSource, odrive.protocol.StreamSink):
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
            raise odrive.protocol.TimeoutException("expected {} bytes but got only {}", n_bytes, len(result))
        return result

# TODO: provide SerialPacketTransport
