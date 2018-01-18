# See protocol.hpp for an overview of the protocol

import time
import struct
from abc import ABC, abstractmethod

SYNC_BYTE = 0xAA
CRC8_INIT = 0x42
CRC16_INIT = 0x1337
PROTOCOL_VERSION = 1

CRC8_DEFAULT = 0x37 # this must match the polynomial in the C++ implementation
CRC16_DEFAULT = 0x3d65 # this must match the polynomial in the C++ implementation

MAX_PACKET_SIZE = 128

def calc_crc(remainder, value, polynomial, bitwidth):
    topbit = (1 << (bitwidth - 1))

    # Bring the next byte into the remainder.
    remainder ^= (value << (bitwidth - 8))
    for bitnumber in range(0,8):
        if (remainder & topbit):
            remainder = (remainder << 1) ^ polynomial
        else:
            remainder = (remainder << 1)

    return remainder & ((1 << bitwidth) - 1)

def calc_crc8(remainder, value):
    if isinstance(value, bytearray) or isinstance(value, bytes) or isinstance(value, list):
        for byte in value:
            remainder = calc_crc(remainder, byte, CRC8_DEFAULT, 8)
    else:
        remainder = calc_crc(remainder, byte, CRC8_DEFAULT, 8)
    return remainder

def calc_crc16(remainder, value):
    if isinstance(value, bytearray) or isinstance(value, bytes) or isinstance(value, list):
        for byte in value:
            remainder = calc_crc(remainder, byte, CRC16_DEFAULT, 16)
    else:
        remainder = calc_crc(remainder, value, CRC16_DEFAULT, 16)
    return remainder

# Can be verified with http://www.sunshine2k.de/coding/javascript/crc/crc_js.html:
#print(hex(calc_crc8(0x12, [1, 2, 3, 4, 5, 0x10, 0x13, 0x37])))
#print(hex(calc_crc16(0xfeef, [1, 2, 3, 4, 5, 0x10, 0x13, 0x37])))


class TimeoutException(Exception):
    pass

class ChannelBrokenException(Exception):
    pass

class DeviceInitException(Exception):
    pass


class StreamSource(ABC):
    @abstractmethod
    def get_bytes(self, deadline):
        pass

class StreamSink(ABC):
    @abstractmethod
    def process_bytes(self, bytes):
        pass

class PacketSource(ABC):
    @abstractmethod
    def get_packet(self, deadline):
        pass

class PacketSink(ABC):
    @abstractmethod
    def process_packet(self, packet):
        pass


class StreamToPacketConverter(StreamSink):
    _header = []
    _packet = []
    _packet_length = 0

    def __init__(self, output):
        self._output = output

    def process_bytes(self, bytes):
        """
        Processes an arbitrary number of bytes. If one or more full packets are
        are received, they are sent to this instance's output PacketSink.
        Incomplete packets are buffered between subsequent calls to this function.
        """

        for byte in bytes:
            if (len(self._header) < 3):
                # Process header byte
                self._header.append(byte)
                if (len(self._header) == 1) and (self._header[0] != SYNC_BYTE):
                    self._header = []
                elif (len(self._header) == 2) and (self._header[1] & 0x80):
                    self._header = [] # TODO: support packets larger than 128 bytes
                elif (len(self._header) == 3) and calc_crc8(CRC8_INIT, self._header):
                    self._header = []
                elif (len(self._header) == 3):
                    self._packet_length = self._header[1] + 2
            else:
                # Process payload byte
                self._packet.append(byte)

            # If both header and packet are fully received, hand it on to the packet processor
            if (len(self._header) == 3) and (len(self._packet) == self._packet_length):
                if calc_crc16(CRC16_INIT, self._packet) == 0:
                    self._output.process_packet(self._packet[:-2])
                self._header = []
                self._packet = []
                self._packet_length = 0


class PacketToStreamConverter(PacketSink):
    def __init__(self, output):
        self._output = output

    def process_packet(self, packet):
        if (len(packet) >= MAX_PACKET_SIZE):
            raise NotImplementedError("packet larger than 127 currently not supported")

        header = [SYNC_BYTE, len(packet)]
        header.append(calc_crc8(CRC8_INIT, header))

        self._output.process_bytes(header)
        self._output.process_bytes(packet)

        # append CRC in big endian
        crc16 = calc_crc16(CRC16_INIT, packet)
        self._output.process_bytes(struct.pack('>H', crc16))

class PacketFromStreamConverter(PacketSource):
    def __init__(self, input):
        self._input = input
    
    def get_packet(self, deadline):
        """
        Requests bytes from the underlying input stream until a full packet is
        received or the deadline is reached, in which case None is returned. A
        deadline before the current time corresponds to non-blocking mode.
        """
        while True:
            header = bytes()

            # TODO: sometimes this call hangs, even though the device apparently sent something
            header = header + self._input.get_bytes_or_fail(1, deadline)
            if (header[0] != SYNC_BYTE):
                #print("sync byte mismatch")
                continue

            header = header + self._input.get_bytes_or_fail(1, deadline)
            if (header[1] & 0x80):
                #print("packet too large")
                continue # TODO: support packets larger than 128 bytes

            header = header + self._input.get_bytes_or_fail(1, deadline)
            if calc_crc8(CRC8_INIT, header) != 0:
                #print("crc8 mismatch")
                continue

            packet_length = header[1] + 2
            #print("wait for {} bytes".format(packet_length))
            packet = self._input.get_bytes_or_fail(packet_length, deadline)
            if calc_crc16(CRC16_INIT, packet) != 0:
                #print("crc16 mismatch")
                continue
            return packet[:-2]


class Channel(PacketSink):
    _outbound_seq_no = 0
    _interface_definition_crc = 0
    _expected_acks = {}

    # Choose these parameters to be sensible for a specific transport layer
    _resend_timeout = 0.1     # [s]
    _send_attempts = 5

    def __init__(self, name, input, output):
        """
        Params:
        input: A PacketSource where this channel will source packets from on
               demand. Alternatively packets can be provided to this channel
               directly by calling process_packet on this instance.
        output: A PacketSink where this channel will put outgoing packets.
        """
        self._name = name
        self._input = input
        self._output = output

    def remote_endpoint_operation(self, endpoint_id, input, expect_ack, output_length):
        if input is None:
            input = bytearray(0)
        if (len(input) >= 128):
            raise Exception("packet larger than 127 currently not supported")

        if (expect_ack):
            endpoint_id |= 0x8000

        self._outbound_seq_no = ((self._outbound_seq_no + 1) & 0x7fff)
        self._outbound_seq_no |= 0x80 # FIXME: we hardwire one bit of the seq-no to 1 to avoid conflicts with the legacy protocol
        seq_no = self._outbound_seq_no
        packet = struct.pack('<HHH', seq_no, endpoint_id, output_length)
        packet = packet + input

        crc16 = calc_crc16(CRC16_INIT, packet)
        if (endpoint_id & 0x7fff == 0):
            trailer = PROTOCOL_VERSION
        else:
            trailer = self._interface_definition_crc
        #print("append trailer " + trailer)
        packet = packet + struct.pack('<H', trailer)

        if (expect_ack):
            self._expected_acks[seq_no] = None
            attempt = 0
            while (attempt < self._send_attempts):
                self._output.process_packet(packet)
                deadline = time.monotonic() + self._resend_timeout
                # Read and process packets until we get an ack or need to resend
                # TODO: support I/O driven reception (wait on semaphore)
                while True:
                    try:
                        response = self._input.get_packet(deadline)
                    except TimeoutException:
                        break # resend
                    # process response, which is hopefully our ACK
                    self.process_packet(response)
                    if not self._expected_acks[seq_no] is None:
                        return self._expected_acks.pop(seq_no, None)
                    break
                # TODO: record channel statistics
                attempt += 1
            raise ChannelBrokenException()
        else:
            # fire and forget
            self._output.process_packet(packet)
            return None
    
    def remote_endpoint_read_buffer(self, endpoint_id):
        """
        Handles reads from long endpoints
        """
        # TODO: handle device that could (maliciously) send infinite stream
        buffer = bytes()
        while True:
            chunk_length = 64
            chunk = self.remote_endpoint_operation(endpoint_id, struct.pack("<I", len(buffer)), True, chunk_length)
            if (len(chunk) == 0):
                break
            buffer += chunk
        return buffer

    def process_packet(self, packet):
        #print("process packet")
        packet = bytes(packet)
        if (len(packet) < 2):
            raise Exception("packet too short")

        seq_no = struct.unpack('<H', packet[0:2])[0]

        if (seq_no & 0x8000):
            seq_no &= 0x7fff
            self._expected_acks[seq_no] = packet[2:]

        else:
            #if (calc_crc16(crc16, struct.pack('<HBB', PROTOCOL_VERSION, packet[-2], packet[-1]))):
            #    raise Exception("CRC16 mismatch")
            print("endpoint requested")
            # TODO: handle local endpoint operation
