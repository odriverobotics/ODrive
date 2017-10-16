# See protocol.hpp for an overview of the protocol

import struct

SYNC_BYTE = '$'
CRC8_INIT = 0
CRC16_INIT = 0
PROTOCOL_VERSION = 1

CRC8_DEFAULT = 0x37 # this must match the polynomial in the C++ implementation
CRC16_DEFAULT = 0x3d65 # this must match the polynomial in the C++ implementation

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
    if type(value) == bytearray or isinstance(value, list):
        for b in value:
            remainder = calc_crc(remainder, b, CRC8_DEFAULT, 8)
    else:
        remainder = calc_crc(remainder, b, CRC8_DEFAULT, 8)
    return remainder

def calc_crc16(remainder, value):
    if type(value) == bytearray or isinstance(value, list):
        for b in value:
            remainder = calc_crc(remainder, b, CRC16_DEFAULT, 16)
    else:
        remainder = calc_crc(remainder, b, CRC16_DEFAULT, 16)
    return remainder

# Can be verified with http://www.sunshine2k.de/coding/javascript/crc/crc_js.html:
#print(hex(calc_crc8(0x12, [1, 2, 3, 4, 5, 0x10, 0x13, 0x37])))
#print(hex(calc_crc16(0xfeef, [1, 2, 3, 4, 5, 0x10, 0x13, 0x37])))

class StreamWriter(object):
    pass
    
class PacketWriter(object):
    pass


class StreamToPacketConverter(StreamWriter):
    _header = []
    _packet = []
    _packet_length = 0

    def __init__(self, output):
        self._output = output

    def write_bytes(self, bytes):
        result = None

        for b in bytes:
            if (len(self._header) < 3):
                # Process header byte
                self._header.append(b)
                if (len(self._header) == 1) and (self._header[0] != SYNC_BYTE):
                    self._header = []
                elif (len(self._header) == 2) and (self._header[1] & 0x80):
                    self._header = [] # TODO: support packets larger than 128 bytes
                elif (len(self._header) == 3) and calc_crc8(CRC8_INIT, self._header):
                    self._header = []
                elif (len(self._header) == 3):
                    self._packet_length = self._header[1]
            else:
                # Process payload byte
                self._packet.append(b)

            # If both header and packet are fully received, hand it on to the packet processor
            if (len(self._header) == 3) and (len(self._packet) == self._packet_length):
                try:
                    self._output.write_packet(self._packet)
                except Exception, ex:
                    result = ex
                self._header = []
                self._packet = []
                self._packet_length = 0
        
        if isinstance(result, Exception):
            raise Exception("something went wrong")


class PacketToStreamConverter(PacketWriter):
    def __init__(self, output):
        self._output = output

    def write_packet(self, packet):
        if (len(packet) >= 128):
            raise Exception("packet larger than 127 currently not supported")

        header = [SYNC_BYTE, len(packet)]
        header.append(calc_crc8(CRC8_INIT, header))

        self._output.write_bytes(header)
        self._output.write_bytes(packet)


class Channel(PacketWriter):
    _outbound_seq_no = 0
    _interface_definition_crc = bytearray(2)
    _expected_acks = {}

    def __init__(self, input, output):
        """
        Params:
        input: A PacketReader where this channel will source packets from on
               demand. Alternatively packets can be provided to this channel
               directly by calling write_packet on this instance.
        output: A PacketWriter where this channel will put outgoing packets.
        """
        self._input = input
        self._output = output

    def remote_endpoint_operation(self, endpoint_id, input, expect_ack, output_length):
        if (len(input) >= 128):
            raise Exception("packet larger than 127 currently not supported")

        if (expect_ack):
            endpoint_id |= 0x8000

        self._outbound_seq_no = ((self._outbound_seq_no + 1) & 0x7fff)
        seq_no = self._outbound_seq_no
        packet = struct.pack('<HHH', seq_no, endpoint_id, output_length)
        packet = packet + input

        crc16 = calc_crc16(CRC16_INIT, packet)
        if (endpoint_id == 0):
            crc16 = calc_crc16(crc16, struct.pack('<H', PROTOCOL_VERSION))
        else:
            crc16 = calc_crc16(crc16, self._interface_definition_crc)

        packet = packet + struct.pack('<H', crc16)

        if (expect_ack):
            self._expected_acks[seq_no] = False

        self._output.write_packet(packet)

        if (expect_ack):
            # Read and process packets until we get an ack
            # TODO: add timeout
            # TODO: support I/O driven reception (wait on semaphore)
            while (not self._expected_acks[seq_no]):
                self.write_packet(self._input.read_packet())

            if (expect_ack):
                self._expected_acks.pop(seq_no, None)
    
    def write_packet(self, packet):
        if (len(packet) < 4):
            raise Exception("packet too short")

        # calculate CRC for later validation
        crc16 = calc_crc16(CRC16_INIT, packet[:-2])

        seq_no = struct.unpack('<H', packet[0:2])[0]

        if (seq_no & 0x8000):
            if (calc_crc16(crc16, struct.pack('<HBB', PROTOCOL_VERSION, packet[-2], packet[-1]))):
                raise Exception("CRC16 mismatch")

        else:
            print("endpoint requested")
            # TODO: handle local endpoint operation
