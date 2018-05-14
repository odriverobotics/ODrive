
import sys
import socket
import fibre.protocol
from fibre.core import object_from_channel

def noprint(x):
  pass

class UDPTransport(fibre.protocol.PacketSource, fibre.protocol.PacketSink):
  def __init__(self, dest_addr, dest_port, printer):
    # TODO: FIXME: use IPv6
    # Problem: getaddrinfo fails if the resolver returns an
    # IPv4 address, but we are using AF_INET6
    #family = socket.AF_INET6 if socket.has_ipv6 else socket.AF_INET
    family = socket.AF_INET
    self.sock = socket.socket(family, socket.SOCK_DGRAM)
    # TODO: Determine the right address to use from the list
    self.target = socket.getaddrinfo(dest_addr,dest_port, family)[0][4]

  def process_packet(self, buffer):
    self.sock.sendto(buffer, self.target)

  def get_packet(self, deadline):
    # TODO: implement deadline
    data, addr = self.sock.recvfrom(1024)
    return data



def channel_from_udp_destination(dest_addr, dest_port, printer=noprint, device_stdout=noprint):
    """
    Inits a Fibre Protocol channel from a UDP hostname and port.
    """
    udp_transport = fibre.udp_transport.UDPTransport(dest_addr, dest_port, printer)
    return fibre.protocol.Channel(
            "UDP device {}:{}".format(dest_addr, dest_port),
            udp_transport, udp_transport,
            device_stdout)

def open_udp(destination, printer=noprint, device_stdout=noprint):
  try:
    dest_addr = ':'.join(destination.split(":")[:-1])
    dest_port = int(destination.split(":")[-1])
  except (ValueError, IndexError):
    raise Exception('"{}" is not a valid UDP destination. The format should be something like "localhost:1234".'
                    .format(destination))
  channel = channel_from_udp_destination(dest_addr, dest_port)
  udp_device = object_from_channel(channel, printer)
  return udp_device
