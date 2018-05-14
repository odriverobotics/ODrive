
import sys
import socket
import time
import fibre.protocol
from fibre.core import object_from_channel

def noprint(x):
  pass

class TCPTransport(fibre.protocol.StreamSource, fibre.protocol.StreamSink):
  def __init__(self, dest_addr, dest_port, printer):
    # TODO: FIXME: use IPv6
    # Problem: getaddrinfo fails if the resolver returns an
    # IPv4 address, but we are using AF_INET6
    #family = socket.AF_INET6 if socket.has_ipv6 else socket.AF_INET
    family = socket.AF_INET
    self.sock = socket.socket(family, socket.SOCK_STREAM)
    # TODO: Determine the right address to use from the list
    self.target = socket.getaddrinfo(dest_addr, dest_port, family)[0][4]
    # TODO: this blocks until a connection is established, or the system cancels it
    self.sock.connect(self.target)

  def process_bytes(self, buffer):
    self.sock.send(buffer)

  def get_bytes(self, n_bytes, deadline):
    """
    Returns n bytes unless the deadline is reached, in which case the bytes
    that were read up to that point are returned. If deadline is None the
    function blocks forever. A deadline before the current time corresponds
    to non-blocking mode.
    """
    # convert deadline to seconds (floating point)
    deadline = None if deadline is None else max(deadline - time.monotonic(), 0)
    self.sock.settimeout(deadline)
    try:
      data = self.sock.recv(n_bytes, socket.MSG_WAITALL) # receive n_bytes
      return data
    except TimeoutError:
      # if we got a timeout data will still be none, so we call recv again
      # this time in non blocking state and see if we can get some data
      return self.sock.recv(n_bytes, socket.MSG_DONTWAIT)

  def get_bytes_or_fail(self, n_bytes, deadline):
    result = self.get_bytes(n_bytes, deadline)
    if len(result) < n_bytes:
      raise fibre.protocol.TimeoutException("expected {} bytes but got only {}".format(n_bytes, len(result)))
    return result



def channel_from_tcp_destination(dest_addr, dest_port, printer=noprint, device_stdout=noprint):
    """
    Inits a Fibre Protocol channel from a TCP hostname and port.
    """
    tcp_transport = fibre.tcp_transport.TCPTransport(dest_addr, dest_port, printer)
    stream2packet_input = fibre.protocol.PacketFromStreamConverter(tcp_transport, device_stdout)
    packet2stream_output = fibre.protocol.StreamBasedPacketSink(tcp_transport)
    return fibre.protocol.Channel(
            "TCP device {}:{}".format(dest_addr, dest_port),
            stream2packet_input, packet2stream_output,
            device_stdout)

def open_tcp(destination, printer=noprint, device_stdout=noprint):
  try:
    dest_addr = ':'.join(destination.split(":")[:-1])
    dest_port = int(destination.split(":")[-1])
  except (ValueError, IndexError):
    raise Exception('"{}" is not a valid TCP destination. The format should be something like "localhost:1234".'
                    .format(destination))
  channel = channel_from_tcp_destination(dest_addr, dest_port)
  tcp_device = object_from_channel(channel, printer)
  return tcp_device
