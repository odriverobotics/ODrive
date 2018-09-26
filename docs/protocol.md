
# ODrive Communication Protocol #

Communicating with an ODrive consists of a series of endpoint operations.
An endpoint can theoretically be any kind data serialized in any way.
There is a default serialization implementation for POD types; for custom types
you must (de)serialize yourself. In the future we may provide a default serializer
for structs.
The available endpoints can be enumerated by reading the JSON from endpoint 0
and can theoretically be different for each communication interface (they are not in practice).

Each endpoint operation can send bytes to one endpoint (referenced by its ID)
and at the same time receive bytes from the same endpoint. The semantics of
these payloads are specific to each endpoint's type, the name of which is
indicated in the JSON.

For instance an int32 endpoint's input and output is a 4 byte little endian
representation. In general the convention for combined read/write requests is
_exchange_, i.e. the returned value is the old value. Custom endpoint handlers
may be non-compliant.

There is a packet based version and a stream based variant of the protocol. Each
variant is employed as appropriate. For instance USB runs the packet based variant
by default while UART runs the stream based variant.


## Packet format ##
We will call the ODrive "server" and the PC "client". A request is a message
from the PC to the ODrive and a response is a message from the ODrive to the
PC.

Each request-response transaction corresponds to a single endpoint operation.

__Request__

  - __Bytes 0, 1__ Sequence number, MSB = 0
      - Currently the server does not care about ordering and does not filter resent messages.
  - __Bytes 2, 3__ Endpoint ID
      - The IDs of all endpoints can be obtained from the JSON definition. The JSON definition can be obtained by reading from endpoint 0.
    If (and only if) the MSB is set to 1 the client expects a response for this request.
  - __Bytes 4, 5__ Expected response size
      - The number of bytes that should be returned to the client. If the client doesn't need any response data, it can set this value to 0. The operation will still be acknowledged if the
    MSB in EndpointID is set.
  - __Bytes 6 to N-3__ Payload
      - The length of the payload is determined by the total packet size. The format of the payload depends on the endpoint type. The endpoint type can be obtained from the JSON definition.
  - __Bytes N-2, N-1__
      - For endpoint 0: Protocol version (currently 1). A server shall ignore packets with other values.
      - For all other endpoints: The CRC16 calculated over the JSON definition. The CRC16 init value is the protocol version (currently 1). A server shall ignore packets that set this field incorrectly. See protocol.hpp for CRC details.

__Response__

  - __Bytes 0, 1__ Sequence number, MSB = 1
      - The sequence number of the request to which this is the response.
  - __Bytes 2, 3__ Payload
      - The length of the payload tends to be equal to the number of expected bytes as indicated
    in the request. The server must not expect the client to accept more bytes than it requested.

## Stream format ##
The stream based format is just a wrapper for the packet format.

  - __Byte 0__ Sync byte `0xAA`
  - __Bytes 1, 2__ Packet length
      - Currently both parties shall only emit and accept values of 0 through 127.
  - __Bytes 3__ CRC8 of bytes 0 through 2
      - See protocol.hpp for CRC details.
  - __Bytes 4 to N-3__ Packet
  - __Bytes N-2, N-1__ CRC16
      - See protocol.hpp for CRC details.
