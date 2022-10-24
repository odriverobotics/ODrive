.. _can-protocol:

================================================================================
CAN Protocol
================================================================================

.. contents::
   :depth: 1
   :local:

This document describes the CAN Protocol.  For examples of usage, check out our :ref:`CAN Guide! <can-guide>`


Configuring ODrive for CAN
--------------------------------------------------------------------------------

Configuration of the CAN parameters should be done via USB before putting the device on the bus.

To set the desired baud rate, use :code:`<odrv>.can.config.baud_rate = <value>`.

Each axis looks like a separate node on the bus. 
Thus, they both have the two properties :code:`can_node_id` and :code:`can_node_id_extended`. 
The node ID can be from 0 to 63 (0x3F) inclusive, or, if extended CAN IDs are used, from 0 to 16777215 (0xFFFFFF). 
If you want to connect more than one ODrive on a CAN bus, you must set different node IDs for the second ODrive or they will conflict and crash the bus.

Example Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: iPython
        
    odrv0.axis0.config.can.node_id = 3
    odrv0.axis1.config.can.node_id = 1
    odrv0.can.config.baud_rate = 500000
    odrv0.save_configuration()
    odrv0.reboot()

Transport Protocol
--------------------------------------------------------------------------------

We've implemented a very basic CAN protocol that we call "CAN Simple" to get users going with ODrive. 
This protocol is sufficiently abstracted that it is straightforward to add other protocols such as CANOpen, J1939, or Fibre over ISO-TP in the future. 
Unfortunately, implementing those protocols is a lot of work, and we wanted to give users a way to control ODrive's basic functions via CAN sooner rather than later.

CAN Frame
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

At its most basic, the CAN Simple frame looks like this:

* Upper 6 bits - Node ID - max 0x3F (or 0xFFFFFF when using extended CAN IDs)
* Lower 5 bits - Command ID - max 0x1F

To understand how the Node ID and Command ID interact, let's look at an example

The 11-bit Arbitration ID is setup as follows:

:code:`can_id = axis_id << 5 | cmd_id`

For example, an Axis ID of :code:`0x01` with a command of :code:`0x0C` would be result in :code:`0x2C`:

:code:`0x01 << 5 | 0x0C = 0x2C`

Messages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table:: CAN Messages
   :file: figures/can-protocol.csv
   :header-rows: 1
.. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All multibyte values are little endian (aka Intel format, aka least significant byte first).

.. note::

    * These messages are call & response. The Master node sends a message with the RTR bit set, and the axis responds with the same ID and specified payload.  
    * These CANOpen messages are reserved to avoid bus collisions with CANOpen devices.  They are not used by CAN Simple.  
    * These messages can be sent to either address on a given ODrive board.
	* You must send a valid GPIO pin number in the first byte to recieve coreect ADC voltage feedback. Since you're both sending and receiving data the RTR bit must be set to false.


Cyclic Messages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Cyclic messages are sent by ODrive on a timer without a request. As of firmware verion `0.5.4`, the Cyclic messsages are:

.. list-table::
   :widths: 25 25 25 50
   :header-rows: 1

   * - ID
     - Name
     - Config Variable
     - Default Rate (ms)
   * - 0x01
     - Heartbeat
     - `heartbeat_rate_ms`
     - 100
   * - 0x09
     - Get Encoder Estimates
     - `encoder_rate_ms`
     - 10
   * - 0x03
     - Get Motor Error
     - `motor_error_rate_ms`
     - 0
   * - 0x04
     - Get Encoder Error
     - `encoder_error_rate_ms`
     - 0
   * - 0x1D
     - Get Controller Error
     - `controller_error_rate_ms`
     - 0
   * - 0x05
     - Get Sensorless Error
     - `sensorless_error_rate_ms`
     - 0
   * - 0x0A
     - Get Encoder Count
     - `encoder_count_rate_ms`
     - 0
   * - 0x14
     - Get Iq
     - `iq_rate_ms`
     - 0
   * - 0x15
     - Get Sensorless Estimates
     - `sensorless_rate_ms`
     - 0
   * - 0x17
     - Get Bus Voltage Current
     - `bus_vi_rate_ms`
     - 0


.. ID | Name | Rate (ms)
.. --:    | :--  | :--
.. 0x001 | ODrive Heartbeat Message | 100
.. 0x009 | Encoder Estimates | 10

.. Command ID | Message Name
.. :-- | :-- | :--
 .. 0x01 | `heartbeat_rate_ms` | Heartbeat
 .. 0x09 | `encoder_rate_ms` | Get Encoder Estimates
 .. 0x03 | `motor_error_rate_ms` | Get Motor Error
 .. 0x04 | `encoder_error_rate_ms` | Get Encoder Error
 .. 0x1D | `controller_error_rate_ms` | Get Controller Error
 .. 0x05 | `sensorless_error_rate_ms` | Get Sensorless Error
 .. 0x0A | `encoder_count_rate_ms` | Get Encoder Count
 .. 0x14 | `iq_rate_ms` | Get Iq
 .. 0x15 | `sensorless_rate_ms` | Get Sensorless Estimates
 .. 0x17 | `bus_vi_rate_ms` | Get Bus Voltage Current

These can be configured for each axis, see e.g. :code:`axis.config.can`.


Interoperability with CANopen
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can deconflict with CANopen like this:

* :code:`odrv0.axis0.config.can.node_id = 0x010` - Reserves messages 0x200 through 0x21F  
* :code:`odrv0.axis1.config.can.node_id = 0x018` - Reserves messages 0x300 through 0x31F

It may not be obvious, but this allows for some compatibility with CANOpen.  
Although the address space 0x200 and 0x300 correspond to receive PDO base addresses, we can guarantee they will not conflict if all CANopen node IDs are >= 32.  E.g.:

* CANopen nodeID = 35 = 0x23
* Receive PDO 0x200 + nodeID = 0x223, which does not conflict with the range [0x200 : 0x21F]

Be careful that you don't assign too many nodeIDs per PDO group.  Four CAN Simple nodes (32*4) is all of the available address space of a single PDO.  
If the bus is strictly ODrive CAN Simple nodes, a simple sequential Node ID assignment will work fine.

