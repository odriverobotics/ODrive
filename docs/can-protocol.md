# CAN Protocol

This document describes the CAN Protocol.  For examples of usage, check out our [CAN Guide!](can-guide.md)


---
## Configuring ODrive for CAN
Configuration of the CAN parameters should be done via USB before putting the device on the bus.

To set the desired baud rate, use `<odrv>.can.config.baud_rate = <value>`.

Each axis looks like a separate node on the bus. Thus, they both have the two properties `can_node_id` and `can_node_id_extended`. The node ID can be from 0 to 63 (0x3F) inclusive, or, if extended CAN IDs are used, from 0 to 16777215 (0xFFFFFF). If you want to connect more than one ODrive on a CAN bus, you must set different node IDs for the second ODrive or they will conflict and crash the bus.

### Example Configuration

```
odrv0.axis0.config.can_node_id = 3
odrv0.axis1.config.can_node_id = 1
odrv0.can.config.baud_rate = 500000
odrv0.save_configuration()
odrv0.reboot()
```

---
## Transport Protocol
We've implemented a very basic CAN protocol that we call "CAN Simple" to get users going with ODrive.  This protocol is sufficiently abstracted that it is straightforward to add other protocols such as CANOpen, J1939, or Fibre over ISO-TP in the future.  Unfortunately, implementing those protocols is a lot of work, and we wanted to give users a way to control ODrive's basic functions via CAN sooner rather than later.

### CAN Frame
At its most basic, the CAN Simple frame looks like this:

* Upper 6 bits - Node ID - max 0x3F (or 0xFFFFFF when using extended CAN IDs)
* Lower 5 bits - Command ID - max 0x1F

To understand how the Node ID and Command ID interact, let's look at an example

The 11-bit Arbitration ID is setup as follows:

`can_id = axis_id << 5 | cmd_id`

For example, an Axis ID of `0x01` with a command of `0x0C` would be result in `0x2C`:

`0x01 << 5 | 0x0C = 0x2C`

### Messages

CMD ID | Name | Sender | Signals | Start byte | Signal Type | Bits | Factor | Offset
--:    | :--  | :--  | :-- | :-- | :-- | :-- | :-- | :--
0x000 | CANOpen NMT Message\*\* | Master | - | - | - | - | - | -
0x001 | ODrive Heartbeat Message | Axis | Axis Error<br>Axis Current State<br>Controller Status | 0<br>4<br>7 | Unsigned Int<br>Unsigned Int<br>Bitfield | 32<br>8<br>8 | -<br>-<br>- | -<br>-<br>-
0x002 | ODrive Estop Message | Master | - | - | - | - | - | -
0x003 | Get Motor Error\* | Axis  | Motor Error | 0 | Unsigned Int | 64 | 1 | 0
0x004 | Get Encoder Error\*  | Axis | Encoder Error | 0 | Unsigned Int | 32 | 1 | 0
0x005 | Get Sensorless Error\* | Axis | Sensorless Error | 0 | Unsigned Int | 32 | 1 | 0
0x006 | Set Axis Node ID | Master | Axis CAN Node ID | 0 | Unsigned Int | 32 | 1 | 0
0x007 | Set Axis Requested State | Master | Axis Requested State | 0 | Unsigned Int | 32 | 1 | 0
0x008 | Set Axis Startup Config | Master | - Not yet implemented - | - | - | - | - | -
0x009 | Get Encoder Estimates\* | Master | Encoder Pos Estimate<br>Encoder Vel Estimate | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x00A | Get Encoder Count\* | Master | Encoder Shadow Count<br>Encoder Count in CPR | 0<br>4 | Signed Int<br>Signed Int | 32<br>32 | 1<br>1 | 0<br>0
0x00B | Set Controller Modes | Master | Control Mode<br>Input Mode | 0<br>4 | Signed Int<br>Signed Int | 32<br>32 | 1<br>1 | 0<br>0
0x00C | Set Input Pos | Master | Input Pos<br>Vel FF<br>Torque FF | 0<br>4<br>6 | IEEE 754 Float<br>Signed Int<br>Signed Int | 32<br>16<br>16 | 1<br>0.001<br>0.001 | 0<br>0<br>0
0x00D | Set Input Vel | Master | Input Vel<br>Torque FF | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x00E | Set Input Torque | Master | Input Torque | 0 |  IEEE 754 Float | 32 | 1 | 0
0x00F | Set Limits | Master | Velocity Limit<br>Current Limit | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br> | 1<br>1 | 0<br>0
0x010 | Start Anticogging | Master | - | - | - | - | - | -
0x011 | Set Traj Vel Limit | Master | Traj Vel Limit | 0 | IEEE 754 Float | 32 | 1 | 0
0x012 | Set Traj Accel Limits | Master | Traj Accel Limit<br>Traj Decel Limit | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x013 | Set Traj Inertia | Master | Traj Inertia | 0 | IEEE 754 Float | 32 | 1 | 0
0x014 | Get IQ\* | Axis | Iq Setpoint<br>Iq Measured | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x015 | Get Sensorless Estimates\* | Master | Sensorless Pos Estimate<br>Sensorless Vel Estimate | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x016 | Reboot ODrive | Master\*\*\* | - | - | - | - | - | -
0x017 | Get Vbus Voltage | Master\*\*\* | Vbus Voltage | 0 | IEEE 754 Float | 32 | 1 | 0
0x018 | Clear Errors | Master | - | - | - | - | - | -
0x019 | Set Linear Count | Master | Position | 0 | Signed Int | 32 | 1 | 0
0x700 | CANOpen Heartbeat Message\*\* | Slave | - | -  | - | - | - | -
-|-|-|----------------------------------|-|--------------------|-|-|-

All multibyte values are little endian (aka Intel format, aka least significant byte first).

\* Note: These messages are call & response.  The Master node sends a message with the RTR bit set, and the axis responds with the same ID and specified payload.  
\*\* Note:  These CANOpen messages are reserved to avoid bus collisions with CANOpen devices.  They are not used by CAN Simple.  
\*\*\* Note:  These messages can be sent to either address on a given ODrive board.

---

### Interoperability with CANopen
You can deconflict with CANopen like this:

`odrv0.axis0.config.can.node_id = 0x010` - Reserves messages 0x200 through 0x21F  
`odrv0.axis1.config.can.node_id = 0x018` - Reserves messages 0x300 through 0x31F

It may not be obvious, but this allows for some compatibility with CANOpen.  Although the address space 0x200 and 0x300 correspond to receive PDO base addresses, we can guarantee they will not conflict if all CANopen node IDs are >= 32.  E.g.:

CANopen nodeID = 35 = 0x23
Receive PDO 0x200 + nodeID = 0x223, which does not conflict with the range [0x200 : 0x21F]

Be careful that you don't assign too many nodeIDs per PDO group.  Four CAN Simple nodes (32*4) is all of the available address space of a single PDO.  If the bus is strictly ODrive CAN Simple nodes, a simple sequential Node ID assignment will work fine.

