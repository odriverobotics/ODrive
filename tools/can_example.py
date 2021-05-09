import can

bus = can.interface.Bus('can0', bustype='socketcan')
axisID = 0x1 << 5
msgID = axisID | 0x7

msg = can.Message(arbitration_id=msgID, data=[0, 0, 0, 0, 3, 0, 0, 0], dlc=8)
print(msg)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent!")