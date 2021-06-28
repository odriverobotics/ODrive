import can

bus = can.Bus("can0", bustype="socketcan")
axisID = 0x1

print("Requesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE (0x03) on axisID: " + str(axisID))
msg = can.Message(arbitration_id=axisID << 5 | 0x07, data=[3, 0, 0, 0, 0, 0, 0, 0], dlc=8, is_extended_id=False)
print(msg)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent!  Please verify can0 is working first")

print("Waiting for calibration to finish...")
# Read messages infinitely and wait for the right ID to show up
while True:
    msg = bus.recv()
    if msg.arbitration_id == (axisID << 5 | 0x01):
        current_state = msg.data[4] | msg.data[5] << 8 | msg.data[6] << 16 | msg.data[7] << 24
        if current_state == 0x1:
            print("\nAxis has returned to Idle state.")
            break

for msg in bus:
    if(msg.arbitration_id == (axisID << 5 | 0x01)):
        errorCode = msg.data[0] | msg.data[1] << 8 | msg.data[2] << 16 | msg.data[3] << 24
        print("\nReceived Axis heartbeat message:")
        if errorCode == 0x0:
            print("No errors")
        else:
            print("Axis error!  Error code: "+str(hex(errorCode)))
        break

print("\nPutting axis",axisID,"into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
msg = can.Message(arbitration_id=axisID << 5 | 0x07, data=[8, 0, 0, 0, 0, 0, 0, 0], dlc=8, is_extended_id=False)
print(msg)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent!")

for msg in bus:
    if msg.arbitration_id == (axisID << 5 | 0x01):
        print("\nReceived Axis heartbeat message:")
        if msg.data[4] == 0x8:
            print("Axis has entered closed loop")
        else:
            print("Axis failed to enter closed loop")
        break