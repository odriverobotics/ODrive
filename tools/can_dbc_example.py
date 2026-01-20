import math
import can
import cantools
import time

db = cantools.database.load_file("odrive-cansimple.dbc")
# print(db)

# bus = can.Bus("vcan0", bustype="virtual")
bus = can.Bus("can0", bustype="socketcan")
axisID = 0x1
axis="Axis1"
print("\nRequesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE (0x03) on axisID: " + str(axisID))
msg = db.get_message_by_name(f'{axis}_Set_Axis_State')
data = msg.encode({'Axis_Requested_State': 0x03})
msg = can.Message(arbitration_id=msg.frame_id | axisID << 5, is_extended_id=False, data=data)
print(db.decode_message(f'{axis}_Set_Axis_State', msg.data))
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
    if msg.arbitration_id == ((axisID << 5) | db.get_message_by_name(f'{axis}_Heartbeat').frame_id):
        current_state = db.decode_message(f'{axis}_Heartbeat', msg.data)['Axis_State']
        if current_state == 'IDLE':
            print("\nAxis has returned to Idle state.")
            break

for msg in bus:
    if msg.arbitration_id == ((axisID << 5) | db.get_message_by_name(f'{axis}_Heartbeat').frame_id):
        errorCode = db.decode_message(f'{axis}_Heartbeat', msg.data)['Axis_Error']
        print(errorCode)
        if errorCode == 'NONE':
            print("No errors")
        else:
            print("Axis error!  Error code: "+str(hex(errorCode)))
        break

print("\nPutting axis",axisID,"into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
data = db.encode_message(f'{axis}_Set_Axis_State', {'Axis_Requested_State': 0x08})
msg = can.Message(arbitration_id=0x07 | axisID << 5, is_extended_id=False, data=data)
print(msg)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent!")

for msg in bus:
    if msg.arbitration_id == 0x01 | axisID << 5:
        print("\nReceived Axis heartbeat message:")
        msg = db.decode_message(f'{axis}_Heartbeat', msg.data)
        print(msg)
        if msg['Axis_State'] == 0x8:
            print("Axis has entered closed loop")
        else:
            print("Axis failed to enter closed loop")
        break

target = 0

data = db.encode_message(f'{axis}_Set_Limits', {'Velocity_Limit':10.0, 'Current_Limit':10.0})
msg = can.Message(arbitration_id=axisID << 5 | 0x00F, is_extended_id=False, data=data)
bus.send(msg)

t0 = time.monotonic()
print("\nSweeping velocity between -4 and 4.\nCTRL+C to stop\n")
time.sleep(2)
i = 0
while True:
    try:
        setpoint = 4.0 * math.sin((time.monotonic() - t0)*2)
        # Don't spam the terminal
        if (i % 25 == 0):
            print(f"Velocity: {round(setpoint,2)}")
            i = 0
        data = db.encode_message(f'{axis}_Set_Input_Pos', {'Input_Pos':setpoint, 'Vel_FF':setpoint, 'Torque_FF':0.0})
        msg = can.Message(arbitration_id=axisID << 5 | 0x00C, data=data, is_extended_id=False)
        bus.send(msg)
        i+=1
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
print("\nDone! Setting to AXIS_STATE_IDLE")
data = db.encode_message(f'{axis}_Set_Axis_State', {'Axis_Requested_State': 0x01})
msg = can.Message(arbitration_id=axisID << 5 | 0x07, is_extended_id=False, data=data)
time.sleep(0.5)
bus.send(msg)
while True:
    for msg in bus:
        if msg.arbitration_id == 0x01 | axisID << 5:
            current_state = db.decode_message(f'{axis}_Heartbeat', msg.data)['Axis_State']
            print(current_state)
        if current_state == 'IDLE':
            print("\nAxis has returned to Idle state.")
            break
    break