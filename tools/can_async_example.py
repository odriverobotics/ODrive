import can

bus1 = can.interface.Bus('can0', bustype='virtual')
bus2 = can.interface.Bus('can0', bustype='virtual')

msg1 = can.Message(arbitration_id=0xabcde, data=[1,2,3])
bus1.send(msg1)
msg2 = bus2.recv()

print(hex(msg1.arbitration_id))
print(hex(msg2.arbitration_id))
assert msg1.arbitration_id == msg2.arbitration_id