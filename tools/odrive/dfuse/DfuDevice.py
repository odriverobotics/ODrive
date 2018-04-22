import usb.util
import time

DFU_REQUEST_SEND = 0x21
DFU_REQUEST_RECEIVE = 0xa1

DFU_DETACH    = 0x00
DFU_DNLOAD    = 0x01
DFU_UPLOAD    = 0x02
DFU_GETSTATUS = 0x03
DFU_CLRSTATUS = 0x04
DFU_GETSTATE  = 0x05
DFU_ABORT     = 0x06

# Order is LSB first
def address_to_4bytes(a):
    return [ a % 256, (a >> 8)%256, (a >> 16)%256, (a >> 24)%256 ]

class DfuDevice:
    def __init__(self, device, timeout = None):
        self.dev = device
        self.timeout = timeout
        self.cfg = self.dev[0]
        self.intf = None
        #self.dev.reset()
        self.cfg.set()

    def alternates(self):
        return [(usb.util.get_string(self.dev, intf.iInterface), intf) for intf in self.cfg]

    def set_alternate(self, intf):
        if isinstance(intf, tuple):
            self.intf = intf[1]
        else:
            self.intf = intf
        
        self.intf.set_altsetting()

    def control_msg(self, requestType, request, value, buffer, timeout=None):
        return self.dev.ctrl_transfer(requestType, request, value, self.intf.bInterfaceNumber, buffer, timeout=timeout)

    def detach(self, timeout):
        return self.control_msg(DFU_REQUEST_SEND, DFU_DETACH, timeout, None)
    
    def dnload(self, blockNum, data):
        cnt = self.control_msg(DFU_REQUEST_SEND, DFU_DNLOAD, blockNum, list(data))
        return cnt
    
    def upload(self, blockNum, size):
        return self.control_msg(DFU_REQUEST_RECEIVE, DFU_UPLOAD, blockNum, size)

    def get_status(self, timeout=None):
        status = self.control_msg(DFU_REQUEST_RECEIVE, DFU_GETSTATUS, 0, 6, timeout=timeout)
        return (status[0], status[4], status[1] + (status[2] << 8) + (status[3] << 16), status[5])
    
    def clear_status(self):
        self.control_msg(DFU_REQUEST_SEND, DFU_CLRSTATUS, 0, None)

    def get_state(self):
        return self.control_msg(DFU_REQUEST_RECEIVE, DFU_GETSTATE, 0, 1)[0]

    def abort(self):
        self.control_msg(DFU_REQUEST_RECEIVE, DFU_ABORT, 0, 0)

    def set_address(self, ap):
        return self.dnload(0x0, [0x21] + address_to_4bytes(ap))

    def write(self, block, data):
        return self.dnload(block + 2, data)

    def read(self, block, size):
        return self.upload(block + 2, size)
    
    def erase(self, pa):
        return self.dnload(0x0, [0x41] + address_to_4bytes(pa))

    def leave(self):
        return self.dnload(0x0, []) # Just send an empty data.

    def wait_while_state(self, state, timeout=None):
        if not isinstance(state, (list, tuple)):
            states = (state,)
        else:
            states = state

        status = self.get_status()
        
        while (status[1] in states):
            claimed_timeout = status[2]
            actual_timeout = int(max(timeout or 0, claimed_timeout))
            #print("timeout = %f, claimed = %f" % (timeout, status[2]))
            #time.sleep(timeout)
            status = self.get_status(timeout=actual_timeout)
        
        return status

