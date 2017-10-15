"""
Provides classes for rudimentary testing if you don't have an ODrive.
"""

JSONDescriptor = '''
[
    { "name": "vbus_voltage", "id": 0, "type": "float", "mode": "r" },
    { "name": "elec_rad_per_enc", "id": 1, "type": "float", "mode": "r" },
    { "name": "motor0", "id": 2, "type": "tree", "content": [
        { "name": "pos_setpoint", "id": 3, "type": "float" },
        { "name": "pos_gain", "id": 4, "type": "float" },
        { "name": "vel_setpoint", "id": 5, "type": "float" },
        { "name": "current_control", "id": 6, "type": "tree", "content": [
            { "name": "v_current_control_integral_d", "id": 7, "type": "float" },
            { "name": "v_current_control_integral_q", "id": 8, "type": "float" },
            { "name": "Ibus", "id": 9, "type": "float" }
        ]},
        { "name": "encoder", "id": 10, "type": "tree", "content": [
            { "name": "phase", "id": 11, "type": "float" },
            { "name": "pll_pos", "id": 12, "type": "float" },
            { "name": "pll_vel", "id": 13, "type": "float" }
        ]}
    ]}
]
'''


class MockDevice(object):
    """
    Implements a mock device that supports the 'r', 'w' and 'j' commands
    """

    _rx_buf = ""
    _tx_buf = ""
    _values = ["0.0"] * 13

    def execute_cmd(self, cmd):
        if cmd[0] == 'j':
            self._tx_buf = self._tx_buf + JSONDescriptor.replace('\n', '') + '\n'
        elif cmd[0] == 'r':
            [_, id] = cmd.split()
            self._tx_buf = self._tx_buf + self._values[int(id)] + '\n'
        elif cmd[0] == 'w':
            [_, id, val] = cmd.split()
            self._values[int(id)] = val
        else:
            # TODO: report error
            pass

    def send(self, buffer):
        """
        Sends a string to the virtual device
        """
        self._rx_buf = self._rx_buf + buffer
        while '\n' in self._rx_buf:
            [cmd, _, self._rx_buf] = self._rx_buf.partition('\n')
            self.execute_cmd(cmd)

    def receive_until(self, char):
        """
        Reads a string from the virtual device's TX buffer
        """
        [result, char, self._tx_buf] = self._tx_buf.partition(char)
        return result + char
