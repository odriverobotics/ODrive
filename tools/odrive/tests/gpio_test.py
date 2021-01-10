
import test_runner

import time
import math
import os
import numpy as np

from odrive.enums import *
from odrive.utils import set_motor_thermistor_coeffs
from test_runner import *

teensy_code_template = """
int gpios[] = {{gpios}};

void setup() {
  for (auto num: gpios) {
    pinMode(num, OUTPUT);
    digitalWrite(num, LOW);
  }
}

int active_gpio = 0;
void loop() {
  digitalWrite(gpios[active_gpio], LOW);
  active_gpio = (active_gpio + 1) % (sizeof(gpios) / sizeof(gpios[0]));
  digitalWrite(gpios[active_gpio], HIGH);
  delay(100);
}
"""

class TestInputs():
    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            alternatives = []
            for teensy in testrig.get_components(TeensyComponent):
                # all GPIOs except 8, 12 and 15 are digital input capable
                odrive_gpios = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 16, 17, 18, 19, 20, 21, 22]

                connections = [
                    testrig.net_by_component.get(getattr(odrive, f'gpio{gpio}'), set()).intersection(set(teensy.gpios))
                    for gpio in odrive_gpios
                ]
                disconnected_gpios = [gpio for i, gpio in enumerate(odrive_gpios) if not connections[i]]
                
                if any(disconnected_gpios):
                    logger.debug(f"note: can't run GPIO test on {testrig.get_component_name(odrive)} because GPIOs {disconnected_gpios} are not connected to a Teensy.")
                else:
                    alternatives.append((teensy, tuple(odrive_gpios), tuple([teensy_gpios.pop() for teensy_gpios in connections])))

            yield AnyTestCase(*[(odrive, teensy, odrive_gpios, teensy_gpios, None) for teensy, odrive_gpios, teensy_gpios in alternatives])

    def run_test(self, odrive: ODriveComponent, teensy: TeensyComponent, odrive_gpios: tuple, teensy_gpios: tuple, logger: Logger):
        switch_interval = 0.1 # [s]

        code = teensy_code_template.replace("{gpios}", ", ".join(str(gpio.num) for gpio in teensy_gpios))
        teensy.compile_and_program(code)

        for gpio_num in odrive_gpios:
            setattr(odrive.handle.config, f'gpio{gpio_num}_mode', GPIO_MODE_DIGITAL)
        odrive.save_config_and_reboot()

        mask = sum((1 << num) for num in odrive_gpios)

        # Wait for a change in the GPIO states (but at most 3x the expected interval)
        initial_state = odrive.handle.get_gpio_states() & mask
        for i in range(30):
            if odrive.handle.get_gpio_states() & mask != initial_state:
                break
            time.sleep(0.1 * switch_interval)

        # Sync to the middle of the GPIO switch frame
        time.sleep(0.5 * switch_interval)

        # Take one sample for each switch frame
        meas = []
        for _ in range(len(odrive_gpios)):
            meas.append(odrive.handle.get_gpio_states() & mask)
            time.sleep(1.0 * switch_interval)

        expected = [(1 << num) for num in odrive_gpios]

        # Find the best rotation of the measurement array
        best_match = None
        best_match_val = -1
        for _ in range(len(odrive_gpios)):
            meas = meas[1:] + [meas[0]]
            similarity = sum([m == expected[i] for i, m in enumerate(meas)])
            if similarity > best_match_val:
                best_match = meas
                best_match_val = similarity

        # Check if all values match
        test_assert_eq(best_match, expected)


tests = [
  TestInputs()
]

if __name__ == '__main__':
    test_runner.run(tests)
