
import test_runner

import time
import math
import os
import numpy as np

from odrive.enums import *
from odrive.utils import set_motor_thermistor_coeffs
from test_runner import *

def test_assert_not_stale(data):
    if len(set(data)) <= 2:
        raise TestFailed(f"data seems stale: {set(data)}")

class TestVbusVoltage():
    """
    Checks if the onboard DC voltage sensor measures the expected test rig
    voltage without too much noise.
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            yield odrive, None

    def run_test(self, odrive: ODriveComponent, logger: Logger):
        # TODO: vary voltage with programmable power supply
        expected_voltage = float(odrive.yaml['vbus-voltage'])

        data = record_log(lambda: [odrive.handle.vbus_voltage], duration=5.0)
        slope, offset, fitted_curve = fit_line(data[:,(0,1)])
        test_assert_not_stale(data[:,1])
        test_assert_eq(offset, expected_voltage, accuracy=0.02)
        test_assert_eq(slope, 0, range=0.01)
        test_curve_fit(data[:,(0,1)], fitted_curve, max_mean_err = expected_voltage * 0.01, inlier_range = expected_voltage * 0.02, max_outliers = len(data[:,0]) * 0.01)

class TestOnboardTempSensors():
    """
    Checks if the onboard FET thermistor measures a plausible non-stale
    temperature without too much noise.
    """

    def get_test_cases(self, testrig: TestRig):
        for axis in testrig.get_components(ODriveAxisComponent):
            yield axis, None

    def run_test(self, axis: ODriveAxisComponent, logger: Logger):
        data = record_log(lambda: [axis.handle.motor.fet_thermistor.temperature], duration=5.0)
        slope, offset, fitted_curve = fit_line(data[:,(0,1)])
        test_assert_not_stale(data[:,1])
        test_assert_within(offset, 10, 80) # fet temp expected to be between 10°C and 80°C
        test_assert_eq(slope, 0, range=0.01)
        test_curve_fit(data[:,(0,1)], fitted_curve, max_mean_err = 0.2, inlier_range = 0.5, max_outliers = len(data[:,0]) * 0.01)


teensy_code_template = """
void setup() {
  analogWriteResolution(10);
  // base clock of the PWM timer is 150MHz (on Teensy 4.0)
  int freq = 150000000/1024; // ~146.5kHz PWM frequency
  analogWriteFrequency({analog_out}, freq);
}

const float r_load = 1000.0f;
const float r_testrig = 150.0f; // resistance between Teensy and ODrive
const float r_25 = 10000.0f;
const float beta = 3434;
const float r_inf = r_25 * exp(-beta/(25.0f + 273.15f));

const float t_min = 20.0f;
const float t_max = 100.0f;
const float period_ms = 1000.0f;

int time_ms = 0;
void loop() {
  time_ms = (time_ms + 1) % 1000;

  // simulated temperature
  float temp = t_min + time_ms / period_ms * (t_max - t_min) + 273.15f;

  // simulated thermistor resistance
  float resistance = r_inf * exp(beta/temp);

  // actual output voltage (relative to Vcc)
  float output = (resistance - r_testrig) / (resistance + r_load);
  
  analogWrite({analog_out}, 1023 * min(max(output, 0.0f), 1.0f));
  delay(1);
}
"""

class TestOffboardTempSensors():
    """
    Verifies the motor thermistor input.
    
    The PWM output of a Teensy is used to simulate a thermistor that goes from
    20°C to 100°C in one second.
    On ODrive v4.x the motor temp input must be connected to the Teensy GPIO
    through a 150 Ohm resistor. The onboard low pass filter is used to converts
    the Teensy's PWM to an analog signal.
    """
    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            if odrive.yaml['board-version'].startswith('v4.'):
                teensy_gpios = testrig.get_connected_components(odrive.gpio2, TeensyGpio)
                yield AnyTestCase(*[(odrive.axes[0], teensy_gpio.parent, teensy_gpio, tf) for teensy_gpio, tf in teensy_gpios])

    def run_test(self, axis: ODriveAxisComponent, teensy: TeensyComponent, teensy_gpio: TeensyGpio, logger: Logger):
        code = teensy_code_template.replace("{analog_out}", str(teensy_gpio.num))
        teensy.compile_and_program(code)

        set_motor_thermistor_coeffs(axis.handle, Rload=1000, R_25=10000, Beta=3434,
          Tmin=0, Tmax=140, thermistor_bottom=True)
        axis.parent.handle.config.gpio2_mode = GPIO_MODE_ANALOG_IN
        axis.parent.save_config_and_reboot()

        data = record_log(lambda: [axis.handle.motor.motor_thermistor.temperature], duration=5.0)
        slope, offset, fitted_curve = fit_sawtooth(data[:,(0,1)], 20, 100)
        test_assert_eq(slope, (100 - 20) / 1.0, accuracy=0.005)
        test_curve_fit(data[:,(0,1)], fitted_curve, max_mean_err = 3.0, inlier_range = 6.0, max_outliers = len(data[:,0]) * 0.02)

tests = [
  TestVbusVoltage(),
  TestOnboardTempSensors(),
  TestOffboardTempSensors()
]

if __name__ == '__main__':
    test_runner.run(tests)
