
import test_runner

import time
from math import pi
import os

from fibre.utils import Logger
from odrive.enums import *
from test_runner import *


teensy_code_template = """
void setup() {
  pinMode({enc_a}, OUTPUT);
  pinMode({enc_b}, OUTPUT);
}

int cpr = 8192;
int rpm = 30;

// the loop routine runs over and over again forever:
void loop() {
  int microseconds_per_count = (1000000 * 60 / cpr / rpm);

  for (;;) {
    digitalWrite({enc_a}, HIGH);
    delayMicroseconds(microseconds_per_count);
    digitalWrite({enc_b}, HIGH);
    delayMicroseconds(microseconds_per_count);
    digitalWrite({enc_a}, LOW);
    delayMicroseconds(microseconds_per_count);
    digitalWrite({enc_b}, LOW);
    delayMicroseconds(microseconds_per_count);
  }
}

"""

teensy_code_template2 = """
void setup() {
  analogWriteResolution(10);
  int freq = 150000000/1024; // ~146.5kHz PWM frequency
  analogWriteFrequency({enc_sin}, freq);
  analogWriteFrequency({enc_cos}, freq);
}

int rpm = 60;
float pos = 0;

void loop() {
  pos += 0.001f * ((float)rpm / 60.0f);
  if (pos > 1.0f)
    pos -= 1.0f;
  analogWrite({enc_sin}, (int)(512.0f + 512.0f * sin(2.0f * M_PI * pos)));
  analogWrite({enc_cos}, (int)(512.0f + 512.0f * cos(2.0f * M_PI * pos)));
  delay(1);
}
"""


class TestEncoderBase():
    """
    Base class for encoder tests.
    TODO: incremental encoder doesn't use this yet.

    All encoder tests expect the encoder to run at a constant velocity.
    This can be achieved by generating an encoder signal with a Teensy.

    During 5 seconds, several variables are recorded and then compared against
    the expected waveform. This is either a straight line, a sawtooth function
    or a constant.
    """

    def run_generic_encoder_test(self, encoder, true_cpr, true_rps):
        encoder.config.cpr = true_cpr
        true_cps = true_cpr * true_rps

        logger.debug("Recording log...")
        data = []
        start = time.monotonic()
        encoder.set_linear_count(0) # prevent numerical errors
        while time.monotonic() - start < 5.0:
            data.append((
                time.monotonic() - start,
                encoder.shadow_count,
                encoder.count_in_cpr,
                encoder.phase,
                encoder.pos_estimate,
                encoder.pos_cpr,
                encoder.vel_estimate,
            ))
        
        data = np.array(data)

        short_period = (abs(1 / true_rps) < 5.0)
        reverse = (true_rps < 0)

        # encoder.shadow_count
        slope, offset, fitted_curve = fit_line(data[:,(0,1)])
        test_assert_eq(slope, true_cps, accuracy=0.005)
        test_curve_fit(data[:,(0,1)], fitted_curve, max_mean_err = true_cpr * 0.01, inlier_range = true_cpr * 0.01, max_outliers = len(data[:,0]) * 0.02)

        # encoder.count_in_cpr
        slope, offset, fitted_curve = fit_sawtooth(data[:,(0,2)], true_cpr if reverse else 0, 0 if reverse else true_cpr)
        test_assert_eq(slope, true_cps, accuracy=0.005)
        test_curve_fit(data[:,(0,2)], fitted_curve, max_mean_err = true_cpr * 0.01, inlier_range = true_cpr * 0.01, max_outliers = len(data[:,0]) * 0.02)

        # encoder.phase
        slope, offset, fitted_curve = fit_sawtooth(data[:,(0,3)], -pi, pi, sigma=5)
        test_assert_eq(slope / 7, 2*pi*abs(true_rps), accuracy=0.01)
        test_curve_fit(data[:,(0,3)], fitted_curve, max_mean_err = true_cpr * 0.01, inlier_range = true_cpr * 0.01, max_outliers = len(data[:,0]) * 0.02)

        # encoder.pos_estimate
        slope, offset, fitted_curve = fit_line(data[:,(0,4)])
        test_assert_eq(slope, true_cps, accuracy=0.005)
        test_curve_fit(data[:,(0,4)], fitted_curve, max_mean_err = true_cpr * 0.01, inlier_range = true_cpr * 0.01, max_outliers = len(data[:,0]) * 0.02)
        
        # encoder.pos_cpr
        slope, offset, fitted_curve = fit_sawtooth(data[:,(0,5)], true_cpr if reverse else 0, 0 if reverse else true_cpr)
        test_assert_eq(slope, true_cps, accuracy=0.005)
        test_curve_fit(data[:,(0,5)], fitted_curve, max_mean_err = true_cpr * 0.05, inlier_range = true_cpr * 0.05, max_outliers = len(data[:,0]) * 0.02)

        # encoder.vel_estimate
        slope, offset, fitted_curve = fit_line(data[:,(0,6)])
        test_assert_eq(slope, 0.0, range = true_cpr * abs(true_rps) * 0.005)
        test_assert_eq(offset, true_cpr * true_rps, accuracy = 0.005)
        test_curve_fit(data[:,(0,6)], fitted_curve, max_mean_err = true_cpr * 0.05, inlier_range = true_cpr * 0.05, max_outliers = len(data[:,0]) * 0.02)




class TestIncrementalEncoder(TestEncoderBase):

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            for encoder in odrive.encoders:
                # Find the Teensy that is connected to the encoder pins and the corresponding Teensy GPIOs

                gpio_conns = [
                    testrig.get_directly_connected_components(encoder.a),
                    testrig.get_directly_connected_components(encoder.b),
                ]

                valid_combinations = [
                    (combination[0].parent,) + tuple(combination)
                    for combination in itertools.product(*gpio_conns)
                    if ((len(set(c.parent for c in combination)) == 1) and isinstance(combination[0].parent, TeensyComponent))
                ]

                yield (encoder, valid_combinations)


    def run_test(self, enc: EncoderComponent, teensy: TeensyComponent, teensy_gpio_a: int, teensy_gpio_b: int, logger: Logger):
        true_cps = 8192*-0.5 # counts per second generated by the virtual encoder
        
        code = teensy_code_template.replace("{enc_a}", str(teensy_gpio_a.num)).replace("{enc_b}", str(teensy_gpio_b.num))
        teensy.compile_and_program(code)

        if enc.handle.config.mode != ENCODER_MODE_INCREMENTAL:
            enc.handle.config.mode = ENCODER_MODE_INCREMENTAL
            enc.parent.save_config_and_reboot()
        else:
            time.sleep(1.0) # wait for PLLs to stabilize

        encoder = enc.handle

        logger.debug("check if variables move at the correct velocity (8192 CPR)...")
        self.run_generic_encoder_test(enc.handle, 8192, true_cps / 8192)
        logger.debug("check if variables move at the correct velocity (65536 CPR)...")
        self.run_generic_encoder_test(enc.handle, 65536, true_cps / 65536)
        encoder.config.cpr = 8192



class TestSinCosEncoder(TestEncoderBase):
    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            gpio_conns = [
                testrig.get_directly_connected_components(odrive.gpio3),
                testrig.get_directly_connected_components(odrive.gpio4),
            ]

            valid_combinations = [
                (combination[0].parent,) + tuple(combination)
                for combination in itertools.product(*gpio_conns)
                if ((len(set(c.parent for c in combination)) == 1) and isinstance(combination[0].parent, TeensyComponent))
            ]

            yield (odrive.encoders[0], valid_combinations)


    def run_test(self, enc: EncoderComponent, teensy: TeensyComponent, teensy_gpio_sin: TeensyGpio, teensy_gpio_cos: TeensyGpio, logger: Logger):
        code = teensy_code_template2.replace("{enc_sin}", str(teensy_gpio_sin.num)).replace("{enc_cos}", str(teensy_gpio_cos.num))
        teensy.compile_and_program(code)

        if enc.handle.config.mode != ENCODER_MODE_SINCOS:
            enc.parent.unuse_gpios()
            enc.handle.config.mode = ENCODER_MODE_SINCOS
            enc.parent.save_config_and_reboot()
        else:
            time.sleep(1.0) # wait for PLLs to stabilize

        self.run_generic_encoder_test(enc.handle, 6283, 1.0)



if __name__ == '__main__':
    test_runner.run([
        TestIncrementalEncoder(),
        TestSinCosEncoder(),
    ])
