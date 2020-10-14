
import test_runner

from fibre.utils import Logger
from test_runner import *

class EncoderPassthrough():
    """
    Does nothing except passing encoder0 through.
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            encoders = testrig.get_connected_components({
                'a': (odrive.encoders[0].a, False),
                'b': (odrive.encoders[0].b, False),
                'z': (odrive.encoders[0].z, False)
            }, EncoderComponent)
            motors = testrig.get_connected_components(odrive.axes[0], MotorComponent)

            for motor, encoder in itertools.product(motors, encoders):
                if encoder.impl in testrig.get_connected_components(motor):
                    yield (odrive.axes[0], motor, encoder)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        logger.debug(f'Encoder {axis_ctx.num} was passed through')

if __name__ == '__main__':
    test_runner.run(EncoderPassthrough())
