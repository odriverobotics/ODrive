
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

            yield AnyTestCase(*[(odrive.axes[0], test_fixture) for encoder, test_fixture in encoders])

    def run_test(self, axis_ctx: ODriveAxisComponent, logger: Logger):
        logger.debug(f'Encoder {axis_ctx.num} was passed through')

tests = [EncoderPassthrough()]

if __name__ == '__main__':
    test_runner.run(tests)
