#!/bin/python3

import sys
import os

try:
    exec(open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'fibre', 'tools', 'interface_generator.py')).read())
except ImportError as ex:
    print(str(ex), file=sys.stderr)
    print("Note that there are new compile-time dependencies since around v0.5.1.", file=sys.stderr)
    print("Check out https://github.com/madcowswe/ODrive/blob/devel/docs/developer-guide.md#prerequisites for details.", file=sys.stderr)
    exit(1)
