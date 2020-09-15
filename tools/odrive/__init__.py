
import os
import sys

# We want to use the fibre package that is included with the odrive package
# in order to avoid any version mismatch issues,
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "pyfibre"))

# Syntactic sugar to make usage more intuitive.
import fibre
find_any = fibre.find_any
find_all = fibre.find_all

# Standard convention is to add a __version__ attribute to the package
from .version import get_version_str
__version__ = get_version_str()
del get_version_str
