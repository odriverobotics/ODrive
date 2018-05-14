
import fibre
find_any = fibre.find_any
find_all = fibre.find_all

# Standard convention is to add a __version__ attribute to the package
from .version import get_version_str
__version__ = get_version_str()
del get_version_str
