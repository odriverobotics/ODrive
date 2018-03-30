
# Standard convention is to add a __version__ attribute to the package
from .version import get_version
__version__ = get_version()
del get_version
