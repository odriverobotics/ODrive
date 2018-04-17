"""
This script is used to deploy the ODrive python tools to PyPi
so that users can install them easily with
"pip install odrive"

To install the package and its dependencies locally, run:
    sudo pip install -r requirements.txt

To build and package the python tools into a tar archive:
    python setup.py sdist

Warning: Before you proceed, be aware that you can upload a
specific version only ever once. After that you need to increment
the hotfix number. Deleting the release manually on the PyPi
website does not help.

Use TestPyPi while developing.

To build, package and upload the python tools to TestPyPi, run:
    python setup.py sdist upload -r pypitest
To make a real release ensure you're at the release commit
and then run the above command without the "test".

To install a prerelease version from test index:
    sudo pip install --index-url https://test.pypi.org/simple/ --no-cache-dir odrive


PyPi access requires that you have set up ~/.pypirc with your
PyPi credentials and that your account has the rights
to publish packages with the name odrive.
"""

# TODO: add additional y/n prompt to prevent from erroneous upload

from distutils.core import setup
import os
import sys

creating_package = "sdist" in sys.argv

# Load version from Git tag
import odrive.version
version = odrive.version.get_version(git_only=creating_package)

# Change this if you already uploaded the current
# version but need to release a hotfix
hotfix = 0

if creating_package and (hotfix > 0 or not version[-1].isdigit()):
  # Add this for hotfixes
  version += "-" + str(hotfix)


# If we're currently creating the package we need to autogenerate
# a file that contains the version string
if creating_package:
  version_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'odrive', 'version.txt')
  with open(version_file_path, mode='w') as version_file:
    version_file.write(version)


setup(
  name = 'odrive',
  packages = ['odrive', 'odrive.dfuse'], # this must be the same as the name above
  scripts = ['odrivetool', 'odrive_demo.py'],
  version = version,
  description = 'Control utilities for the ODrive high performance motor controller',
  author = 'Oskar Weigl',
  author_email = 'oskar.weigl@odriverobotics.com',
  license='MIT',
  url = 'https://github.com/madcowswe/ODrive',
  keywords = ['odrive', 'motor', 'motor control'],
  install_requires = [
    'PyUSB',    # Required to access USB devices from Python through libusb
    'PySerial', # Required to access serial devices from Python
    'IntelHex', # Used to by DFU to load firmware files
    'matplotlib', # Required to run the liveplotter
    'pywin32 >= 1.0;platform_system=="Windows"' # Required for fancy terminal features on Windows
  ],
  package_data={'': ['version.txt']},
  include_package_data=True,
  classifiers = [],
)

# TODO: include README

# clean up
if creating_package:
  os.remove(version_file_path)
