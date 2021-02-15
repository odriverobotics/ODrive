"""
This script is used to deploy the Fibre python library to PyPi
so that users can install them easily with
"pip install fibre"

To install the package and its dependencies locally, run:
    sudo pip install -r requirements.txt

To build and package the python tools into a tar archive:
    python setup.py sdist

Warning: Before you proceed, be aware that you can upload a
specific version only once ever. After that you need to increment
the hotfix number. Deleting the release manually on the PyPi
website does not help.

Use TestPyPi while developing.

To build, package and upload the python tools to TestPyPi, run:
    python setup.py sdist upload -r pypitest
To make a real release ensure you're at the release commit
and then run the above command without the "test" (so just "pypi").

To install a prerelease version from test index:
    sudo pip install --pre --index-url https://test.pypi.org/simple/ --no-cache-dir fibre


PyPi access requires that you have set up ~/.pypirc with your
PyPi credentials and that your account has the rights
to publish packages with the name fibre.
"""

# TODO: add additional y/n prompt to prevent from erroneous upload

from setuptools import setup
import os
import sys

# Change this if you already uploaded the current
# version but need to release a hotfix
hotfix = 0

#creating_package = "sdist" in sys.argv
#
## Load version from Git tag
#import odrive.version
#version = odrive.version.get_version_str(git_only=creating_package)
#
#if creating_package and (hotfix > 0 or not version[-1].isdigit()):
#  # Add this for hotfixes
#  version += "-" + str(hotfix)
#
#
## If we're currently creating the package we need to autogenerate
## a file that contains the version string
#if creating_package:
#  version_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'odrive', 'version.txt')
#  with open(version_file_path, mode='w') as version_file:
#    version_file.write(version)
#
## TODO: find a better place for this
#if not creating_package:
#  import platform
#  if platform.system() == 'Linux':
#    import odrive.utils
#    odrive.utils.setup_udev_rules(odrive.utils.Logger())

setup(
  name = 'fibre',
  packages = ['fibre'],
  #scripts = ['..fibre', 'odrivetool.bat', 'odrive_demo.py'],
  version = '0.0.1dev0',
  description = 'Abstraction layer for painlessly building object oriented distributed systems that just work',
  author = 'Samuel Sadok',
  author_email = 'samuel.sadok@bluewin.ch',
  license='MIT',
  url = 'https://github.com/samuelsadok/fibre',
  keywords = ['communication', 'transport-layer', 'rpc'],
  install_requires = [],
  #package_data={'': ['version.txt']},
  classifiers = [],
)

# TODO: include README

## clean up
#if creating_package:
#  os.remove(version_file_path)
