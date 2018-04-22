
import re
import subprocess
import os
import sys

def get_version(git_only=False):
    """
    Returns the versions of the tools
    If git_only is true, the version.txt file is ignored even
    if it is present.
    """
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # Try to read the version.txt file that is generated during
    # the packaging step
    version_file_path = os.path.join(script_dir, 'version.txt')
    if os.path.exists(version_file_path) and git_only == False:
        with open(version_file_path) as version_file:
            return version_file.readline().rstrip('\n')

    try:
        # Determine the current git commit version
        git_result = subprocess.run(["git", "describe", "--always", "--tags", "--dirty=*"],
            cwd=script_dir,
            stdout=subprocess.PIPE, timeout=10)
        git_tag = git_result.stdout.decode(sys.stdout.encoding)

        regex=r'.*v([0-9a-zA-Z]).([0-9a-zA-Z]).([0-9a-zA-Z])(.*)'
        package_version_major = int(re.sub(regex, r"\1", git_tag))
        package_version_minor = int(re.sub(regex, r"\2", git_tag))
        package_version_revision = int(re.sub(regex, r"\3", git_tag))
        package_version_unreleased = (re.sub(regex, r"\4", git_tag) != "")

        if package_version_unreleased:
            package_version_revision += 1

        # TODO: fetch from Git describe
        version = '{}.{}.{}'.format(package_version_major, package_version_minor, package_version_revision)

        if package_version_unreleased:
            version += ".dev"
    except Exception as ex:
        print(ex)
        version = "whatever version in " + script_dir
    return version
