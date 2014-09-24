## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
#    scripts=['ros/src/cob_object_detection_fake'],
    packages=['cob_object_detection_fake'],
    package_dir={'': 'ros/src'}
)

setup(**setup_args)
