#!/usr/bin/env python
""" ROS Trimble BD960 Python Setup"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_trimble_bd960'],
    package_dir={'': 'src'}
)

setup(**d)
