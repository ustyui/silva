#!/usr/bin/env python

# DO NOT RUN setup.py, use catkin_make instead
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['modules']
d['package_dir'] = {'': 'src'}

setup(**d)
