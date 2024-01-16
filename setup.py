#!/usr/bin/env python

"""The setup script."""

import os
import sys
from setuptools import setup, find_packages

this_dir = os.path.abspath(os.path.dirname(__file__))

VERSIONFILE = os.path.join(this_dir, "rosbridgeml", "__init__.py")
VERSION = None
for line in open(VERSIONFILE, "r").readlines():
    if line.startswith('__version__'):
        VERSION = line.split('\"')[1]

if not VERSION:
    raise RuntimeError('No version defined in rosbridgeml.__init__.py')

if sys.argv[-1].startswith('publish'):
    if os.system("pip list | grep wheel"):
        print("wheel not installed.\nUse `pip install wheel`.\nExiting.")
        sys.exit()
    if os.system("pip list | grep twine"):
        print("twine not installed.\nUse `pip install twine`.\nExiting.")
        sys.exit()
    os.system("python setup.py sdist bdist_wheel")
    if sys.argv[-1] == 'publishtest':
        os.system("twine upload -r test dist/*")
    else:
        os.system("twine upload dist/*")
        print("You probably want to also tag the version now:")
        print("  git tag -a {0} -m 'version {0}'".format(VERSION))
        print("  git push --tags")
    sys.exit()

with open('README.md') as readme_file:
    readme = readme_file.read()

# with open('HISTORY.rst') as history_file:
#     history = history_file.read()
history = ''

requirements = []

with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    entry_points={
        'console_scripts': [
            'rosbridge=rosbridgeml.cli:main',
        ],
        'textx_generators': [
            'rosbridge_ros=rosbridgeml.generator:generator_ros',
            'rosbridge_ros2=rosbridgeml.generator:generator_ros2',
        ],
        'textx_languages': [
            'rosbridge = rosbridgeml:rosbridge_language',
        ]
    },
    install_requires=requirements,
    long_description=readme + '\n\n' + history,
    include_package_data=True,
    package_data={'': ['*.tx']},
    keywords='rosbridge',
    name='rosbridgeml',
    packages=find_packages(include=['rosbridgeml', 'rosbridgeml.*']),
    test_suite='tests',
    url='https://github.com/robotics-4-all/rosbridge-dsl',
    version=VERSION,
    zip_safe=False,
)

