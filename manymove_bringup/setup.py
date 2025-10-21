"""Setup script for the manymove bringup package."""

import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'manymove_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/**/*.yaml', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Pastorio',
    maintainer_email='pastoriomarco@gmail.com',
    description='Bringup for manymove repo',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
