"""Setup script for the manymove bringup package."""

import os
from collections import defaultdict
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'manymove_bringup'


def list_data_files(base_dir):
    """Collect data files preserving their relative directory structure.

    Symlinks are skipped to avoid packaging broken links across distros.
    """
    data_files = defaultdict(list)
    for root, _, files in os.walk(base_dir):
        if not files:
            continue
        install_root = os.path.join('share', package_name, root)
        for file_name in files:
            full_path = os.path.join(root, file_name)
            # Skip symlinks (e.g., distro-specific absolute links)
            if os.path.islink(full_path):
                continue
            data_files[install_root].append(full_path)
    return list(data_files.items())

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ]
    + list_data_files('config'),
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
