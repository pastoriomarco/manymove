"""Setup script for the manymove bringup package."""

import os
from collections import defaultdict
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'manymove_bringup'


def list_data_files(base_dir):
    """
    Collect data files preserving their relative directory structure.

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


def list_ur_variant_aliases(base_dir):
    """Provide duplicate installs of UR parameter sets at config/<ur_type> for compatibility."""
    variant_root = os.path.join(base_dir, 'ur')
    if not os.path.isdir(variant_root):
        return []

    aliases = []
    for variant in sorted(os.listdir(variant_root)):
        variant_dir = os.path.join(variant_root, variant)
        if not os.path.isdir(variant_dir):
            continue
        files = []
        for file_name in sorted(os.listdir(variant_dir)):
            full_path = os.path.join(variant_dir, file_name)
            if not os.path.isfile(full_path) or os.path.islink(full_path):
                continue
            files.append(full_path)
        if files:
            install_root = os.path.join('share', package_name, base_dir, variant)
            aliases.append((install_root, files))
    return aliases


config_data_files = list_data_files('config')
config_data_files.extend(list_ur_variant_aliases('config'))

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ]
    + config_data_files
    + list_data_files('urdf')
    + list_data_files('srdf'),
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
