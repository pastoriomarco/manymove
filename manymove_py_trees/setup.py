"""Setup script for the manymove py trees package."""

from setuptools import find_packages
from setuptools import setup

package_name = 'manymove_py_trees'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Pastorio',
    maintainer_email='pastoriomarco@gmail.com',
    description='PyTrees-based client for manymove_planner actions',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_client_fake = manymove_py_trees.bt_client_fake:main',
            'bt_client_fake_panda = manymove_py_trees.bt_client_panda:main',
        ],
    },
)
