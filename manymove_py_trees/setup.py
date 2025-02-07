from setuptools import find_packages, setup

package_name = 'manymove_py_trees'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tndlux',
    maintainer_email='pastoriomarco@gmail.com',
    description='PyTrees-based client for manymove_planner actions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_client = manymove_py_trees.bt_client:main',
            'bt_client_panda = manymove_py_trees.bt_client_panda:main',
        ],
    },
)
