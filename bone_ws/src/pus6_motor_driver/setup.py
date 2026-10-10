from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pus6_motor_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bone_drill',
    maintainer_email='dev@example.com',
    description='ROS 2 driver package for 6PUS Inspire electric cylinders.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = pus6_motor_driver.motor_driver_node:main',
            'motor_driver_legacy_debug = pus6_motor_driver.motor_driver_node:legacy_debug_main',
        ],
    },
)
