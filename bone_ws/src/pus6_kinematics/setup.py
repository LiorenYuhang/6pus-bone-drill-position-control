from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pus6_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML>=5.3'],
    zip_safe=True,
    maintainer='bone_drill',
    maintainer_email='dev@example.com',
    description='Kinematics calculation package for 6PUS parallel manipulator.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = pus6_kinematics.kinematics_node:main',
            'fk_node = pus6_kinematics.fk_node:main',
        ],
    },
)
