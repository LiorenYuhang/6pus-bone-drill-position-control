from setuptools import find_packages, setup

package_name = 'pus6_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bone_drill',
    maintainer_email='dev@example.com',
    description='Control system package for 6PUS parallel manipulator.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = pus6_control.control_node:main',
        ],
    },
)
