import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'white_slot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='papa',
    maintainer_email='phakaewpra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = white_slot.drive_node:main',
            'joystick_node = white_slot.joystick_node:main',
            'odom_node = white_slot.odom_node:main',
            'no_imu_odom_node = white_slot.no_imu_odom_node:main',
            'fuse_odom_node = white_slot.fuse_odom:main'


        ],
    },
)
