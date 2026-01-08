from setuptools import setup
import os
from glob import glob

package_name = 'offboard_control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Offboard Control for PX4 using ROS2 and MAVROS with SLAM Toolbox',
    license='Apache License 2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'offboard_control = offboard_control_pkg.offboard_control:main',
            'slam_launch = offboard_control_pkg.slam_launch:main',
            'drone_node = offboard_control_pkg.drone_node:main',
            'multi_drone_sender = offboard_control_pkg.multi_drone_sender:main',
            'fast_scan_node = offboard_control_pkg.fast_scan_node:main',
        ],
    },
)
