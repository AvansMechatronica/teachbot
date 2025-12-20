from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'teachbot_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include RViz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tos',
    maintainer_email='l.c.w.a.verstraete@student.tue.nl',
    description='ROS2 node for TOS Teachbot joint state publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teachbot_publisher = teachbot_ros.teachbot_publisher:main',
            'teachbot_monitor = teachbot_ros.utils.teachbot_monitor:main',
            'teachbot_monitor_gui = teachbot_ros.utils.teachbot_monitor_gui:main',
        ],
    },
)
