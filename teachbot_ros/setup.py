from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'teachbot_ros'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['teachbot_ros', 'launch/', 'rviz/', 'config/']),
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
            'teachbot_monitor_gui = teachbot_ros.utils.teachbot_monitor_gui:main',
            'teachbot_state_publisher_gui = teachbot_ros.utils.teachbot_state_publisher_gui:main',
            'publish_jointstates_from_sim = teachbot_ros.utils.publish_jointstates_from_sim:main',
        ],
    },
)
