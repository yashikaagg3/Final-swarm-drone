from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'swarm_drone'


def data_files_for(directory):
    files = []
    for path in glob(os.path.join(directory, '**', '*'), recursive=True):
        if os.path.isfile(path):
            files.append(path)
    return [(os.path.join('share', package_name, directory), files)] if files else []


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *data_files_for('launch'),
        *data_files_for('config'),
        *data_files_for('urdf'),
        *data_files_for('worlds'),
        *data_files_for('rviz'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yashika Aggarwal',
    maintainer_email='yashikaagg3@gmail.com',
    description='ROS 2 swarm drone simulation: multi-drone autonomous area coverage in Gazebo',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'leader = swarm_drone.leader:main',
            'follower = swarm_drone.follower:main',
            'marker_manager = swarm_drone.marker_manager:main',
            'task_monitor = swarm_drone.task_monitor:main',
        ],
    },
)
