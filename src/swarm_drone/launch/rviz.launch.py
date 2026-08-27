"""
Launch file for RViz visualization with swarm_drone configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_drone')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'swarm.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            # TF and odometry are stamped with sim time; without this RViz reads
            # the wall clock and discards every transform as too old.
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    ])
