"""
Launches the autonomous swarm mission logic (no Gazebo here).

Starts one leader node (swarm.leader_id) and one follower node per
other drone id (from config/swarm.yaml), plus marker_manager and
task_monitor. Run simulation.launch.py first so the drones exist to
control.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_drone')
    config_path = os.path.join(pkg_share, 'config', 'swarm.yaml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    num_drones = int(cfg['swarm']['num_drones'])
    leader_id = int(cfg['swarm']['leader_id'])
    use_sim_time = bool(cfg.get('simulation', {}).get('use_sim_time', True))

    actions = [
        Node(
            package='swarm_drone', executable='leader', name='leader',
            namespace=f'drone_{leader_id}',
            parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='swarm_drone', executable='marker_manager', name='marker_manager',
            parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='swarm_drone', executable='task_monitor', name='task_monitor',
            parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ]

    for drone_id in range(num_drones):
        if drone_id == leader_id:
            continue
        actions.append(Node(
            package='swarm_drone', executable='follower', name=f'follower_{drone_id}',
            namespace=f'drone_{drone_id}',
            parameters=[{
                'drone_id': drone_id,
                'config_path': config_path,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
