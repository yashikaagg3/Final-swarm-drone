"""
Manual keyboard teleop for ONE selected drone, for debugging.

Defaults to config.testing.test_drone_id (override with
`drone_id:=<n>` on the command line). Run this from an interactive
terminal (it reads raw keyboard input) while simulation.launch.py is
running. A manual Twist here simply becomes the drone's latest
cmd_vel, overriding whatever auto.launch.py's leader/follower node was
last commanding for that drone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_drone')
    with open(os.path.join(pkg_share, 'config', 'swarm.yaml'), 'r') as f:
        cfg = yaml.safe_load(f)
    default_id = cfg.get('testing', {}).get('test_drone_id', 0)

    requested = LaunchConfiguration('drone_id').perform(context)
    drone_id = int(requested) if requested != '' else int(default_id)
    topic = f'/drone_{drone_id}/cmd_vel'

    print(f'[teleop.launch] Controlling drone_{drone_id} via {topic} - '
          f'i/j/k/l/,/u/o to move, q/z to change speed, k to stop.')

    return [
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            emulate_tty=True,
            remappings=[('cmd_vel', topic)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_id', default_value='',
            description='Drone id to control (default: testing.test_drone_id from swarm.yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
