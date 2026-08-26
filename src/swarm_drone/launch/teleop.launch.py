"""
Prints the command to manually fly ONE selected drone with the keyboard.

`ros2 launch` does not give the processes it manages a real terminal on
stdin, so an interactive keyboard tool like teleop_twist_keyboard can't
actually run through it (it fails with `termios.error: Inappropriate
ioctl for device` - a known ROS 2 launch limitation, not specific to
this package). Instead of failing at runtime, this just prints the
`ros2 run` command that does work, so `drone_id:=<n>` stays the single
place you pick which drone to fly.

Defaults to config.testing.test_drone_id (override with
`drone_id:=<n>` on the command line). Run the printed command from an
interactive terminal while simulation.launch.py is running. A manual
Twist there simply becomes the drone's latest cmd_vel, overriding
whatever auto.launch.py's leader/follower node was last commanding for
that drone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_drone')
    with open(os.path.join(pkg_share, 'config', 'swarm.yaml'), 'r') as f:
        cfg = yaml.safe_load(f)
    default_id = cfg.get('testing', {}).get('test_drone_id', 0)

    requested = LaunchConfiguration('drone_id').perform(context)
    drone_id = int(requested) if requested != '' else int(default_id)
    topic = f'/drone_{drone_id}/cmd_vel'
    run_cmd = (
        'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
        f'--ros-args -r cmd_vel:={topic}'
    )

    return [LogInfo(msg=(
        '\n'
        f'  ros2 launch cannot host an interactive keyboard tool - run this instead:\n\n'
        f'  {run_cmd}\n\n'
        f'  (i/j/k/l/,/u/o to move, q/z to change speed, k to stop, drone_{drone_id})\n'
    ))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_id', default_value='',
            description='Drone id to control (default: testing.test_drone_id from swarm.yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
