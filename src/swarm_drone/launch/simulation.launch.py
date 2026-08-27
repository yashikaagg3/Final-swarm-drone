"""
Starts Gazebo and spawns swarm.num_drones drones with unique namespaces.

Loads the mapping world and spawns drones (from config/swarm.yaml) into
namespaces /drone_<i>, each with its own robot_state_publisher and
ros_gz_bridge topics.

Drones would logically all start from the same takeoff point, but
spawning multiple physical bodies at exactly the same position causes
Gazebo collisions - each drone is spawned on a small ring around the
takeoff point, staggered in time, and converges to the shared point
once follower/leader logic commands takeoff (see follower.py).
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node
import xacro
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_drone')

    with open(os.path.join(pkg_share, 'config', 'swarm.yaml'), 'r') as f:
        cfg = yaml.safe_load(f)

    num_drones = int(cfg['swarm']['num_drones'])
    if num_drones < 1:
        raise RuntimeError(f'swarm.num_drones must be >= 1, got {num_drones}')

    takeoff = cfg['takeoff']
    base_x, base_y, base_z = float(takeoff['x']), float(takeoff['y']), float(takeoff['z'])
    ring_spacing = float(takeoff.get('spawn_ring_spacing', 0.5))
    use_sim_time = bool(cfg.get('simulation', {}).get('use_sim_time', True))
    world_name = str(cfg.get('simulation', {}).get('world_name', 'mapping_world'))

    world_path = os.path.join(pkg_share, 'worlds', 'mapping_world.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    actions = [
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
            output='screen',
        ),
    ]

    spawn_stagger_sec = 3.0

    for i in range(num_drones):
        name = f'drone_{i}'
        prefix = f'{name}_'

        # Spawn on a small ring around the logical takeoff point so bodies
        # don't overlap; drones converge to the real takeoff point on command.
        if num_drones == 1:
            sx, sy = base_x, base_y
        else:
            angle = 2.0 * math.pi * i / num_drones
            sx = base_x + ring_spacing * math.cos(angle)
            sy = base_y + ring_spacing * math.sin(angle)
        sz = base_z + 0.3

        robot_description = xacro.process_file(
            xacro_path, mappings={'prefix': prefix}).toxml()

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        )

        spawn = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', world_name,
                '-string', robot_description,
                '-name', name,
                '-x', str(sx), '-y', str(sy), '-z', str(sz),
            ],
            output='screen',
        )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=name,
            name='gz_bridge',
            arguments=[
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                f'/{prefix}camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                f'/{prefix}camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            ],
            remappings=[
                (f'/model/{name}/cmd_vel', 'cmd_vel'),
                (f'/model/{name}/odometry', 'odom'),
                (f'/{prefix}camera/image_raw', 'camera/image_raw'),
                (f'/{prefix}camera/camera_info', 'camera/camera_info'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )

        # OdometryPublisher reports each drone's pose directly in world
        # coordinates, so "map" and "<prefix>odom" coincide - publish that
        # as a static identity transform so RViz can resolve robot models.
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            namespace=name,
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', f'{prefix}odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        # Stagger spawns in time (on top of the ring offset) to give Gazebo's
        # physics a clean step between each new body being introduced.
        spawn_time = i * spawn_stagger_sec + 2.0
        actions.append(TimerAction(period=spawn_time, actions=[spawn]))
        actions.append(TimerAction(period=spawn_time, actions=[rsp, bridge, static_tf]))

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
