"""
Unified launch file for the swarm_drone package.

Launches:
 1. Gazebo Fortress simulation with mapping_world.sdf and clock bridge.
 2. Spawns all drones with URDF, state publishers, topic bridges, and static TFs.
 3. Leader node (configured with auto_start=false so it waits for mission goal CLI input).
 4. Follower nodes for all non-leader drones.
 5. marker_manager (RViz visualization) and task_monitor (transcript logger).

The drones spawn and sit on the ground ready until mission_cli is run!
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
import xacro
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_drone')
    config_path = os.path.join(pkg_share, 'config', 'swarm.yaml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    num_drones = int(cfg['swarm']['num_drones'])
    leader_id = int(cfg['swarm']['leader_id'])
    if num_drones < 1:
        raise RuntimeError(f'swarm.num_drones must be >= 1, got {num_drones}')

    use_sim_time = bool(cfg.get('simulation', {}).get('use_sim_time', True))
    world_name = str(cfg.get('simulation', {}).get('world_name', 'mapping_world'))

    world_path = os.path.join(pkg_share, 'worlds', 'mapping_world.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    actions = [
        # 1. Primary simulation engine & world clock
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
            # No IGN_IP: Ignition Transport discovers over UDP multicast, and the
            # loopback interface has no MULTICAST flag, so pinning it to 127.0.0.1
            # makes the create/state services intermittently unreachable.
            additional_env={
                'MESA_GL_VERSION_OVERRIDE': '3.3',
                'QT_X11_NO_MITSHM': '1',
            },
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
        # 2. Priority entity spawner (spawns drones into Gazebo immediately)
        Node(
            package='swarm_drone', executable='spawner', name='spawner',
            parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ]

    # 3. Setup robot state publishers, topic bridges, and TFs for each drone
    for i in range(num_drones):
        name = f'drone_{i}'
        prefix = f'{name}_'

        robot_description = xacro.process_file(
            xacro_path, mappings={'prefix': prefix}).toxml()

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=name,
            name='gz_bridge',
            arguments=[
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                # OdometryPublisher emits odom->base_link here; without it the TF
                # tree is split and RViz cannot place the robot models.
                f'/model/{name}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                f'/{prefix}camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                f'/{prefix}camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            ],
            remappings=[
                (f'/model/{name}/cmd_vel', 'cmd_vel'),
                (f'/model/{name}/odometry', 'odom'),
                (f'/model/{name}/pose', '/tf'),
                (f'/{prefix}camera/image_raw', 'camera/image_raw'),
                (f'/{prefix}camera/camera_info', 'camera/camera_info'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )

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

        actions.append(rsp)
        actions.append(bridge)
        actions.append(static_tf)

    # 4. Swarm orchestration and management nodes
    actions.append(Node(
        package='swarm_drone', executable='leader', name='leader',
        namespace=f'drone_{leader_id}',
        parameters=[{
            'config_path': config_path,
            'use_sim_time': use_sim_time,
            'auto_start': False,
        }],
        output='screen',
    ))

    actions.append(Node(
        package='swarm_drone', executable='marker_manager', name='marker_manager',
        parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
        output='screen',
    ))

    actions.append(Node(
        package='swarm_drone', executable='task_monitor', name='task_monitor',
        parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
        output='screen',
    ))

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
