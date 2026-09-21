"""
Unified launch file for the swarm_drone package.

Launches:
 1. Vehicle backend from simulation.backend in swarm.yaml:
      gazebo (default): Ignition Fortress + VelocityControl URDF drones.
      mavros: PX4 SITL (x500) + one MAVROS node per drone.
 2. Leader node (auto_start=false so it waits for mission_cli / /swarm/goal).
 3. Follower nodes for all non-leader drones.
 4. marker_manager (RViz visualization) and task_monitor (transcript logger).

The drones spawn and sit on the ground ready until mission_cli is run.
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node
import xacro
import yaml


def _spawn_xy(i, num_drones, base_x, base_y, ring_spacing):
    if num_drones == 1:
        return base_x, base_y
    angle = 2.0 * math.pi * i / num_drones
    return (
        base_x + ring_spacing * math.cos(angle),
        base_y + ring_spacing * math.sin(angle),
    )


def _materialize_px4_world(pkg_share, wind_speed, wind_dir_deg, gusts):
    template_path = os.path.join(pkg_share, 'worlds', 'mapping_world_px4.sdf')
    with open(template_path, 'r', encoding='utf-8') as handle:
        text = handle.read()
    rad = math.radians(wind_dir_deg)
    vx = wind_speed * math.cos(rad)
    vy = wind_speed * math.sin(rad)
    enable = 'true' if wind_speed > 0.0 else 'false'
    gust = '0.25' if gusts and wind_speed > 0.0 else '0.0'
    text = (
        text.replace('ENABLE_WIND', enable)
        .replace('WIND_VX', f'{vx:.3f}')
        .replace('WIND_VY', f'{vy:.3f}')
        .replace('GUST_AMPLITUDE', gust)
    )
    out_dir = os.path.join(tempfile.gettempdir(), 'swarm_drone_px4_worlds')
    worlds_sub = os.path.join(out_dir, 'worlds')
    os.makedirs(worlds_sub, exist_ok=True)
    out_path = os.path.join(worlds_sub, 'mapping_world_px4.sdf')
    with open(out_path, 'w', encoding='utf-8') as handle:
        handle.write(text)
    return out_dir


def _resolve_px4_dir(mavros_cfg):
    explicit = str(mavros_cfg.get('px4_dir', '') or '').strip()
    candidates = [
        explicit,
        os.environ.get('PX4_AUTOPILOT_DIR', ''),
        os.path.expanduser('~/PX4-Autopilot'),
        os.path.expanduser('~/src/PX4-Autopilot'),
    ]
    for candidate in candidates:
        if not candidate:
            continue
        for build_name in ('px4_sitl_default', 'px4_sitl'):
            binary = os.path.join(candidate, 'build', build_name, 'bin', 'px4')
            if os.path.isfile(binary):
                return candidate, binary
    raise RuntimeError(
        'PX4 SITL binary not found. Build PX4 (`make px4_sitl`) and set '
        'simulation.mavros.px4_dir or PX4_AUTOPILOT_DIR. See docs/SIM_TO_HARDWARE.md.'
    )


def _gz_resource_path(px4_dir, world_dir):
    extras = [
        world_dir,
        os.path.join(px4_dir, 'Tools', 'simulation', 'gz'),
        os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds'),
        os.path.expanduser('~/.simulation-gazebo'),
        os.path.expanduser('~/PX4-gazebo-models'),
    ]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    parts = [existing] if existing else []
    for path in extras:
        if path and os.path.isdir(path) and path not in parts:
            parts.append(path)
    return os.pathsep.join(parts)


def _mavros_param_files():
    try:
        share = get_package_share_directory('mavros')
    except Exception as exc:
        raise RuntimeError(
            'The mavros package is not installed. On Humble: '
            '`sudo apt install ros-humble-mavros ros-humble-mavros-extras` '
            'and run install_geographiclib_datasets.sh. See docs/SIM_TO_HARDWARE.md.'
        ) from exc
    files = []
    for rel in (
            os.path.join('launch', 'px4_pluginlists.yaml'),
            os.path.join('launch', 'px4_config.yaml'),
            os.path.join('config', 'px4_pluginlists.yaml'),
            os.path.join('config', 'px4_config.yaml'),
    ):
        path = os.path.join(share, rel)
        if os.path.isfile(path):
            files.append(path)
    return files


def _mavros_node(drone_id, fcu_url, sysid, use_sim_time, extra_params):
    parameters = list(extra_params)
    parameters.append({
        'use_sim_time': use_sim_time,
        'fcu_url': fcu_url,
        'gcs_url': '',
        'tgt_system': sysid,
        'tgt_component': 1,
        'fcu_protocol': 'v2.0',
    })
    return Node(
        package='mavros',
        executable='mavros_node',
        namespace=f'drone_{drone_id}',
        name='mavros',
        parameters=parameters,
        output='screen',
        emulate_tty=True,
    )


def _gazebo_backend_actions(pkg_share, cfg, config_path, num_drones, use_sim_time):
    world_name = str(cfg.get('simulation', {}).get('world_name', 'mapping_world'))
    world_path = os.path.join(pkg_share, 'worlds', 'mapping_world.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    actions = [
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
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
            arguments=[
                f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
            output='screen',
        ),
        Node(
            package='swarm_drone', executable='spawner', name='spawner',
            parameters=[{'config_path': config_path, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ]

    for i in range(num_drones):
        name = f'drone_{i}'
        prefix = f'{name}_'
        robot_description = xacro.process_file(
            xacro_path, mappings={'prefix': prefix}).toxml()
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ))
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=name,
            name='gz_bridge',
            arguments=[
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
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
        ))
        actions.append(Node(
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
        ))
    return actions


def _mavros_backend_actions(pkg_share, cfg, num_drones, use_sim_time):
    sim = cfg.get('simulation', {})
    mavros_cfg = sim.get('mavros', {}) or {}
    wind = sim.get('wind', {}) or {}
    takeoff = cfg.get('takeoff', {})
    start_sitl = bool(mavros_cfg.get('start_sitl', True))
    fcu_pattern = str(
        mavros_cfg.get('fcu_url_pattern', 'udp://:{local}@127.0.0.1:{remote}'))
    model = str(mavros_cfg.get('model', 'gz_x500'))
    extra_params = _mavros_param_files()

    actions = []

    if start_sitl:
        px4_dir, px4_bin = _resolve_px4_dir(mavros_cfg)
        wind_speed = float(wind.get('speed_m_s', 0.0))
        wind_dir = float(wind.get('direction_deg', 0.0))
        gusts = bool(wind.get('gusts', False))
        world_dir = _materialize_px4_world(pkg_share, wind_speed, wind_dir, gusts)
        gz_path = _gz_resource_path(px4_dir, world_dir)
        base_x = float(takeoff.get('x', 0.0))
        base_y = float(takeoff.get('y', 0.0))
        base_z = float(takeoff.get('z', 0.0))
        ring = float(takeoff.get('spawn_ring_spacing', 2.0))

        for i in range(num_drones):
            sx, sy = _spawn_xy(i, num_drones, base_x, base_y, ring)
            sz = base_z + 0.3
            env = os.environ.copy()
            env['GZ_SIM_RESOURCE_PATH'] = gz_path
            env['PX4_SYS_AUTOSTART'] = '4001'
            env['PX4_SIM_MODEL'] = model
            env['PX4_GZ_MODEL'] = 'x500'
            env['PX4_GZ_MODEL_POSE'] = f'{sx:.2f},{sy:.2f},{sz:.2f},0,0,0'
            if i == 0:
                env['PX4_GZ_WORLD'] = 'mapping_world_px4'
            instance_dir = os.path.join(
                tempfile.gettempdir(), f'px4_sitl_drone_{i}')
            os.makedirs(instance_dir, exist_ok=True)
            px4_proc = ExecuteProcess(
                cmd=[px4_bin, '-i', str(i), '-w', instance_dir],
                cwd=px4_dir,
                additional_env=env,
                output='screen',
                emulate_tty=True,
            )
            actions.append(TimerAction(period=float(i) * 4.0, actions=[px4_proc]))

    for i in range(num_drones):
        if '{' in fcu_pattern:
            fcu_url = fcu_pattern.format(
                local=14540 + i, remote=14580 + i, drone_id=i)
        else:
            fcu_url = fcu_pattern
        sysid = i + 1
        mavros = _mavros_node(i, fcu_url, sysid, use_sim_time, extra_params)
        delay = 8.0 + float(i) * 4.0 if start_sitl else 0.0
        actions.append(TimerAction(period=delay, actions=[mavros]))
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            namespace=f'drone_{i}',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    return actions


def _swarm_logic_actions(config_path, num_drones, leader_id, use_sim_time):
    actions = [
        Node(
            package='swarm_drone', executable='leader', name='leader',
            namespace=f'drone_{leader_id}',
            parameters=[{
                'config_path': config_path,
                'use_sim_time': use_sim_time,
                'auto_start': False,
            }],
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


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_drone')
    config_path = os.path.join(pkg_share, 'config', 'swarm.yaml')

    with open(config_path, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)

    num_drones = int(cfg['swarm']['num_drones'])
    leader_id = int(cfg['swarm']['leader_id'])
    if num_drones < 1:
        raise RuntimeError(f'swarm.num_drones must be >= 1, got {num_drones}')

    sim = cfg.get('simulation', {})
    use_sim_time = bool(sim.get('use_sim_time', True))
    backend = str(sim.get('backend', 'mavros')).lower()

    if backend == 'mavros':
        actions = _mavros_backend_actions(
            pkg_share, cfg, num_drones, use_sim_time)
    elif backend == 'gazebo':
        actions = _gazebo_backend_actions(
            pkg_share, cfg, config_path, num_drones, use_sim_time)
    else:
        raise RuntimeError(
            f'simulation.backend must be gazebo or mavros, got {backend!r}')

    actions.extend(
        _swarm_logic_actions(config_path, num_drones, leader_id, use_sim_time))
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
