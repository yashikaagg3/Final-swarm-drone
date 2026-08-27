"""
Robust drone spawner node.

Spawns drones sequentially, retrying each one until it actually shows up in
the running world.
"""

import math
import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from swarm_drone.swarm_config import default_config_path, SwarmConfig
import xacro


class SpawnerNode(Node):

    def __init__(self):
        super().__init__('spawner')
        self.get_logger().info('Spawner node started. Preparing to spawn swarm into Gazebo...')

    def _model_exists(self, world_name, name, env):
        # `create` exits 0 and logs to stderr even when its request times out,
        # so the world itself is the only trustworthy source of truth.
        try:
            proc = subprocess.run(
                ['ign', 'model', '--list', '-w', world_name],
                capture_output=True, text=True, env=env, timeout=10)
        except subprocess.TimeoutExpired:
            return False
        models = {line.strip().lstrip('-').strip() for line in proc.stdout.splitlines()}
        return name in models

    def spawn_swarm(self):
        pkg_share = get_package_share_directory('swarm_drone')
        config_path = default_config_path()
        config = SwarmConfig.from_yaml_file(config_path)

        world_name = config.world_name
        num_drones = config.num_drones
        base_x, base_y, base_z = config.takeoff_x, config.takeoff_y, config.takeoff_z
        ring_spacing = config.spawn_ring_spacing
        xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

        env = os.environ.copy()
        env['IGN_IP'] = '127.0.0.1'

        for i in range(num_drones):
            name = f'drone_{i}'
            prefix = f'{name}_'

            if num_drones == 1:
                sx, sy = base_x, base_y
            else:
                angle = 2.0 * math.pi * i / num_drones
                sx = base_x + ring_spacing * math.cos(angle)
                sy = base_y + ring_spacing * math.sin(angle)
            sz = base_z + 0.3

            robot_description = xacro.process_file(
                xacro_path, mappings={'prefix': prefix}).toxml()

            self.get_logger().info(
                f'Spawning {name} at position ({sx:.2f}, {sy:.2f}, {sz:.2f})...')

            spawned = False
            for retry in range(15):
                cmd = [
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', world_name,
                    '-string', robot_description,
                    '-name', name,
                    '-x', str(sx), '-y', str(sy), '-z', str(sz),
                ]
                proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
                if self._model_exists(world_name, name, env):
                    self.get_logger().info(f'[SUCCESS] {name} spawned cleanly!')
                    spawned = True
                    break
                self.get_logger().warn(
                    f'{name} missing from world after attempt {retry + 1}/15: '
                    f'{(proc.stderr or proc.stdout).strip()}')
                time.sleep(0.5)

            if not spawned:
                self.get_logger().error(
                    f'[FAILED] {name} never appeared in world "{world_name}". If every drone '
                    'fails here, ros_gz_sim is most likely built against a different Gazebo '
                    'than the one being launched.')

            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = SpawnerNode()
    try:
        node.spawn_swarm()
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
