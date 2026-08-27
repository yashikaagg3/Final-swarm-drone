"""
Human-readable mission transcript.

Subscribes to /swarm/mission_state and every /drone_<i>/state topic and
logs a line each time something changes (initialization, region
assignment, state transition, mission completion) - a plain read-only
observer, it never publishes anything back into the swarm.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_drone.swarm_config import default_config_path, SwarmConfig


class TaskMonitorNode(Node):

    def __init__(self):
        super().__init__('task_monitor')
        self.declare_parameter('config_path', default_config_path())
        self.config = SwarmConfig.from_yaml_file(self.get_parameter('config_path').value)

        self._announced_init = False
        self._announced_region = set()
        self._last_drone_state = {}
        self._last_mission_state = None

        self.create_subscription(String, '/swarm/mission_state', self._on_mission_state, 10)
        for drone_id in range(self.config.num_drones):
            self.create_subscription(
                String, f'/drone_{drone_id}/state', self._make_state_callback(drone_id), 10)

    def _make_state_callback(self, drone_id):
        def _callback(msg):
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return

            if not self._announced_init:
                self._announced_init = True
                self.get_logger().info('Swarm initialized')
                self.get_logger().info(f'Number of drones: {self.config.num_drones}')
                self.get_logger().info(f'Leader: Drone {self.config.leader_id}')

            region_id = data.get('region_id')
            if region_id is not None and drone_id not in self._announced_region:
                self._announced_region.add(drone_id)
                self.get_logger().info(f'Drone {drone_id} -> Region {region_id}')

            state = data.get('state')
            if state and self._last_drone_state.get(drone_id) != state:
                self._last_drone_state[drone_id] = state
                self.get_logger().info(f'Drone {drone_id} -> {state}')
        return _callback

    def _on_mission_state(self, msg):
        if msg.data == self._last_mission_state:
            return
        self._last_mission_state = msg.data
        if msg.data == 'MISSION_COMPLETE':
            self.get_logger().info('SWARM MISSION COMPLETE')


def main(args=None):
    rclpy.init(args=args)
    node = TaskMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
