"""
Per-drone mission state machine, and the standalone follower node.

DroneMission implements the state machine every drone (leader included)
runs for its own flight: INITIALIZING -> WAITING_FOR_TASK -> TASK_ASSIGNED
-> TAKING_OFF -> GOING_TO_REGION -> COVERING -> COMPLETED -> LANDING ->
IDLE. leader.py reuses this class for the leader's own coverage so that
logic is never duplicated between the two roles.

FollowerNode is the ROS 2 node run for each non-leader drone: it wires a
DroneMission to /swarm/region_assignment and to this drone's own
/drone_<id>/state topic (published via DroneMission itself).
"""

from enum import Enum
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_drone.coverage_planner import generate_coverage_path
from swarm_drone.drone_controller import DroneController
from swarm_drone.region_allocator import Region
from swarm_drone.swarm_config import default_config_path, SwarmConfig


class DroneState(str, Enum):
    INITIALIZING = 'INITIALIZING'
    WAITING_FOR_TASK = 'WAITING_FOR_TASK'
    TASK_ASSIGNED = 'TASK_ASSIGNED'
    TAKING_OFF = 'TAKING_OFF'
    GOING_TO_REGION = 'GOING_TO_REGION'
    COVERING = 'COVERING'
    COMPLETED = 'COMPLETED'
    LANDING = 'LANDING'
    IDLE = 'IDLE'


class DroneMission:
    """Owns one drone's DroneController and its INITIALIZING..IDLE state."""

    def __init__(self, node, drone_id, role, config, state_rate_hz=2.0, tick_rate_hz=5.0):
        self.drone_id = drone_id
        self.role = role
        self.state = DroneState.INITIALIZING
        self.region = None
        self.waypoints = []

        self._node = node
        self._config = config
        self._waypoint_index = 0
        self._assigned_at = None

        self.controller = DroneController(
            node, cruise_speed=config.cruise_speed, waypoint_tolerance=config.waypoint_tolerance)
        self._state_pub = node.create_publisher(String, 'state', 10)

        node.create_timer(1.0 / state_rate_hz, self._publish_state)
        node.create_timer(1.0 / tick_rate_hz, self._tick)

        # Nothing to actually initialize (no sensors to calibrate) - go
        # straight to waiting for the leader's task assignment.
        self.state = DroneState.WAITING_FOR_TASK

    def assign_region(self, region: Region):
        if self.state != DroneState.WAITING_FOR_TASK:
            return  # already assigned; ignore a duplicate/late assignment
        self.region = region
        self.waypoints = generate_coverage_path(
            region, self._config.waypoint_spacing, self._config.boundary_margin,
            altitude=self._config.flight_altitude)
        self._waypoint_index = 0
        self._assigned_at = time.monotonic()
        self.state = DroneState.TASK_ASSIGNED

    def _tick(self):
        if self.state == DroneState.TASK_ASSIGNED:
            # Stagger takeoff by drone_id so drones spawned close together
            # on the takeoff ring don't all lift off - and collide - at once.
            stagger = self.drone_id * self._config.stagger_delay_sec
            if time.monotonic() - self._assigned_at >= stagger:
                self.state = DroneState.TAKING_OFF
                self.controller.takeoff(
                    self._config.flight_altitude, on_complete=self._on_takeoff_done)
        elif self.state == DroneState.COMPLETED:
            self.state = DroneState.LANDING
            self.controller.land(on_complete=self._on_landed)

    def _on_takeoff_done(self):
        self.state = DroneState.GOING_TO_REGION
        self._go_to_next_waypoint()

    def _go_to_next_waypoint(self):
        if self._waypoint_index >= len(self.waypoints):
            self.state = DroneState.COMPLETED
            self.controller.stop()
            return
        if self._waypoint_index > 0:
            self.state = DroneState.COVERING
        x, y, z = self.waypoints[self._waypoint_index]
        self.controller.move_to(x, y, z, on_reached=self._on_waypoint_reached)

    def _on_waypoint_reached(self):
        self._waypoint_index += 1
        self._go_to_next_waypoint()

    def _on_landed(self):
        self.state = DroneState.IDLE

    def coverage_progress(self):
        if not self.waypoints:
            return 0.0
        return min(1.0, self._waypoint_index / len(self.waypoints))

    def _publish_state(self):
        position = self.controller.current_position()
        msg = String()
        msg.data = json.dumps({
            'drone_id': self.drone_id,
            'role': self.role,
            'state': self.state.value,
            'region_id': self.region.drone_id if self.region else None,
            'position': list(position) if position else None,
            'progress': self.coverage_progress(),
        })
        self._state_pub.publish(msg)


class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower')
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('config_path', default_config_path())

        drone_id = int(self.get_parameter('drone_id').value)
        config_path = self.get_parameter('config_path').value
        config = SwarmConfig.from_yaml_file(config_path)

        if config.is_leader(drone_id):
            self.get_logger().warn(
                f'drone_id {drone_id} is configured as swarm.leader_id - '
                f'run leader.py for this drone instead of follower.py')

        self.mission = DroneMission(self, drone_id, 'follower', config)
        self.create_subscription(String, '/swarm/region_assignment', self._on_assignment, 10)
        self.get_logger().info(f'Follower drone_{drone_id} initialized, waiting for task.')

    def _on_assignment(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Received malformed region_assignment message')
            return
        for entry in data.get('assignments', []):
            if entry['drone_id'] == self.mission.drone_id:
                region = Region(
                    drone_id=entry['drone_id'],
                    min_x=entry['min_x'], max_x=entry['max_x'],
                    min_y=entry['min_y'], max_y=entry['max_y'])
                self.mission.assign_region(region)
                break


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
