"""
Swarm orchestration node.

Runs the leader state machine (INITIALIZING -> WAITING_FOR_SWARM ->
ALLOCATING_TASKS -> WAITING_FOR_ACK -> MISSION_RUNNING -> MONITORING ->
MISSION_COMPLETE) on top of TaskManager (communication/bookkeeping) and
region_allocator (equal-area division). The leader is also an active
drone, so it runs its own DroneMission (from follower.py) for its own
takeoff/coverage - that logic is never duplicated here.
"""

from enum import Enum
import time

import rclpy
from rclpy.node import Node

from swarm_drone.follower import DroneMission, DroneState
from swarm_drone.region_allocator import allocate_regions
from swarm_drone.swarm_config import default_config_path, SwarmConfig
from swarm_drone.task_manager import TaskManager

_COMPLETED_OR_LATER = {DroneState.COMPLETED.value, DroneState.LANDING.value, DroneState.IDLE.value}
_STILL_WAITING = {None, DroneState.INITIALIZING.value, DroneState.WAITING_FOR_TASK.value}


class LeaderState(str, Enum):
    INITIALIZING = 'INITIALIZING'
    WAITING_FOR_SWARM = 'WAITING_FOR_SWARM'
    WAITING_FOR_GOAL = 'WAITING_FOR_GOAL'
    ALLOCATING_TASKS = 'ALLOCATING_TASKS'
    WAITING_FOR_ACK = 'WAITING_FOR_ACK'
    MISSION_RUNNING = 'MISSION_RUNNING'
    MONITORING = 'MONITORING'
    MISSION_COMPLETE = 'MISSION_COMPLETE'


class LeaderNode(Node):

    def __init__(self):
        super().__init__('leader')
        self.declare_parameter('config_path', default_config_path())
        self.declare_parameter('auto_start', True)
        config = SwarmConfig.from_yaml_file(self.get_parameter('config_path').value)
        self.config = config

        self.mission = DroneMission(self, config.leader_id, 'leader', config)
        self.task_manager = TaskManager(self, config)

        self.state = LeaderState.INITIALIZING
        self._ack_wait_start = None
        self._last_assignment_publish = 0.0
        self._assignment_republish_period = 1.0
        self._ever_completed = set()
        self._warned_stale = set()
        self._overridden_drones = set()

        from std_msgs.msg import String
        self.create_subscription(String, '/swarm/goal', self._on_goal_received, 10)
        self.create_subscription(String, '/swarm/manual_override', self._on_manual_override, 10)

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Swarm initialized. Number of drones: {config.num_drones}. '
            f'Leader: Drone {config.leader_id}')
        self.state = LeaderState.WAITING_FOR_SWARM

    def _follower_ids(self):
        return [i for i in range(self.config.num_drones) if i != self.config.leader_id]

    def _tick(self):
        now = time.monotonic()
        self._check_heartbeats(now)

        if self.state == LeaderState.WAITING_FOR_SWARM:
            self._tick_waiting_for_swarm()
        elif self.state == LeaderState.ALLOCATING_TASKS:
            self._tick_allocating_tasks(now)
        elif self.state == LeaderState.WAITING_FOR_ACK:
            self._tick_waiting_for_ack(now)
        elif self.state == LeaderState.MISSION_RUNNING:
            self.get_logger().info('Mission running - all drones underway.')
            self.state = LeaderState.MONITORING
        elif self.state == LeaderState.MONITORING:
            self._tick_monitoring()

        self.task_manager.publish_mission_state(self.state.value)

    def _tick_waiting_for_swarm(self):
        if self.mission.state != DroneState.WAITING_FOR_TASK:
            return
        for follower_id in self._follower_ids():
            if self.task_manager.state_of(follower_id) is None:
                return  # haven't heard from this follower yet
        auto_start = self.get_parameter('auto_start').value
        if auto_start:
            self.get_logger().info('All drones ready. Starting task allocation (auto_start=True).')
            self.state = LeaderState.ALLOCATING_TASKS
        else:
            self.get_logger().info('All drones ready. Waiting for mission goal via mission_cli...')
            self.state = LeaderState.WAITING_FOR_GOAL

    def _on_goal_received(self, msg):
        import json
        if self.state in (LeaderState.WAITING_FOR_SWARM, LeaderState.WAITING_FOR_GOAL):
            try:
                data = json.loads(msg.data)
                self.config.min_x = float(data.get('min_x', self.config.min_x))
                self.config.max_x = float(data.get('max_x', self.config.max_x))
                self.config.min_y = float(data.get('min_y', self.config.min_y))
                self.config.max_y = float(data.get('max_y', self.config.max_y))
                self.config.flight_altitude = float(
                    data.get('flight_altitude', self.config.flight_altitude))
                self.config.waypoint_spacing = float(
                    data.get('waypoint_spacing', self.config.waypoint_spacing))
                if 'cruise_speed' in data:
                    self.config.cruise_speed = float(data['cruise_speed'])
                self.get_logger().info(
                    f'Received Goal: Area=[{self.config.min_x}, {self.config.max_x}] x '
                    f'[{self.config.min_y}, {self.config.max_y}].')
                self.state = LeaderState.ALLOCATING_TASKS
            except Exception as e:
                self.get_logger().error(f'Failed to parse mission goal JSON: {e}')

    def _on_manual_override(self, msg):
        import json
        try:
            data = json.loads(msg.data)
            drone_id = int(data['drone_id'])
            action = str(data['action'])
            if action == 'override':
                self._overridden_drones.add(drone_id)
                self.get_logger().warn(
                    f'Drone {drone_id} is manually overridden! Pausing progress tracking.')
            elif action == 'resume':
                self._overridden_drones.discard(drone_id)
                self.get_logger().info(
                    f'Drone {drone_id} manual control released. Resuming progress tracking.')
        except Exception as e:
            self.get_logger().error(f'Error processing manual override message: {e}')

    def _tick_allocating_tasks(self, now):
        regions = allocate_regions(
            self.config.num_drones, self.config.min_x, self.config.max_x,
            self.config.min_y, self.config.max_y)
        self.task_manager.set_assignments(regions)
        for region in sorted(regions, key=lambda r: r.drone_id):
            self.get_logger().info(f'Drone {region.drone_id} -> Region {region.drone_id}')
            if region.drone_id == self.config.leader_id:
                self.mission.assign_region(region)
        self._ack_wait_start = now
        self.state = LeaderState.WAITING_FOR_ACK

    def _tick_waiting_for_ack(self, now):
        if now - self._last_assignment_publish > self._assignment_republish_period:
            self.task_manager.publish_assignments(self.config.flight_altitude)
            self._last_assignment_publish = now

        if self._all_acknowledged():
            self.get_logger().info('All drones acknowledged their region assignment.')
            self.state = LeaderState.MISSION_RUNNING
        elif now - self._ack_wait_start > self.config.ack_timeout_sec:
            for follower_id in self._follower_ids():
                if self.task_manager.state_of(follower_id) in _STILL_WAITING:
                    self.get_logger().error(
                        f'Drone {follower_id} has not acknowledged its region '
                        f'assignment after {self.config.ack_timeout_sec}s')

    def _all_acknowledged(self):
        if self.mission.state in (DroneState.INITIALIZING, DroneState.WAITING_FOR_TASK):
            return False
        for follower_id in self._follower_ids():
            if self.task_manager.state_of(follower_id) in _STILL_WAITING:
                return False
        return True

    def _tick_monitoring(self):
        leader_done = (
            self.mission.state.value in _COMPLETED_OR_LATER
            and self.config.leader_id not in self._overridden_drones)
        if leader_done:
            self._ever_completed.add(self.config.leader_id)
        for follower_id in self._follower_ids():
            follower_done = (
                follower_id not in self._overridden_drones
                and self.task_manager.state_of(follower_id) in _COMPLETED_OR_LATER)
            if follower_done:
                self._ever_completed.add(follower_id)

        if len(self._ever_completed) == self.config.num_drones and not self._overridden_drones:
            self.get_logger().info('SWARM MISSION COMPLETE')
            self.state = LeaderState.MISSION_COMPLETE

    def _check_heartbeats(self, now):
        for follower_id in self._follower_ids():
            age = self.task_manager.seconds_since_seen(follower_id)
            stale = age is not None and age > self.config.heartbeat_timeout_sec
            if stale and follower_id not in self._warned_stale:
                self.get_logger().error(
                    f'Drone {follower_id} unresponsive: no state update for '
                    f'{age:.1f}s (timeout {self.config.heartbeat_timeout_sec}s)')
                self._warned_stale.add(follower_id)
            elif not stale and follower_id in self._warned_stale:
                self._warned_stale.discard(follower_id)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
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
