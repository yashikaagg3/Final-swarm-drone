"""
Leader-side task-allocation and handshake bookkeeping over ROS 2 topics.

Owns the /swarm/region_assignment and /swarm/mission_state publishers and
the subscriptions to every drone's /drone_<i>/state topic. LeaderNode
(leader.py) uses this for all swarm-wide communication instead of talking
to topics directly, keeping the pub/sub wiring in one place.
"""

import json
import time

from std_msgs.msg import String


class TaskManager:

    def __init__(self, node, config):
        self._node = node
        self._config = config
        self.assignments = {}       # drone_id -> Region
        self._latest_status = {}    # drone_id -> parsed status dict
        self._last_seen = {}        # drone_id -> time.monotonic() of last status

        self._assignment_pub = node.create_publisher(String, '/swarm/region_assignment', 10)
        self._mission_state_pub = node.create_publisher(String, '/swarm/mission_state', 10)

        for drone_id in range(config.num_drones):
            node.create_subscription(
                String, f'/drone_{drone_id}/state', self._make_status_callback(drone_id), 10)

    def _make_status_callback(self, drone_id):
        def _callback(msg):
            try:
                self._latest_status[drone_id] = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                return
            self._last_seen[drone_id] = time.monotonic()
        return _callback

    def known_drone_ids(self):
        return list(self._latest_status.keys())

    def status_of(self, drone_id):
        return self._latest_status.get(drone_id)

    def state_of(self, drone_id):
        status = self._latest_status.get(drone_id)
        return status['state'] if status else None

    def seconds_since_seen(self, drone_id):
        last = self._last_seen.get(drone_id)
        return None if last is None else time.monotonic() - last

    def set_assignments(self, regions):
        self.assignments = {region.drone_id: region for region in regions}

    def publish_assignments(self, flight_altitude):
        msg = String()
        msg.data = json.dumps({
            'flight_altitude': flight_altitude,
            'assignments': [
                {
                    'drone_id': drone_id,
                    'min_x': region.min_x, 'max_x': region.max_x,
                    'min_y': region.min_y, 'max_y': region.max_y,
                }
                for drone_id, region in self.assignments.items()
            ],
        })
        self._assignment_pub.publish(msg)

    def publish_mission_state(self, state_name):
        msg = String()
        msg.data = state_name
        self._mission_state_pub.publish(msg)
