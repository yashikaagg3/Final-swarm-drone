"""
Publishes a visualization_msgs/MarkerArray summarizing the whole swarm.

Subscribes to /swarm/region_assignment and every /drone_<i>/state topic
(the same topics leader/follower already publish - no new data channel
is introduced) and republishes drone positions, roles, region
boundaries/ids and coverage waypoints as one MarkerArray on
/swarm/markers. Waypoints are recomputed locally with the same
deterministic coverage_planner used by DroneMission, so they never need
their own topic.
"""

import json

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarm_drone.coverage_planner import generate_coverage_path
from swarm_drone.region_allocator import Region
from swarm_drone.swarm_config import default_config_path, SwarmConfig
from visualization_msgs.msg import Marker, MarkerArray

_PALETTE = [
    (0.90, 0.20, 0.20), (0.20, 0.70, 0.30), (0.20, 0.40, 0.90),
    (0.90, 0.60, 0.10), (0.60, 0.20, 0.80), (0.10, 0.80, 0.80),
]


class MarkerManagerNode(Node):

    def __init__(self):
        super().__init__('marker_manager')
        self.declare_parameter('config_path', default_config_path())
        self.config = SwarmConfig.from_yaml_file(self.get_parameter('config_path').value)

        self._regions = {}     # drone_id -> Region
        self._waypoints = {}   # drone_id -> list of (x, y, z)
        self._status = {}      # drone_id -> parsed state dict

        self._pub = self.create_publisher(MarkerArray, '/swarm/markers', 10)
        self.create_subscription(String, '/swarm/region_assignment', self._on_assignment, 10)
        for drone_id in range(self.config.num_drones):
            self.create_subscription(
                String, f'/drone_{drone_id}/state', self._make_status_callback(drone_id), 10)
        self.create_timer(0.5, self._publish_markers)

    def _make_status_callback(self, drone_id):
        def _callback(msg):
            try:
                self._status[drone_id] = json.loads(msg.data)
            except json.JSONDecodeError:
                pass
        return _callback

    def _on_assignment(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        altitude = data.get('flight_altitude', self.config.flight_altitude)
        for entry in data.get('assignments', []):
            region = Region(
                drone_id=entry['drone_id'], min_x=entry['min_x'], max_x=entry['max_x'],
                min_y=entry['min_y'], max_y=entry['max_y'])
            self._regions[region.drone_id] = region
            self._waypoints[region.drone_id] = generate_coverage_path(
                region, self.config.waypoint_spacing, self.config.boundary_margin,
                altitude=altitude)

    @staticmethod
    def _color_for(drone_id):
        return _PALETTE[drone_id % len(_PALETTE)]

    @staticmethod
    def _marker_base(ns, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _region_boundary_marker(self, drone_id, region, color, stamp):
        marker = self._marker_base('region_boundary', drone_id, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.1
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 0.9)
        corners = [
            (region.min_x, region.min_y), (region.max_x, region.min_y),
            (region.max_x, region.max_y), (region.min_x, region.max_y),
            (region.min_x, region.min_y),
        ]
        marker.points = [Point(x=x, y=y, z=0.05) for x, y in corners]
        return marker

    def _region_label_marker(self, drone_id, region, stamp):
        marker = self._marker_base('region_label', drone_id, Marker.TEXT_VIEW_FACING, stamp)
        cx, cy = region.center()
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = cx, cy, 0.3
        marker.scale.z = 0.6
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'Region {drone_id}'
        return marker

    def _waypoints_marker(self, drone_id, waypoints, color, stamp):
        marker = self._marker_base('waypoints', drone_id, Marker.POINTS, stamp)
        marker.scale.x = marker.scale.y = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 0.6)
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in waypoints]
        return marker

    def _position_marker(self, drone_id, position, role, color, stamp):
        marker = self._marker_base('drone_position', drone_id, Marker.SPHERE, stamp)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        size = 0.5 if role == 'leader' else 0.35
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
        return marker

    def _label_marker(self, drone_id, status, role, stamp):
        marker = self._marker_base('drone_label', drone_id, Marker.TEXT_VIEW_FACING, stamp)
        x, y, z = status['position']
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z + 0.6
        marker.scale.z = 0.4
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        progress_pct = int(round(status.get('progress', 0.0) * 100))
        marker.text = f"D{drone_id} [{role.upper()}] {status.get('state', '?')} {progress_pct}%"
        return marker

    def _publish_markers(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for drone_id in range(self.config.num_drones):
            color = self._color_for(drone_id)
            role = 'leader' if self.config.is_leader(drone_id) else 'follower'

            region = self._regions.get(drone_id)
            if region:
                array.markers.append(self._region_boundary_marker(drone_id, region, color, stamp))
                array.markers.append(self._region_label_marker(drone_id, region, stamp))
                waypoints = self._waypoints.get(drone_id, [])
                if waypoints:
                    array.markers.append(self._waypoints_marker(drone_id, waypoints, color, stamp))

            status = self._status.get(drone_id)
            if status and status.get('position'):
                array.markers.append(
                    self._position_marker(drone_id, status['position'], role, color, stamp))
                array.markers.append(self._label_marker(drone_id, status, role, stamp))

        self._pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerManagerNode()
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
