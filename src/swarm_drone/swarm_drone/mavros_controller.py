"""
MAVROS / PX4 backend for DroneController.

Streams PoseStamped setpoints at >=20 Hz, handles OFFBOARD + arming, and
reads local ENU pose. Same takeoff/move_to/stop/land API as the gazebo
backend. Requires mavros_msgs at import time (only loaded when backend
is mavros).
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.qos import qos_profile_sensor_data


class MavrosDroneController:
    """PX4 OFFBOARD position-setpoint backend used when backend is mavros."""

    def __init__(
            self, node, cruise_speed=1.0, waypoint_tolerance=0.3,
            setpoint_rate_hz=20.0):
        self._node = node
        self._cruise_speed = cruise_speed
        self._tolerance = waypoint_tolerance

        self._position = None
        self._orientation = None
        self._target = None
        self._fcu_state = None
        self._mode = 'idle'  # idle | takeoff_prep | seek | hover | landing | landed
        self._on_reached = None
        self._takeoff_alt = None
        self._last_offboard_request = 0.0
        self._last_arm_request = 0.0
        self._last_land_request = 0.0
        self._service_period = 1.0

        self._setpoint_pub = node.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)
        node.create_subscription(
            PoseStamped, 'mavros/local_position/pose',
            self._pose_callback, qos_profile_sensor_data)
        node.create_subscription(State, 'mavros/state', self._state_callback, 10)

        self._arming_client = node.create_client(CommandBool, 'mavros/cmd/arming')
        self._mode_client = node.create_client(SetMode, 'mavros/set_mode')
        self._land_client = node.create_client(CommandTOL, 'mavros/cmd/land')

        node.create_timer(1.0 / setpoint_rate_hz, self._control_step)

    def _pose_callback(self, msg):
        p = msg.pose.position
        self._position = (p.x, p.y, p.z)
        self._orientation = msg.pose.orientation

    def _state_callback(self, msg):
        self._fcu_state = msg

    def has_odometry(self):
        return self._position is not None

    def current_position(self):
        return self._position

    def distance_to(self, x, y, z):
        if self._position is None:
            return None
        return math.dist(self._position, (x, y, z))

    def is_idle(self):
        return self._mode in ('idle', 'landed')

    def _is_connected(self):
        return self._fcu_state is not None and self._fcu_state.connected

    def _is_armed(self):
        return self._fcu_state is not None and self._fcu_state.armed

    def _is_offboard(self):
        return (
            self._fcu_state is not None
            and self._fcu_state.mode == 'OFFBOARD')

    def move_to(self, x, y, z, on_reached=None):
        """Hold/fly to (x, y, z) by streaming a local ENU position setpoint."""
        self._target = (x, y, z)
        self._mode = 'seek'
        self._on_reached = on_reached

    def takeoff(self, altitude, on_complete=None):
        """Stream setpoints, enter OFFBOARD, arm, then climb to altitude."""
        self._takeoff_alt = altitude
        self._on_reached = on_complete
        self._mode = 'takeoff_prep'
        if self._position is not None:
            self._target = (self._position[0], self._position[1], altitude)

    def land(self, on_complete=None):
        """Request PX4 AUTO.LAND and wait until the vehicle disarms."""
        self._on_reached = on_complete
        self._mode = 'landing'
        self._last_land_request = 0.0

    def stop(self):
        """Hold the current local pose (do not publish zero velocity)."""
        if self._position is not None:
            self._target = self._position
        self._mode = 'hover'
        self._on_reached = None

    def _publish_setpoint(self):
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        if self._target is not None:
            pose.pose.position.x = float(self._target[0])
            pose.pose.position.y = float(self._target[1])
            pose.pose.position.z = float(self._target[2])
        elif self._position is not None:
            pose.pose.position.x = float(self._position[0])
            pose.pose.position.y = float(self._position[1])
            pose.pose.position.z = float(self._position[2])
        else:
            pose.pose.position.z = 0.0
        if self._orientation is not None:
            pose.pose.orientation = self._orientation
        else:
            pose.pose.orientation.w = 1.0
        self._setpoint_pub.publish(pose)

    def _try_set_offboard(self, now):
        if now - self._last_offboard_request < self._service_period:
            return
        if not self._mode_client.service_is_ready():
            return
        self._last_offboard_request = now
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self._mode_client.call_async(req)
        future.add_done_callback(self._on_set_mode_done)

    def _on_set_mode_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f'set_mode failed: {exc}')
            return
        if result is not None and not result.mode_sent:
            self._node.get_logger().warn('PX4 rejected OFFBOARD mode request.')

    def _try_arm(self, now, value=True):
        if now - self._last_arm_request < self._service_period:
            return
        if not self._arming_client.service_is_ready():
            return
        self._last_arm_request = now
        req = CommandBool.Request()
        req.value = value
        future = self._arming_client.call_async(req)
        future.add_done_callback(self._on_arm_done)

    def _on_arm_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f'arming failed: {exc}')
            return
        if result is not None and not result.success:
            self._node.get_logger().warn(
                f'Arming request unsuccessful (success={result.success}).')

    def _try_land(self, now):
        if now - self._last_land_request < self._service_period:
            return
        if not self._land_client.service_is_ready():
            return
        self._last_land_request = now
        req = CommandTOL.Request()
        future = self._land_client.call_async(req)
        future.add_done_callback(self._on_land_done)

    def _on_land_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f'land command failed: {exc}')
            return
        if result is not None and not result.success:
            self._node.get_logger().warn('PX4 land command was not accepted.')

    def _fire_reached(self):
        callback, self._on_reached = self._on_reached, None
        if callback:
            callback()

    def _control_step(self):
        now = time.monotonic()
        self._publish_setpoint()

        if self._mode == 'idle' or self._mode == 'landed' or self._mode == 'hover':
            return

        if self._mode == 'takeoff_prep':
            if self._position is None or not self._is_connected():
                return
            if self._target is None:
                self._target = (
                    self._position[0], self._position[1], self._takeoff_alt)
            if not self._is_offboard():
                self._try_set_offboard(now)
                return
            if not self._is_armed():
                self._try_arm(now, True)
                return
            self._mode = 'seek'
            return

        if self._mode == 'seek':
            if self._position is None or self._target is None:
                return
            if self.distance_to(*self._target) <= self._tolerance:
                self._mode = 'hover'
                self._fire_reached()
            return

        if self._mode == 'landing':
            if self._is_armed():
                self._try_land(now)
                return
            self._mode = 'landed'
            self._fire_reached()
