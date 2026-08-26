"""
Low-level per-drone control interface.

DroneController is the ONLY piece of code that knows how a drone is
actually actuated in simulation (gz-sim VelocityControl + OdometryPublisher,
bridged to plain ROS 2 `cmd_vel` / `odom` topics in the drone's namespace).

Leader/follower/coverage logic talks to this object through takeoff(),
move_to(), stop() and land() only. To later swap the backend for
MAVROS/PX4, only this file needs to change - callers are unaffected.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DroneController:

    def __init__(self, node, cruise_speed=1.0, waypoint_tolerance=0.3, control_rate_hz=10.0):
        self._node = node
        self._cruise_speed = cruise_speed
        self._tolerance = waypoint_tolerance

        self._position = None  # (x, y, z) from odometry, None until first message
        self._target = None
        self._mode = 'idle'  # idle | seek | hover | landed
        self._on_reached = None

        self._cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
        node.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        node.create_timer(1.0 / control_rate_hz, self._control_step)

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        self._position = (p.x, p.y, p.z)

    def has_odometry(self):
        return self._position is not None

    def current_position(self):
        return self._position

    def distance_to(self, x, y, z):
        if self._position is None:
            return None
        return math.dist(self._position, (x, y, z))

    def is_idle(self):
        return self._mode == 'idle'

    def move_to(self, x, y, z, on_reached=None):
        """Fly (in a straight line, simple P-control) to (x, y, z)."""
        self._target = (x, y, z)
        self._mode = 'seek'
        self._on_reached = on_reached

    def takeoff(self, altitude, on_complete=None):
        """Rise straight up to `altitude` from the current (x, y)."""
        x, y = (self._position[0], self._position[1]) if self._position else (0.0, 0.0)
        self.move_to(x, y, altitude, on_reached=on_complete)

    def land(self, on_complete=None):
        """Descend straight down to just above ground at the current (x, y)."""
        if self._position is None:
            return

        def _landed():
            self._mode = 'idle'
            self._publish_twist(0.0, 0.0, 0.0)
            if on_complete:
                on_complete()

        x, y = self._position[0], self._position[1]
        self.move_to(x, y, 0.15, on_reached=_landed)

    def stop(self):
        """Cancel any active target and hover in place."""
        self._mode = 'hover'
        self._on_reached = None
        self._publish_twist(0.0, 0.0, 0.0)

    def _publish_twist(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self._cmd_pub.publish(msg)

    def _control_step(self):
        if self._mode != 'seek' or self._position is None or self._target is None:
            return

        tx, ty, tz = self._target
        dx, dy, dz = tx - self._position[0], ty - self._position[1], tz - self._position[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= self._tolerance:
            self._mode = 'hover'
            self._publish_twist(0.0, 0.0, 0.0)
            callback, self._on_reached = self._on_reached, None
            if callback:
                callback()
            return

        scale = min(self._cruise_speed / distance, 1.0)
        self._publish_twist(dx * scale, dy * scale, dz * scale)
