"""
Interactive CLI for setting swarm mapping goal parameters.

Prompts user for area bounds, flight altitude, waypoint spacing, etc.
Publishes the goal payload to /swarm/goal for the leader node to begin task allocation.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarm_drone.swarm_config import default_config_path, SwarmConfig


def prompt_float(prompt_text, default_val):
    try:
        raw = input(f'{prompt_text} [{default_val}]: ').strip()
        if not raw:
            return float(default_val)
        return float(raw)
    except ValueError:
        print(f'Invalid input, using default value: {default_val}')
        return float(default_val)


class MissionCliNode(Node):

    def __init__(self):
        super().__init__('mission_cli')
        self.pub = self.create_publisher(String, '/swarm/goal', 10)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCliNode()

    try:
        config_path = default_config_path()
        config = SwarmConfig.from_yaml_file(config_path)
        default_min_x = config.min_x
        default_max_x = config.max_x
        default_min_y = config.min_y
        default_max_y = config.max_y
        default_alt = config.flight_altitude
        default_spacing = config.waypoint_spacing
        default_speed = config.cruise_speed
    except Exception:
        default_min_x, default_max_x = -10.0, 10.0
        default_min_y, default_max_y = -10.0, 10.0
        default_alt = 5.0
        default_spacing = 1.0
        default_speed = 1.0

    print('\n' + '=' * 60)
    print('          SWARM DRONE MISSION GOAL CLI SETUP          ')
    print('=' * 60)
    print('Enter custom mission goal parameters (Press ENTER for default):\n')

    min_x = prompt_float(' - Min X coordinate (meters)', default_min_x)
    max_x = prompt_float(' - Max X coordinate (meters)', default_max_x)
    min_y = prompt_float(' - Min Y coordinate (meters)', default_min_y)
    max_y = prompt_float(' - Max Y coordinate (meters)', default_max_y)
    flight_altitude = prompt_float(' - Flight Altitude (meters)', default_alt)
    waypoint_spacing = prompt_float(' - Waypoint Spacing (meters)', default_spacing)
    cruise_speed = prompt_float(' - Cruise Speed (m/s)', default_speed)

    area_w = max_x - min_x
    area_h = max_y - min_y
    total_area = area_w * area_h

    print('\n' + '-' * 60)
    print('MISSION GOAL SUMMARY:')
    print(
        f'  Area Bounds  : [{min_x:.1f}, {max_x:.1f}] x [{min_y:.1f}, {max_y:.1f}] '
        f'({total_area:.1f} m²)')
    print(f'  Flight Alt   : {flight_altitude:.1f} m')
    print(f'  Row Spacing  : {waypoint_spacing:.1f} m')
    print(f'  Cruise Speed : {cruise_speed:.1f} m/s')
    print('-' * 60)

    try:
        input('\nPress ENTER to dispatch goal to Swarm Leader (or Ctrl+C to cancel)...')
    except (KeyboardInterrupt, EOFError):
        print('\nMission CLI cancelled.')
        node.destroy_node()
        rclpy.shutdown()
        return

    payload = {
        'min_x': min_x,
        'max_x': max_x,
        'min_y': min_y,
        'max_y': max_y,
        'flight_altitude': flight_altitude,
        'waypoint_spacing': waypoint_spacing,
        'cruise_speed': cruise_speed,
    }

    msg = String()
    msg.data = json.dumps(payload)

    # Publish multiple times to ensure connection pickup
    for _ in range(5):
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print('\n[SUCCESS] Mission goal successfully dispatched to Leader node!\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
