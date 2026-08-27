"""
Interactive terminal node for manual teleoperation override of swarm drones.

User selects a drone_id to override. The node notifies the swarm leader/follower
of the manual override, takes keyboard control of /drone_<id>/cmd_vel, and upon
exit sends a resume signal so the drone resumes its autonomous path and the leader
resumes tracking region progress.
"""

import json
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarm_drone.swarm_config import default_config_path, SwarmConfig


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ManualControlNode(Node):

    def __init__(self):
        super().__init__('manual_control')
        self.override_pub = self.create_publisher(String, '/swarm/manual_override', 10)
        self.cmd_pub = None
        self.active_drone_id = None

    def select_drone(self, drone_id):
        self.active_drone_id = drone_id
        topic = f'/drone_{drone_id}/cmd_vel'
        self.cmd_pub = self.create_publisher(Twist, topic, 10)

        # Notify swarm leader & drone of override
        msg = String()
        msg.data = json.dumps({'drone_id': drone_id, 'action': 'override'})
        self.override_pub.publish(msg)

    def release_drone(self):
        if self.active_drone_id is not None:
            # Publish 0 velocity
            if self.cmd_pub:
                self.cmd_pub.publish(Twist())

            # Notify swarm leader & drone of resume
            msg = String()
            msg.data = json.dumps({'drone_id': self.active_drone_id, 'action': 'resume'})
            self.override_pub.publish(msg)
            self.active_drone_id = None

    def send_twist(self, vx, vy, vz):
        if self.cmd_pub:
            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.linear.z = float(vz)
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()

    try:
        config = SwarmConfig.from_yaml_file(default_config_path())
        num_drones = config.num_drones
    except Exception:
        num_drones = 4

    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            print('\n' + '=' * 60)
            print('        SWARM DRONE MANUAL CONTROL INTERFACE        ')
            print('=' * 60)
            print(f'Available Drone IDs: 0 to {num_drones - 1}')
            raw_id = input('Enter Drone ID to control (or "q" to exit): ').strip()

            if raw_id.lower() in ('q', 'exit', 'quit'):
                break

            try:
                drone_id = int(raw_id)
                if not (0 <= drone_id < num_drones):
                    print(f'Invalid drone ID. Must be between 0 and {num_drones - 1}.')
                    continue
            except ValueError:
                print('Invalid input. Please enter an integer drone ID.')
                continue

            node.select_drone(drone_id)

            print('\n' + '*' * 60)
            print(f'   MANUAL OVERRIDE ACTIVE FOR DRONE {drone_id}')
            print('*' * 60)
            print('  Controls:')
            print('    W / I : Forward          S / , : Backward')
            print('    A / J : Left             D / L : Right')
            print('    E / U : Up               C / O : Down')
            print('    Space / K : Stop/Hover')
            print('    Q / X / ESC : Release Control & Resume Auto Mission')
            print('*' * 60 + '\n')

            speed = 1.0
            vx, vy, vz = 0.0, 0.0, 0.0

            while rclpy.ok():
                key = get_key(settings, timeout=0.1)

                if key in ('q', 'x', '\x1b'):  # Exit keys or ESC
                    break

                if key in ('w', 'i'):
                    vx, vy, vz = speed, 0.0, 0.0
                elif key in ('s', ','):
                    vx, vy, vz = -speed, 0.0, 0.0
                elif key in ('a', 'j'):
                    vx, vy, vz = 0.0, speed, 0.0
                elif key in ('d', 'l'):
                    vx, vy, vz = 0.0, -speed, 0.0
                elif key in ('e', 'u'):
                    vx, vy, vz = 0.0, 0.0, speed
                elif key in ('c', 'o'):
                    vx, vy, vz = 0.0, 0.0, -speed
                elif key in (' ', 'k'):
                    vx, vy, vz = 0.0, 0.0, 0.0
                elif key == '+':
                    speed = min(5.0, speed + 0.5)
                    print(f'Speed increased to {speed:.1f} m/s')
                elif key == '-':
                    speed = max(0.2, speed - 0.5)
                    print(f'Speed decreased to {speed:.1f} m/s')

                node.send_twist(vx, vy, vz)
                rclpy.spin_once(node, timeout_sec=0.01)

            print(
                f'\n[RELEASED] Drone {drone_id} control released. '
                f'Leader resuming region progress tracking.')
            node.release_drone()

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.release_drone()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nManual Control CLI closed.')


if __name__ == '__main__':
    main()
