#!/usr/bin/env python3
"""
Emergency Stop Node
Monitors for obstacle proximity and provides a software kill switch.
Publishes to /emergency_stop and directly stops the robot via /cmd_vel.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
import sys
import select
import termios
import tty


SAFE_DISTANCE = 0.15  # metres — emergency stop if obstacle closer than this


class EmergencyStopNode(Node):

    def __init__(self):
        super().__init__('emergency_stop_node')
        # Note: use_sim_time is auto-declared in Jazzy — no need to declare it

        # Publishers
        self.estop_pub  = self.create_publisher(Bool,   '/emergency_stop', 10)
        self.cmdvel_pub = self.create_publisher(TwistStamped,  '/cmd_vel',        10)
        self.status_pub = self.create_publisher(String, '/estop_status',   10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self._stopped = False

        # Timer to check keyboard (non-blocking)
        self.create_timer(0.1, self.check_keyboard)

        self.get_logger().info(
            '🔴 Emergency Stop Node active.\n'
            '   Auto-stop if obstacle < 0.15m\n'
            '   Press  S  to manual stop\n'
            '   Press  R  to resume\n'
            '   Press  Q  to quit'
        )

        # Save terminal settings for key capture
        try:
            self._settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception:
            self._settings = None

    # ── Laser-based auto stop ────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        if self._stopped:
            return
        # Check front 30° arc (indices around 0)
        ranges = msg.ranges
        n = len(ranges)
        if n < 30:
            return
        front_indices = list(range(0, 15)) + list(range(n - 15, n))
        close_obstacles = [
            r for i in front_indices
            if (r := ranges[i]) > 0.01 and r < SAFE_DISTANCE
        ]
        if close_obstacles:
            self.get_logger().warn(
                f'⚠️  Obstacle detected at {min(close_obstacles):.2f}m — AUTO STOP'
            )
            self.trigger_stop(reason='OBSTACLE_PROXIMITY')

    # ── Keyboard handler ─────────────────────────────────────────────────────
    def check_keyboard(self):
        if self._settings is None:
            return
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == 's':
                    self.trigger_stop(reason='MANUAL_STOP')
                elif key == 'r':
                    self.clear_stop()
                elif key == 'q':
                    self.get_logger().info('Quit requested.')
                    rclpy.shutdown()
        except Exception:
            pass

    # ── Stop / Resume ────────────────────────────────────────────────────────
    def trigger_stop(self, reason='MANUAL_STOP'):
        self._stopped = True
        # Publish zero velocity
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.cmdvel_pub.publish(stop_msg)
        # Signal other nodes
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)
        status = String()
        status.data = f'STOPPED:{reason}'
        self.status_pub.publish(status)
        self.get_logger().warn(f'🛑 STOPPED — reason: {reason}')

    def clear_stop(self):
        self._stopped = False
        msg = Bool()
        msg.data = False
        self.estop_pub.publish(msg)
        status = String()
        status.data = 'RESUMED'
        self.status_pub.publish(status)
        self.get_logger().info('✅ Emergency stop cleared — resuming.')

    def destroy_node(self):
        # Restore terminal
        if self._settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
