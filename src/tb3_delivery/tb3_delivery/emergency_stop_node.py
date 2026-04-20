#!/usr/bin/env python3
"""
Emergency Stop Node
Monitors for obstacle proximity and provides a software kill switch.
Publishes to /emergency_stop and directly stops the robot via /cmd_vel.
Press S to stop, R to resume, Q to quit.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, String
import sys
import select
import termios
import tty


SAFE_DISTANCE = 0.15  # metres — emergency stop if obstacle closer than this


class EmergencyStopNode(Node):

    def __init__(self):
        super().__init__('emergency_stop_node')

        # Publishers
        self.estop_pub  = self.create_publisher(Bool,         '/emergency_stop', 10)
        self.cmdvel_pub = self.create_publisher(TwistStamped, '/cmd_vel',        10)
        self.status_pub = self.create_publisher(String,       '/estop_status',   10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self._stopped        = False
        self._auto_stopped   = False  # True when stopped by obstacle (not manual)

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

    # ── Laser-based auto stop ─────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)
        if n < 30:
            return

        # Only check front 30° arc
        front_indices = list(range(0, 15)) + list(range(n - 15, n))
        close = [
            ranges[i] for i in front_indices
            if ranges[i] > 0.01 and ranges[i] < SAFE_DISTANCE
        ]

        if close and not self._stopped:
            self.get_logger().warn(
                f'⚠️  Obstacle at {min(close):.2f}m — AUTO STOP'
            )
            self._auto_stopped = True
            self.trigger_stop(reason='OBSTACLE_PROXIMITY')

        elif not close and self._auto_stopped and self._stopped:
            # Obstacle cleared — auto resume
            self.get_logger().info('✅ Obstacle cleared — auto resuming')
            self._auto_stopped = False
            self.clear_stop()

    # ── Keyboard handler ──────────────────────────────────────────────────
    def check_keyboard(self):
        if self._settings is None:
            return
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == 's':
                    self._auto_stopped = False  # Manual stop, don't auto-resume
                    self.trigger_stop(reason='MANUAL_STOP')
                elif key == 'r':
                    self._auto_stopped = False
                    self.clear_stop()
                elif key == 'q':
                    self.get_logger().info('Quit requested.')
                    rclpy.shutdown()
        except Exception:
            pass

    # ── Stop ──────────────────────────────────────────────────────────────
    def trigger_stop(self, reason='MANUAL_STOP'):
        if self._stopped:
            return  # Already stopped, don't publish again

        self._stopped = True

        # Zero velocity immediately
        stop_msg = TwistStamped()
        stop_msg.header.stamp    = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.cmdvel_pub.publish(stop_msg)

        # Signal cleaning_node to stop and save position
        msg      = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

        status      = String()
        status.data = f'STOPPED:{reason}'
        self.status_pub.publish(status)

        self.get_logger().warn(f'🛑 STOPPED — reason: {reason}')

    # ── Resume ────────────────────────────────────────────────────────────
    def clear_stop(self):
        if not self._stopped:
            return  # Already running, don't publish again

        self._stopped = False

        # Signal cleaning_node to resume from saved waypoint
        msg      = Bool()
        msg.data = False
        self.estop_pub.publish(msg)

        status      = String()
        status.data = 'RESUMED'
        self.status_pub.publish(status)

        self.get_logger().info('✅ Emergency stop cleared — cleaning_node will resume.')

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
