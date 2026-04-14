#!/usr/bin/env python3
"""
Robot Control UI — Terminal Dashboard
Provides: delivery commands, multi-stop delivery, manual movement,
          sensor monitoring, battery display, delivery history,
          cancel delivery, emergency stop, and status display.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
import threading
import os
import time


LOCATIONS = ['kitchen', 'living_room', 'bedroom', 'charging_station']


class RobotUI(Node):

    def __init__(self):
        super().__init__('robot_ui')
        # Note: use_sim_time is auto-declared in Jazzy — no need to declare it

        # Publishers
        self.delivery_pub = self.create_publisher(String, '/delivery_request', 10)
        self.estop_pub    = self.create_publisher(Bool,   '/emergency_stop',   10)
        self.cancel_pub   = self.create_publisher(Bool,   '/cancel_delivery',  10)
        self.cmdvel_pub   = self.create_publisher(TwistStamped,  '/cmd_vel',          10)

        # State
        self._status       = 'UNKNOWN'
        self._current_loc  = 'UNKNOWN'
        self._front_dist   = float('inf')
        self._estop_active = False
        self._battery      = 100.0
        self._queue        = 'EMPTY'
        self._history      = 'No deliveries yet'

        # Subscriptions
        self.create_subscription(String,    '/delivery_status',  self._cb_status,   10)
        self.create_subscription(String,    '/current_location', self._cb_location, 10)
        self.create_subscription(LaserScan, '/scan',             self._cb_scan,     10)
        self.create_subscription(Bool,      '/emergency_stop',   self._cb_estop,    10)
        self.create_subscription(Float32,   '/battery_level',    self._cb_battery,  10)
        self.create_subscription(String,    '/delivery_queue',   self._cb_queue,    10)
        self.create_subscription(String,    '/delivery_history', self._cb_history,  10)

        # Run UI in a separate thread
        self._ui_thread = threading.Thread(target=self.run_ui, daemon=True)
        self._ui_thread.start()

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_status(self, msg):   self._status = msg.data
    def _cb_location(self, msg): self._current_loc = msg.data
    def _cb_estop(self, msg):    self._estop_active = msg.data
    def _cb_battery(self, msg):  self._battery = msg.data
    def _cb_queue(self, msg):    self._queue = msg.data
    def _cb_history(self, msg):  self._history = msg.data

    def _cb_scan(self, msg):
        n = len(msg.ranges)
        if n < 30:
            return
        front = [r for r in list(msg.ranges[:15]) + list(msg.ranges[n-15:]) if r > 0.01]
        self._front_dist = min(front) if front else float('inf')

    # ── UI Loop ──────────────────────────────────────────────────────────────
    def run_ui(self):
        time.sleep(2)  # Wait for ROS connections
        while rclpy.ok():
            os.system('clear')
            estop_str = '🔴 ACTIVE' if self._estop_active else '🟢 CLEAR'
            front_str = (f'{self._front_dist:.2f}m'
                         if self._front_dist != float('inf') else 'N/A')
            batt_str  = f'{self._battery:.0f}%'
            if self._battery <= 25:
                batt_str = f'🪫 {batt_str} LOW'
            elif self._battery >= 90:
                batt_str = f'🔋 {batt_str}'
            else:
                batt_str = f'🔋 {batt_str}'

            W = 54  # box width (inner)

            print('╔' + '═' * W + '╗')
            print('║' + '  🤖  Home Service Delivery Robot — Control UI  '.center(W) + '║')
            print('╠' + '═' * W + '╣')

            # Status section
            self._row('Status',         self._status, W)
            self._row('Location',       self._current_loc, W)
            self._row('Front Sensor',   front_str, W)
            self._row('Emergency Stop', estop_str, W)
            self._row('Battery',        batt_str, W)
            self._row('Queue',          self._queue, W)

            print('╠' + '═' * W + '╣')
            print('║' + '  DELIVERY COMMANDS'.ljust(W) + '║')
            for i, loc in enumerate(LOCATIONS, 1):
                print('║' + f'    {i}. Send to {loc}'.ljust(W) + '║')
            print('║' + f'    5. Multi-stop delivery (enter route)'.ljust(W) + '║')

            print('╠' + '─' * W + '╣')
            print('║' + '  CONTROLS'.ljust(W) + '║')
            print('║' + '    6. Emergency STOP      7. Resume'.ljust(W) + '║')
            print('║' + '    8. Cancel Delivery'.ljust(W) + '║')

            print('╠' + '─' * W + '╣')
            print('║' + '  MANUAL MOVEMENT'.ljust(W) + '║')
            print('║' + '    W. Forward    S. Backward'.ljust(W) + '║')
            print('║' + '    A. Turn Left  D. Turn Right'.ljust(W) + '║')
            print('║' + '    X. Halt'.ljust(W) + '║')

            print('╠' + '─' * W + '╣')
            print('║' + '  HISTORY'.ljust(W) + '║')
            # Wrap long history across lines
            hist_lines = self._wrap_text(self._history, W - 4)
            for hl in hist_lines[:3]:
                print('║' + ('  ' + hl).ljust(W) + '║')

            print('╠' + '─' * W + '╣')
            print('║' + '    Q. Quit UI'.ljust(W) + '║')
            print('╚' + '═' * W + '╝')
            print('Enter choice: ', end='', flush=True)

            try:
                choice = input().strip().upper()
            except EOFError:
                break
            self.handle_input(choice)

    def _row(self, label, value, width):
        content = f'  {label:<16}: {value}'
        # Truncate if too long
        if len(content) > width:
            content = content[:width - 3] + '...'
        print('║' + content.ljust(width) + '║')

    def _wrap_text(self, text, width):
        """Simple word-wrap."""
        if len(text) <= width:
            return [text]
        lines = []
        while text:
            if len(text) <= width:
                lines.append(text)
                break
            # Find last space before width
            idx = text.rfind(' ', 0, width)
            if idx == -1:
                idx = width
            lines.append(text[:idx])
            text = text[idx:].lstrip()
        return lines

    def handle_input(self, choice):
        if choice in ['1', '2', '3', '4']:
            loc = LOCATIONS[int(choice) - 1]
            msg = String()
            msg.data = loc
            self.delivery_pub.publish(msg)
            print(f'📦 Sending robot to: {loc}')
            time.sleep(1)

        elif choice == '5':
            print('\n📋 Multi-stop delivery')
            print(f'   Available: {", ".join(LOCATIONS)}')
            print('   Enter comma-separated route (e.g., kitchen,bedroom,charging_station):')
            try:
                route = input('   Route: ').strip()
            except EOFError:
                return
            if route:
                msg = String()
                msg.data = route
                self.delivery_pub.publish(msg)
                print(f'📦 Queued multi-stop route: {route}')
                time.sleep(1)

        elif choice == '6':
            msg = Bool()
            msg.data = True
            self.estop_pub.publish(msg)
            print('🛑 Emergency stop sent!')
            time.sleep(1)

        elif choice == '7':
            msg = Bool()
            msg.data = False
            self.estop_pub.publish(msg)
            print('✅ Resume sent!')
            time.sleep(1)

        elif choice == '8':
            msg = Bool()
            msg.data = True
            self.cancel_pub.publish(msg)
            print('🚫 Cancel delivery sent!')
            time.sleep(1)

        elif choice == 'W':
            self._move(0.2, 0.0)
        elif choice == 'S':
            self._move(-0.2, 0.0)
        elif choice == 'A':
            self._move(0.0, 0.5)
        elif choice == 'D':
            self._move(0.0, -0.5)
        elif choice == 'X':
            self._move(0.0, 0.0)

        elif choice == 'Q':
            print('Goodbye!')
            raise SystemExit(0)

    def _move(self, linear, angular):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        self.cmdvel_pub.publish(twist)
        time.sleep(0.5)
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.cmdvel_pub.publish(stop_msg)  # Stop after brief movement


def main(args=None):
    rclpy.init(args=args)
    node = RobotUI()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
