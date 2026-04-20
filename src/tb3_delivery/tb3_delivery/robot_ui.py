#!/usr/bin/env python3
"""
Robot Control UI — Home Cleaning Robot Dashboard
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import threading
import os
import time

ROOMS = ['kitchen', 'living_room', 'bedroom']


class RobotUI(Node):

    def __init__(self):
        super().__init__('robot_ui')

        self.clean_pub  = self.create_publisher(String,       '/clean_request',   10)
        self.estop_pub  = self.create_publisher(Bool,         '/emergency_stop',  10)
        self.cancel_pub = self.create_publisher(Bool,         '/cancel_cleaning', 10)
        self.cmdvel_pub = self.create_publisher(TwistStamped, '/cmd_vel',         10)

        self._status    = 'UNKNOWN'
        self._room      = 'UNKNOWN'
        self._progress  = 'N/A'
        self._front     = float('inf')
        self._estop     = False
        self._battery   = 100.0
        self._queue     = 'EMPTY'
        self._history   = 'No sessions yet'

        self.create_subscription(String,    '/cleaning_status',   self._cb_status,   10)
        self.create_subscription(String,    '/current_room',      self._cb_room,     10)
        self.create_subscription(String,    '/cleaning_progress', self._cb_progress, 10)
        self.create_subscription(LaserScan, '/scan',              self._cb_scan,     10)
        self.create_subscription(Bool,      '/emergency_stop',    self._cb_estop,    10)
        self.create_subscription(Float32,   '/battery_level',     self._cb_battery,  10)
        self.create_subscription(String,    '/cleaning_queue',    self._cb_queue,    10)
        self.create_subscription(String,    '/cleaning_history',  self._cb_history,  10)

        self._ui_thread = threading.Thread(target=self.run_ui, daemon=True)
        self._ui_thread.start()

    def _cb_status(self, m):   self._status   = m.data
    def _cb_room(self, m):     self._room     = m.data
    def _cb_progress(self, m): self._progress = m.data
    def _cb_estop(self, m):    self._estop    = m.data
    def _cb_battery(self, m):  self._battery  = m.data
    def _cb_queue(self, m):    self._queue    = m.data
    def _cb_history(self, m):  self._history  = m.data

    def _cb_scan(self, msg):
        n = len(msg.ranges)
        if n < 30:
            return
        front = [r for r in list(msg.ranges[:15]) + list(msg.ranges[n-15:]) if r > 0.01]
        self._front = min(front) if front else float('inf')

    def run_ui(self):
        time.sleep(2)
        while rclpy.ok():
            os.system('clear')
            W = 56
            estop_str = '🔴 ACTIVE' if self._estop else '🟢 CLEAR'
            front_str = f'{self._front:.2f}m' if self._front != float('inf') else 'N/A'
            batt_icon = '🪫' if self._battery <= 25 else '🔋'
            batt_str  = f'{batt_icon} {self._battery:.0f}%'

            print('╔' + '═' * W + '╗')
            print('║' + '  🧹  Home Cleaning Robot — Control UI  '.center(W) + '║')
            print('╠' + '═' * W + '╣')
            self._row('Status',         self._status,   W)
            self._row('Current Room',   self._room,     W)
            self._row('Progress',       self._progress, W)
            self._row('Front Sensor',   front_str,      W)
            self._row('Emergency Stop', estop_str,      W)
            self._row('Battery',        batt_str,       W)
            self._row('Queue',          self._queue,    W)
            print('╠' + '═' * W + '╣')
            print('║' + '  CLEAN COMMANDS'.ljust(W) + '║')
            for i, r in enumerate(ROOMS, 1):
                print('║' + f'    {i}. Clean {r}'.ljust(W) + '║')
            print('║' + '    4. Clean ALL rooms'.ljust(W) + '║')
            print('║' + '    5. Custom room sequence'.ljust(W) + '║')
            print('║' + '    6. Go to charging station'.ljust(W) + '║')
            print('╠' + '─' * W + '╣')
            print('║' + '  CONTROLS'.ljust(W) + '║')
            print('║' + '    7. Emergency STOP      8. Resume'.ljust(W) + '║')
            print('║' + '    9. Cancel current cleaning'.ljust(W) + '║')
            print('╠' + '─' * W + '╣')
            print('║' + '  MANUAL MOVEMENT'.ljust(W) + '║')
            print('║' + '    W. Forward   S. Backward'.ljust(W) + '║')
            print('║' + '    A. Turn Left  D. Turn Right  X. Halt'.ljust(W) + '║')
            print('╠' + '─' * W + '╣')
            print('║' + '  HISTORY'.ljust(W) + '║')
            for hl in self._wrap(self._history, W - 4)[:3]:
                print('║' + ('  ' + hl).ljust(W) + '║')
            print('╠' + '─' * W + '╣')
            print('║' + '    Q. Quit'.ljust(W) + '║')
            print('╚' + '═' * W + '╝')
            print('Enter choice: ', end='', flush=True)

            try:
                choice = input().strip().upper()
            except EOFError:
                break
            self._handle(choice)

    def _row(self, label, value, width):
        content = f'  {label:<16}: {value}'
        if len(content) > width:
            content = content[:width - 3] + '...'
        print('║' + content.ljust(width) + '║')

    def _wrap(self, text, width):
        if len(text) <= width:
            return [text]
        lines = []
        while text:
            if len(text) <= width:
                lines.append(text); break
            idx = text.rfind(' ', 0, width)
            if idx == -1: idx = width
            lines.append(text[:idx])
            text = text[idx:].lstrip()
        return lines

    def _handle(self, choice):
        if choice in ['1', '2', '3']:
            room = ROOMS[int(choice) - 1]
            self._send_clean(room)

        elif choice == '4':
            self._send_clean('all')
            print('🧹 Queued all rooms')
            time.sleep(1)

        elif choice == '5':
            print(f'\n  Available: {", ".join(ROOMS)}')
            print('  Enter comma-separated sequence:')
            try:
                seq = input('  Sequence: ').strip()
            except EOFError:
                return
            if seq:
                self._send_clean(seq)

        elif choice == '6':
            self._send_clean('__charge__')
            print('🔌 Heading to charging station')
            time.sleep(1)

        elif choice == '7':
            msg = Bool(); msg.data = True
            self.estop_pub.publish(msg)
            print('🛑 Emergency stop!')
            time.sleep(1)

        elif choice == '8':
            msg = Bool(); msg.data = False
            self.estop_pub.publish(msg)
            print('✅ Resumed')
            time.sleep(1)

        elif choice == '9':
            msg = Bool(); msg.data = True
            self.cancel_pub.publish(msg)
            print('🚫 Cancelled')
            time.sleep(1)

        elif choice == 'W': self._move(0.2,  0.0)
        elif choice == 'S': self._move(-0.2, 0.0)
        elif choice == 'A': self._move(0.0,  0.5)
        elif choice == 'D': self._move(0.0, -0.5)
        elif choice == 'X': self._move(0.0,  0.0)
        elif choice == 'Q':
            print('Goodbye!')
            raise SystemExit(0)

    def _send_clean(self, room_or_seq):
        msg = String(); msg.data = room_or_seq
        self.clean_pub.publish(msg)
        print(f'🧹 Cleaning request: {room_or_seq}')
        time.sleep(1)

    def _move(self, linear, angular):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.twist.linear.x  = linear
        t.twist.angular.z = angular
        self.cmdvel_pub.publish(t)
        time.sleep(0.5)
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = 'base_link'
        self.cmdvel_pub.publish(stop)


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