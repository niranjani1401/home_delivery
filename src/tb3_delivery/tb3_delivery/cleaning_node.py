#!/usr/bin/env python3
"""
Home Cleaning Robot Node
Performs coverage cleaning of rooms using a boustrophedon (lawnmower)
path pattern. Uses lidar for obstacle detection during cleaning.
Saves trajectory to file. Returns to charging station after each room.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float32
from action_msgs.msg import GoalStatus
import math
import os
import csv
import json
from collections import deque
from datetime import datetime


# ── Room definitions (bounding boxes in world coordinates) ──────────────────
ROOMS = {
    'kitchen':     {'x_min': 1.5,  'x_max': 3.0,  'y_min': 1.5,  'y_max': 3.0},
    'living_room': {'x_min': -3.0, 'x_max': -0.6,  'y_min': -3.0, 'y_max': -0.6},
    'bedroom':     {'x_min': -3.0, 'x_max': -0.8,  'y_min': 1.2,  'y_max': 3.5},
}
CHARGING_STATION      = (2.0, -2.0, 0.0)
COVERAGE_STEP         = 0.40
OBSTACLE_DIST         = 0.20
TRAJECTORY_DIR        = os.path.expanduser('~/tb3_delivery_ws/trajectories')
BATTERY_FULL          = 100.0
BATTERY_DRAIN_PER_ROOM = 20.0
BATTERY_LOW_THRESHOLD = 25.0


def generate_coverage_path(room: dict, step: float = COVERAGE_STEP):
    waypoints = []
    x_min = room['x_min']
    x_max = room['x_max']
    y_min = room['y_min']
    y_max = room['y_max']

    x = x_min
    col = 0
    while x <= x_max + 0.01:
        if col % 2 == 0:
            y_start, y_end = y_min, y_max
            yaw = 1.57
        else:
            y_start, y_end = y_max, y_min
            yaw = -1.57

        waypoints.append((x, y_start, yaw))
        waypoints.append((x, y_end,   yaw))
        x += step
        col += 1

    return waypoints


class CleaningNode(Node):

    def __init__(self):
        super().__init__('cleaning_node')

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(String,    '/clean_request',   self.clean_callback,  10)
        self.create_subscription(Bool,      '/emergency_stop',  self.estop_callback,  10)
        self.create_subscription(Bool,      '/cancel_cleaning', self.cancel_callback, 10)
        self.create_subscription(LaserScan, '/scan',            self.scan_callback,   10)

        # ── Publishers ────────────────────────────────────────────────────
        self.status_pub   = self.create_publisher(String,  '/cleaning_status',   10)
        self.room_pub     = self.create_publisher(String,  '/current_room',      10)
        self.progress_pub = self.create_publisher(String,  '/cleaning_progress', 10)
        self.battery_pub  = self.create_publisher(Float32, '/battery_level',     10)
        self.history_pub  = self.create_publisher(String,  '/cleaning_history',  10)
        self.queue_pub    = self.create_publisher(String,  '/cleaning_queue',    10)

        # ── State ─────────────────────────────────────────────────────────
        self._emergency_stop      = False
        self._obstacle_paused     = False
        self._is_cleaning         = False
        self._cancel_requested    = False
        self._current_goal_handle = None
        self._cleaning_queue      = deque()
        self._cleaning_history    = []
        self._battery_level       = BATTERY_FULL

        # Trajectory logging
        os.makedirs(TRAJECTORY_DIR, exist_ok=True)
        self._trajectory_log  = []
        self._current_room    = None
        self._waypoint_index  = 0
        self._total_waypoints = 0

        self.create_timer(2.0,  self._publish_battery)
        self.create_timer(30.0, self._autosave_trajectory)

        self.get_logger().info(
            '🧹 Cleaning Node started\n'
            '   /clean_request   — room name, comma list, or "all"\n'
            '   /cancel_cleaning — Bool true to cancel\n'
            '   /cleaning_status — current status\n'
            '   /cleaning_progress — waypoint X/Y\n'
            f'  Trajectories saved to: {TRAJECTORY_DIR}'
        )
        self.publish_status('IDLE')

    # ── Lidar obstacle detection ──────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        if self._emergency_stop or not self._is_cleaning:
            return

        ranges = msg.ranges
        n = len(ranges)
        if n < 30:
            return

        # Only check FRONT 60° arc while navigating.
        # Side/rear walls will always be close — don't trigger on them.
        front_indices = list(range(0, 30)) + list(range(n - 30, n))
        close = [
            ranges[i] for i in front_indices
            if ranges[i] > 0.01 and ranges[i] < OBSTACLE_DIST
        ]

        if close and not self._obstacle_paused:
            self._obstacle_paused = True
            self.get_logger().warn(
                f'⚠️  Obstacle at {min(close):.2f}m — pausing cleaning'
            )
            self.publish_status(f'OBSTACLE_PAUSE:{self._current_room}')
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()

        elif not close and self._obstacle_paused:
            self._obstacle_paused = False
            self.get_logger().info('✅ Obstacle cleared — resuming cleaning')
            self.publish_status(f'RESUMING:{self._current_room}')
            if self._current_room and self._is_cleaning:
                self._resume_from_waypoint()

    def _resume_from_waypoint(self):
        """Re-send the current waypoint after obstacle clears."""
        if self._current_room not in ROOMS:
            return
        path = generate_coverage_path(ROOMS[self._current_room])
        if self._waypoint_index < len(path):
            wp = path[self._waypoint_index]
            self._navigate_to_waypoint(wp, self._waypoint_index, len(path))

    # ── Emergency stop ────────────────────────────────────────────────────
    def estop_callback(self, msg: Bool):
        if msg.data:
            self._emergency_stop = True
            self.get_logger().warn('🛑 EMERGENCY STOP')
            self.publish_status('EMERGENCY_STOP')
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()
            self._save_trajectory(self._current_room or 'unknown', aborted=True)
        else:
            self._emergency_stop = False
            self.get_logger().info('✅ Emergency stop cleared')
            self.publish_status('IDLE')

    # ── Cancel ────────────────────────────────────────────────────────────
    def cancel_callback(self, msg: Bool):
        if msg.data:
            self._cancel_requested = True
            self._cleaning_queue.clear()
            self.get_logger().warn('🚫 Cleaning cancelled')
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()
            self._save_trajectory(self._current_room or 'unknown', aborted=True)
            self._is_cleaning = False
            self.publish_status('CANCELLED')

    # ── Clean request ─────────────────────────────────────────────────────
    def clean_callback(self, msg: String):
        raw = msg.data.strip().lower()

        if self._emergency_stop:
            self.get_logger().warn('Cannot clean — emergency stop active.')
            return

        if raw == 'all':
            rooms = list(ROOMS.keys())
        else:
            rooms = [r.strip() for r in raw.split(',') if r.strip()]

        for room in rooms:
            if room not in ROOMS:
                self.get_logger().error(
                    f'Unknown room: "{room}". Valid: {list(ROOMS.keys())} or "all"'
                )
                return

        for room in rooms:
            self._cleaning_queue.append(room)
            self.get_logger().info(f'📋 Queued room: {room}')

        self._publish_queue()

        if not self._is_cleaning:
            self._process_next_room()

    # ── Queue processing ──────────────────────────────────────────────────
    def _process_next_room(self):
        if not self._cleaning_queue:
            self.get_logger().info('✅ All rooms cleaned. Returning to charging station.')
            self.publish_status('RETURNING_TO_BASE')
            self._navigate_to_charging_station()
            return

        if self._emergency_stop or self._cancel_requested:
            self._cancel_requested = False
            return

        if self._battery_level <= BATTERY_LOW_THRESHOLD:
            self.get_logger().warn('⚠️  Low battery — returning to charge first')
            self._cleaning_queue.appendleft('__charge__')

        next_room = self._cleaning_queue.popleft()
        self._publish_queue()

        if next_room == '__charge__':
            self._navigate_to_charging_station()
            return

        self._start_cleaning_room(next_room)

    def _start_cleaning_room(self, room_name: str):
        self._current_room     = room_name
        self._is_cleaning      = True
        self._cancel_requested = False
        self._waypoint_index   = 0
        self._trajectory_log   = []

        path = generate_coverage_path(ROOMS[room_name])
        self._total_waypoints = len(path)

        self.get_logger().info(
            f'🧹 Starting to clean: {room_name} ({self._total_waypoints} waypoints)'
        )
        self.publish_status(f'CLEANING:{room_name}')

        room_msg = String()
        room_msg.data = room_name
        self.room_pub.publish(room_msg)

        self._navigate_to_waypoint(path[0], 0, self._total_waypoints)

    # ── Waypoint navigation ───────────────────────────────────────────────
    def _navigate_to_waypoint(self, waypoint, index: int, total: int):
        if self._emergency_stop or self._cancel_requested:
            return

        x, y, yaw = waypoint

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._trajectory_log.append({
            'timestamp': datetime.now().strftime('%H:%M:%S.%f'),
            'room':      self._current_room,
            'waypoint':  index,
            'x':         x,
            'y':         y,
            'yaw':       yaw,
        })

        progress = f'{index + 1}/{total}'
        prog_msg = String()
        prog_msg.data = f'{self._current_room}:{progress}:({x:.1f},{y:.1f})'
        self.progress_pub.publish(prog_msg)

        self.get_logger().info(
            f'  Waypoint {progress}: ({x:.2f}, {y:.2f})',
            throttle_duration_sec=3.0
        )

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available!')
            self._is_cleaning = False
            self.publish_status('ERROR:NAV2_UNAVAILABLE')
            return

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._waypoint_response_cb(f, waypoint, index, total)
        )

    def _waypoint_response_cb(self, future, waypoint, index, total):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Waypoint {index} rejected — skipping')
            self._advance_waypoint(index, total)
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._waypoint_result_cb(f, waypoint, index, total)
        )

    def _waypoint_result_cb(self, future, waypoint, index, total):
        self._current_goal_handle = None

        if self._emergency_stop or self._cancel_requested:
            return

        if self._obstacle_paused:
            # Will be resumed by scan_callback when obstacle clears
            return

        # Succeeded, cancelled, or failed — always advance to next waypoint.
        # This prevents infinite retry loops on unreachable waypoints.
        self._advance_waypoint(index, total)

    def _advance_waypoint(self, current_index: int, total: int):
        next_index = current_index + 1

        if next_index >= total:
            self.get_logger().info(f'✅ Finished cleaning: {self._current_room}')
            self.publish_status(f'ROOM_DONE:{self._current_room}')

            timestamp = datetime.now().strftime('%H:%M:%S')
            entry = f'[{timestamp}] ✅ {self._current_room} cleaned'
            self._cleaning_history.append(entry)
            hist_msg = String()
            hist_msg.data = ' | '.join(self._cleaning_history[-10:])
            self.history_pub.publish(hist_msg)

            self._save_trajectory(self._current_room)

            self._battery_level = max(
                0.0, self._battery_level - BATTERY_DRAIN_PER_ROOM
            )

            self._is_cleaning = False
            self._process_next_room()
        else:
            self._waypoint_index = next_index
            path = generate_coverage_path(ROOMS[self._current_room])
            self._navigate_to_waypoint(path[next_index], next_index, total)

    # ── Charging station ──────────────────────────────────────────────────
    def _navigate_to_charging_station(self):
        x, y, yaw = CHARGING_STATION
        self.get_logger().info('🔌 Navigating to charging station')
        self.publish_status('GOING_TO_CHARGE')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available for charging!')
            return

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._charging_response_cb)

    def _charging_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Charging station goal rejected!')
            return
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._charging_result_cb)

    def _charging_result_cb(self, future):
        self._current_goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._battery_level = BATTERY_FULL
            self.get_logger().info('🔋 Charging complete!')
            self.publish_status('CHARGING_COMPLETE')
            if self._cleaning_queue:
                self._process_next_room()
            else:
                self.publish_status('IDLE')
        else:
            self.get_logger().error('Failed to reach charging station')
            self.publish_status('ERROR:CHARGING_FAILED')

    # ── Trajectory saving ─────────────────────────────────────────────────
    def _save_trajectory(self, room_name: str, aborted: bool = False):
        if not self._trajectory_log:
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        suffix = '_ABORTED' if aborted else ''
        base = os.path.join(TRAJECTORY_DIR, f'{room_name}_{timestamp}{suffix}')

        csv_path = base + '.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(
                f, fieldnames=['timestamp', 'room', 'waypoint', 'x', 'y', 'yaw']
            )
            writer.writeheader()
            writer.writerows(self._trajectory_log)

        json_path = base + '.json'
        metadata = {
            'room':            room_name,
            'date':            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'total_waypoints': len(self._trajectory_log),
            'aborted':         aborted,
            'battery_after':   self._battery_level,
            'trajectory':      self._trajectory_log,
        }
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        self.get_logger().info(
            f'💾 Trajectory saved:\n'
            f'   CSV:  {csv_path}\n'
            f'   JSON: {json_path}'
        )
        self._trajectory_log = []

    def _autosave_trajectory(self):
        if self._trajectory_log and self._current_room:
            self._save_trajectory(self._current_room + '_autosave')

    # ── Battery ───────────────────────────────────────────────────────────
    def _publish_battery(self):
        msg = Float32()
        msg.data = self._battery_level
        self.battery_pub.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────
    def _publish_queue(self):
        msg = String()
        msg.data = ','.join(self._cleaning_queue) if self._cleaning_queue else 'EMPTY'
        self.queue_pub.publish(msg)

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CleaningNode()
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