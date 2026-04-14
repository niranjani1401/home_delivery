#!/usr/bin/env python3
"""
Autonomous Delivery Node
Navigates TurtleBot3 to predefined delivery locations in sequence.
Supports: kitchen, bedroom, living_room, charging_station

Novelty Features:
  - Multi-stop delivery queue (comma-separated locations)
  - Auto return-to-base after each delivery
  - Delivery history with timestamps
  - Cancel current delivery support
  - Low-battery simulation with auto-return
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float32
from action_msgs.msg import GoalStatus
import math
import time
from collections import deque
from datetime import datetime


# ── Predefined delivery locations (x, y, yaw_radians) ──────────────────────
# Coordinates placed in open floor space, away from walls and furniture.
# Kitchen: right side, above kitchen wall (y=1), left of table (3,3)
# Living room: left-bottom area, away from couch (-3,-3) and bookshelf (-4.5,-1)
# Bedroom: left-top area, away from bed (-3, 3.5)
# Charging station: safe open area in bottom-right quadrant
DELIVERY_LOCATIONS = {
    'kitchen':          (2.0,  2.0,  0.0),   # open area near doorway, clear of table/chairs
    'living_room':      (-2.0, -2.5, 1.57),  # left-bottom, clear of couch
    'bedroom':          (-2.0,  2.5, 3.14),  # left room, near doorway gap
    'charging_station': (2.0,  -2.0, 0.0),   # right-bottom, open space
}

# Battery simulation parameters
BATTERY_FULL = 100.0
BATTERY_DRAIN_PER_DELIVERY = 15.0   # % lost per delivery trip
BATTERY_LOW_THRESHOLD = 25.0        # auto-return when below this
BATTERY_CHARGE_RATE = 5.0           # % gained per second at charging station


class DeliveryNode(Node):

    def __init__(self):
        super().__init__('delivery_node')
        # Note: use_sim_time is auto-declared in Jazzy — no need to declare it

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(String, '/delivery_request',  self.delivery_callback, 10)
        self.create_subscription(Bool,   '/emergency_stop',    self.estop_callback,    10)
        self.create_subscription(Bool,   '/cancel_delivery',   self.cancel_callback,   10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.status_pub   = self.create_publisher(String,  '/delivery_status',   10)
        self.current_pub  = self.create_publisher(String,  '/current_location',  10)
        self.history_pub  = self.create_publisher(String,  '/delivery_history',  10)
        self.battery_pub  = self.create_publisher(Float32, '/battery_level',     10)
        self.queue_pub    = self.create_publisher(String,  '/delivery_queue',    10)

        # ── State ────────────────────────────────────────────────────────────
        self._emergency_stop = False
        self._is_delivering  = False
        self._current_goal_handle = None
        self._delivery_queue = deque()
        self._delivery_history = []
        self._battery_level = BATTERY_FULL
        self._auto_return = True  # Return to charging station after delivery
        self._interrupted_location = None  # Saved when emergency stop fires

        # Battery publish timer (every 2 seconds)
        self.create_timer(2.0, self._publish_battery)

        self.get_logger().info(
            '✅ Delivery Node started (Enhanced)\n'
            '   Topics:\n'
            '     /delivery_request  — send location name or comma-separated list\n'
            '     /cancel_delivery   — cancel current delivery (Bool: true)\n'
            '     /delivery_status   — current status\n'
            '     /delivery_history  — completed deliveries\n'
            '     /battery_level     — simulated battery %\n'
            '     /delivery_queue    — pending stops'
        )
        self.publish_status('IDLE')

    # ── Battery Simulation ───────────────────────────────────────────────────
    def _publish_battery(self):
        msg = Float32()
        msg.data = self._battery_level
        self.battery_pub.publish(msg)

    def _drain_battery(self):
        self._battery_level = max(0.0, self._battery_level - BATTERY_DRAIN_PER_DELIVERY)
        self.get_logger().info(f'🔋 Battery: {self._battery_level:.1f}%')
        if self._battery_level <= BATTERY_LOW_THRESHOLD and not self._is_delivering:
            self.get_logger().warn(
                f'⚠️  LOW BATTERY ({self._battery_level:.1f}%) — auto-returning to charging station'
            )
            self._delivery_queue.clear()
            self._delivery_queue.append('charging_station')
            self._process_next_in_queue()

    def _charge_battery(self):
        """Simulate charging when at charging station."""
        self._battery_level = BATTERY_FULL
        self.get_logger().info(f'🔌 Battery charged to {self._battery_level:.1f}%')

    # ── Emergency Stop ───────────────────────────────────────────────────────
    def estop_callback(self, msg: Bool):
        if msg.data:
            self._emergency_stop = True
            self.get_logger().warn('🛑 EMERGENCY STOP ACTIVATED')
            self.publish_status('EMERGENCY_STOP')
            # Cancel any active navigation (location saved in result_callback)
            if self._current_goal_handle is not None:
                self.get_logger().info('Cancelling active navigation goal...')
                self._current_goal_handle.cancel_goal_async()
        else:
            self._emergency_stop = False
            self.get_logger().info('✅ Emergency stop cleared')
            # Resume interrupted delivery if there was one
            if self._interrupted_location:
                loc = self._interrupted_location
                self._interrupted_location = None
                self.get_logger().info(f'🔄 Resuming delivery to {loc}')
                self.publish_status(f'RESUMING:{loc}')
                self._delivery_queue.appendleft(loc)
                self._process_next_in_queue()
            else:
                self.publish_status('IDLE')

    # ── Cancel Delivery ──────────────────────────────────────────────────────
    def cancel_callback(self, msg: Bool):
        if msg.data and self._is_delivering:
            self.get_logger().warn('🚫 Cancel delivery requested')
            self._delivery_queue.clear()
            self._interrupted_location = None  # Don't allow resume after cancel
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
            self._is_delivering = False
            self.publish_status('CANCELLED')
            self._publish_queue()

    # ── Delivery Request ─────────────────────────────────────────────────────
    def delivery_callback(self, msg: String):
        raw = msg.data.strip().lower()

        if self._emergency_stop:
            self.get_logger().warn('Cannot deliver — emergency stop is active.')
            return

        # ── Parse multi-stop requests (comma-separated) ─────────────────────
        locations = [loc.strip() for loc in raw.split(',') if loc.strip()]

        # Validate all locations first
        for loc in locations:
            if loc not in DELIVERY_LOCATIONS:
                self.get_logger().error(
                    f'Unknown location: "{loc}". '
                    f'Valid: {list(DELIVERY_LOCATIONS.keys())}'
                )
                return

        # Add to queue
        for loc in locations:
            self._delivery_queue.append(loc)
            self.get_logger().info(f'📋 Queued: {loc}')

        self._publish_queue()

        # If not already delivering, start processing the queue
        if not self._is_delivering:
            self._process_next_in_queue()

    # ── Queue Processing ─────────────────────────────────────────────────────
    def _process_next_in_queue(self):
        if not self._delivery_queue:
            # Queue exhausted — optionally return to base
            if self._auto_return and not self._is_delivering:
                self.get_logger().info('📍 Queue complete.')
                self.publish_status('IDLE')
            return

        if self._emergency_stop:
            self.get_logger().warn('Queue paused — emergency stop active.')
            return

        # Check battery before starting next delivery
        if (self._battery_level <= BATTERY_LOW_THRESHOLD
                and self._delivery_queue[0] != 'charging_station'):
            self.get_logger().warn(
                f'⚠️  LOW BATTERY ({self._battery_level:.1f}%) — '
                f'rerouting to charging station before next delivery'
            )
            self._delivery_queue.appendleft('charging_station')

        next_location = self._delivery_queue.popleft()
        self._publish_queue()
        self._is_delivering = True
        self.navigate_to(next_location)

    def _publish_queue(self):
        msg = String()
        msg.data = ','.join(self._delivery_queue) if self._delivery_queue else 'EMPTY'
        self.queue_pub.publish(msg)

    # ── Navigation ───────────────────────────────────────────────────────────
    def navigate_to(self, location_name: str):
        x, y, yaw = DELIVERY_LOCATIONS[location_name]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (rotation around Z)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.publish_status(f'NAVIGATING_TO:{location_name}')
        self.get_logger().info(f'🚀 Navigating to {location_name} ({x}, {y})')

        # Wait for action server
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            self._is_delivering = False
            self.publish_status('ERROR:NAV2_NOT_AVAILABLE')
            return

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda f: self.goal_response_callback(f, location_name)
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        self.get_logger().info(
            f'  Distance remaining: {dist:.2f}m',
            throttle_duration_sec=2.0
        )

    def goal_response_callback(self, future, location_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            self._is_delivering = False
            self.publish_status('ERROR:GOAL_REJECTED')
            # Try next in queue
            self._process_next_in_queue()
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.result_callback(f, location_name)
        )

    def result_callback(self, future, location_name):
        result = future.result()
        status = result.status
        self._current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ Successfully delivered to {location_name}!')
            self.publish_status(f'DELIVERED:{location_name}')

            # Announce current location
            loc_msg = String()
            loc_msg.data = location_name
            self.current_pub.publish(loc_msg)

            # Record in delivery history
            timestamp = datetime.now().strftime('%H:%M:%S')
            entry = f'[{timestamp}] ✅ {location_name}'
            self._delivery_history.append(entry)
            hist_msg = String()
            hist_msg.data = ' | '.join(self._delivery_history[-10:])  # last 10
            self.history_pub.publish(hist_msg)

            # Drain battery
            self._drain_battery()

            # If arrived at charging station, charge up
            if location_name == 'charging_station':
                self._charge_battery()

            # Auto-return: if queue is empty and we're not at charging station
            if (self._auto_return
                    and not self._delivery_queue
                    and location_name != 'charging_station'):
                self.get_logger().info('🏠 Auto-returning to charging station')
                self._delivery_queue.append('charging_station')

        elif self._emergency_stop:
            self.get_logger().warn('Navigation cancelled due to emergency stop.')
            self.publish_status('STOPPED')
            # Save interrupted location for resume
            self._interrupted_location = location_name
            self.get_logger().info(f'💾 Saved "{location_name}" for resume')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Navigation to {location_name} was cancelled.')
            self.publish_status(f'CANCELLED:{location_name}')
        else:
            self.get_logger().error(
                f'Navigation failed to {location_name}. Status: {status}'
            )
            self.publish_status(f'FAILED:{location_name}')

            # Record failure in history
            timestamp = datetime.now().strftime('%H:%M:%S')
            entry = f'[{timestamp}] ❌ {location_name} (FAILED)'
            self._delivery_history.append(entry)

        self._is_delivering = False

        # Process next delivery in queue
        self._process_next_in_queue()

    # ── Helpers ──────────────────────────────────────────────────────────────
    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryNode()
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
