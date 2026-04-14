"""
Delivery Launch File — Standalone
Launches only the delivery and emergency stop nodes.
Use when Gazebo + Nav2 are already running separately.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    delivery_node = Node(
        package='tb3_delivery',
        executable='delivery_node',
        name='delivery_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    estop_node = Node(
        package='tb3_delivery',
        executable='emergency_stop_node',
        name='emergency_stop_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([delivery_node, estop_node])
