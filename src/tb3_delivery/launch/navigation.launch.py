"""
Navigation Launch File
Launches Gazebo with apartment world, spawns TurtleBot3,
starts Nav2 with saved map, and launches delivery + emergency stop nodes.
Compatible with ROS 2 Jazzy (Gazebo Harmonic) AND Humble (Gazebo Classic).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
USE_NEW_GAZEBO = ROS_DISTRO in ('jazzy', 'iron', 'rolling')


def generate_launch_description():
    pkg_tb3_delivery  = get_package_share_directory('tb3_delivery')
    pkg_tb3_gazebo    = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup  = get_package_share_directory('nav2_bringup')

    use_sim_time  = LaunchConfiguration('use_sim_time', default='true')
    world_file    = os.path.join(pkg_tb3_delivery, 'worlds', 'apartment.world')
    map_file      = os.path.join(pkg_tb3_delivery, 'maps',   'apartment_map.yaml')
    nav2_params   = os.path.join(pkg_tb3_delivery, 'config', 'nav2_params.yaml')
    rviz_config   = os.path.join(pkg_nav2_bringup,  'rviz',  'nav2_default_view.rviz')

    # ── Gazebo: Jazzy (Harmonic) vs Humble (Classic) ─────────────────────────
    if USE_NEW_GAZEBO:
        from launch.actions import AppendEnvironmentVariable
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

        set_env_vars = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_tb3_gazebo, 'models')
        )
        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world_file],
                'on_exit_shutdown': 'true'
            }.items()
        )
        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-g -v2 ',
                'on_exit_shutdown': 'true'
            }.items()
        )
        gazebo_actions = [set_env_vars, gzserver, gzclient]

    else:
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'false'
            }.items()
        )
        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )
        gazebo_actions = [gzserver, gzclient]

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Spawn TurtleBot3 ──────────────────────────────────────────────────────
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '1.0',
            'y_pose': '0.0'
        }.items()
    )

    # ── Nav2 Bringup ──────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          map_file,
            'use_sim_time': 'true',
            'params_file':  nav2_params,
        }.items()
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Delivery Node ─────────────────────────────────────────────────────────
    delivery_node = Node(
        package='tb3_delivery',
        executable='delivery_node',
        name='delivery_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Emergency Stop Node ───────────────────────────────────────────────────
    estop_node = Node(
        package='tb3_delivery',
        executable='emergency_stop_node',
        name='emergency_stop_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()
    for action in gazebo_actions:
        ld.add_action(action)
    ld.add_action(spawn_turtlebot)
    ld.add_action(robot_state_publisher)
    ld.add_action(nav2)
    ld.add_action(rviz_node)
    ld.add_action(delivery_node)
    ld.add_action(estop_node)
    return ld
