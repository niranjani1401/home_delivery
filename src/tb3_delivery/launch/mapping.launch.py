"""
Mapping Launch File
Launches Gazebo with custom apartment world, spawns TurtleBot3,
starts SLAM Toolbox for mapping, and opens RViz for visualization.
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
    pkg_slam_toolbox  = get_package_share_directory('slam_toolbox')

    use_sim_time     = LaunchConfiguration('use_sim_time', default='true')
    world_file       = os.path.join(pkg_tb3_delivery, 'worlds', 'apartment.world')
    slam_params_file = os.path.join(pkg_tb3_delivery, 'config', 'slam_params.yaml')

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

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file,
        }.items()
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_slam_toolbox, 'config', 'slam_toolbox_default.rviz')
    # Fallback if slam_toolbox doesn't ship the rviz config
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(
            get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld = LaunchDescription()
    for action in gazebo_actions:
        ld.add_action(action)
    ld.add_action(spawn_turtlebot)
    ld.add_action(robot_state_publisher)
    ld.add_action(slam)
    ld.add_action(rviz_node)
    return ld
