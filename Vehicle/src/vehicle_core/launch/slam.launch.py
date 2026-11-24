"""
Start order: run vehicle_core bringup first (ekf_publish_tf:=false), then this.

    ros2 launch vehicle_core bringup.launch.py ekf_publish_tf:=false
    ros2 launch vehicle_core navigation.launch.py \
        nav2_params:=/path/to/nav2_params.yaml \
        slam_params:=/path/to/slam_params.yaml

The bridge is OFF by default. Nav2 and slam_toolbox start and you can set goals
and watch /cmd_vel, but nothing reaches the vehicle until you add
enable_bridge:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join as path_join


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')

    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')
    enable_bridge = LaunchConfiguration('enable_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('vehicle_core')
    default_nav2 = path_join(pkg_share, 'cfg', 'nav2.yaml')
    default_slam = path_join(pkg_share, 'cfg', 'slam_toolbox.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items())

    # navigation_launch.py = controller, planner, behaviors, bt_navigator,
    # waypoint_follower (+ smoother). No AMCL / map_server, so slam_toolbox
    # owns map -> odom.
    #nav2 = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
    #    launch_arguments={
    #        'params_file': nav2_params,
    #        'use_sim_time': use_sim_time,
    #        'autostart': 'true',
    #    }.items())

    # Change package/executable to wherever you install the bridge.
    #bridge = Node(
    #    package='vehicle_core',
    #    executable='twist_to_internal_drive',
    #    name='twist_to_internal_drive',
    #    output='screen',
    #    condition=IfCondition(enable_bridge),
    #    parameters=[{
    #        'input_topic': '/cmd_vel',
    #        'max_speed_mps': 0.20,
    #        'min_drive_speed_mps': 0.10,
    #        'max_steer_rad': 0.25,   # confirm the real mechanical limit first
    #    }])

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params',
                              default_value=default_nav2,
                              description='Path to nav2_params.yaml'),
        DeclareLaunchArgument('slam_params',
                              default_value=default_slam,
                              description='Path to slam_toolbox params yaml'),
        DeclareLaunchArgument('enable_bridge', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam,
    #    nav2,
    #    bridge,
    ])