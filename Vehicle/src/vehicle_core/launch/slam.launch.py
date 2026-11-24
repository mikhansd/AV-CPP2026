"""
Start vehicle_core bringup first, then SLAM and Nav2:

    ros2 launch vehicle_core bringup.launch.py ekf_publish_tf:=true
    ros2 launch vehicle_core slam.launch.py

The local EKF (world_frame: odom) supplies odom -> base_link; SLAM Toolbox
supplies map -> odom. Keep the EKF transform enabled for this setup.

The vehicle bridge is off by default. Add enable_bridge:=true to forward
Nav2 commands to the drive arbiter after checking the navigation output.
"""

from os.path import join as path_join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('vehicle_core')
    slam_dir = get_package_share_directory('slam_toolbox')

    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')
    enable_bridge = LaunchConfiguration('enable_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_nav2 = path_join(pkg_share, 'cfg', 'nav2.yaml')
    default_slam = path_join(pkg_share, 'cfg', 'slam_toolbox.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path_join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items())

    # Rewrite the clock for servers and their costmaps. Resolve the behavior
    # trees' find-pkg-share substitutions against the installed package.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    # Match lifecycle_manager_navigation.node_names in nav2.yaml.
    # Direct server launch keeps controller/behavior output on /cmd_vel;
    # this configuration does not use a velocity smoother.
    nav2_nodes = [
        Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[configured_params],
        )
        for package, executable, name in [
            ('nav2_controller', 'controller_server', 'controller_server'),
            ('nav2_planner', 'planner_server', 'planner_server'),
            ('nav2_behaviors', 'behavior_server', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
            ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
            ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation'),
        ]
    ]

    bridge = Node(
        package='vehicle_core',
        executable='twist_to_internal_drive',
        name='twist_to_internal_drive',
        output='screen',
        condition=IfCondition(enable_bridge),
        parameters=[{
            'input_topic': '/cmd_vel',
            'max_speed_mps': 0.20,
            'min_drive_speed_mps': 0.02,
            'max_steer_rad': 0.25,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2_params', default_value=default_nav2,
            description='Path to the Nav2 parameter file'),
        DeclareLaunchArgument(
            'slam_params', default_value=default_slam,
            description='Path to the SLAM Toolbox parameter file'),
        DeclareLaunchArgument('enable_bridge', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam,
        *nav2_nodes,
        bridge,
    ])
