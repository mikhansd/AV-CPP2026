from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map = PathJoinSubstitution([
        FindPackageShare('vehicle_core'), 'maps', 'av_cpp_test_map.yaml'
    ])

    map_file = LaunchConfiguration('map_file')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': False,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('vehicle_core'), 'cfg', 'amcl.yaml'
            ])
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description=(
                'Occupancy-grid YAML. Replace the test default with the real '
                'map YAML when available.'),
        ),
        map_server,
        amcl,
        lifecycle_manager,
    ])

