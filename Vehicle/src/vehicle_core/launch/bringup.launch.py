from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from os.path import join as path_join
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('vehicle_core')
    cfg_sm    = path_join(pkg_share, 'cfg', 'state_manager.yaml')
    cfg_da    = path_join(pkg_share, 'cfg', 'drive_arbiter.yaml')
    cfg_i2c   = path_join(pkg_share, 'cfg', 'i2c_bridge.yaml')
    cfg_cam   = path_join(pkg_share, 'cfg', 'camera_gscam.yaml')
    cfg_gps   = path_join(pkg_share, 'cfg', 'gt_u7_gps.yaml')
    cfg_hdc   = path_join(pkg_share, 'cfg', 'heading_calib.yaml')
    cfg_atp   = path_join(pkg_share, 'cfg', 'autopilot.yaml')
    cfg_odom = path_join(pkg_share,'cfg','ackermann_odometry.yaml')
    cfg_rtsp  = path_join(pkg_share, 'cfg', 'rtsp_raw_stream.yaml')

    ekf_params    = PathJoinSubstitution([pkg_share,  'cfg', 'ekf.yaml'])
    navsat_params = PathJoinSubstitution([pkg_share,  'cfg', 'navsat.yaml'])
    path_file     = PathJoinSubstitution([pkg_share, 'data', 'Vehicle_Path.kml'])

    container = ComposableNodeContainer(
        name='vehicle_core_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::StateManagerNode',
                name='state_manager_node',
                parameters=[cfg_sm],
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::DriveArbiterNode',
                name='drive_arbiter_node',
                parameters=[cfg_da],
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::TcpServerNode',
                name='tcp_server_node'
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::I2cBridgeNode',
                name='i2c_bridge_node',
                parameters=[cfg_i2c],
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::GstCameraNode',
                name='camera',
                parameters=[{
                    "device": "/dev/video11",
                    "width": 2112,
                    "height": 1568,
                    "fps": 15,
                    "topic": "/camera/image_raw",
                    "frame_id": "camera_link",
                    "queue_depth": 5,
                }],
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::RtspStreamerNode',
                name='rtsp_streamer_node',
                parameters=[cfg_rtsp],
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::GtU7GpsNode',
                name='gt_u7_gps_node',
                parameters=[cfg_gps],
            ),
            ComposableNode(
                package='vehicle_core',
                plugin='vehicle_core::HeadingCalibratorNode',
                name='heading_calibrator_node',
            ),
            ComposableNode(
                package="vehicle_core",
                plugin="vehicle_core::AutopilotNode",
                name="autopilot_node",
                parameters=[{"path_file": path_file}, cfg_atp],
            ),
            ComposableNode(
                package="vehicle_core",
                plugin="vehicle_core::AckermannOdometryNode",
                name="ackermann_odometry_node",
                parameters=[cfg_odom],
            ),
        ],
        emulate_tty=True,
    )

    # navsat_transform_node: converts /fix -> /filtered/gps/pose (ENU)
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_params],
        remappings=[
            ('/imu/data', '/vehicle/imu/data'),
            ('/gps/fix',  '/fix'),
            # publishes /filtered/gps/pose by default
        ],
    )

    # ekf_localization_node: fuses IMU wz + /vehicle/velocity + /filtered/gps/pose
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[ekf_params],
    )

    # wvs_params = {
    #     "address": "0.0.0.0",
    #     "port": 8080,
    #     "jpeg_quality": 45,
    #     "queue_size": 1,
    #     "server_threads": 2
    # }
    # web_video = Node(
    #     package="web_video_server",
    #     executable="web_video_server",
    #     name="web_video_server",
    #     parameters=[wvs_params],
    #     output="screen"
    # )

    return LaunchDescription([
        container,
        navsat,
        ekf,
        # imu_tf,  # uncomment if you need the static transform
        #web_video,
    ])
