#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


import math
def generate_launch_description():
    # RPLIDAR parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    rplidar_frame_id = LaunchConfiguration('frame_id', default='back_lidar_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Boost')

    # RPLIDAR parameters
    declare_channel_type = DeclareLaunchArgument(
        'channel_type',
        default_value=channel_type,
        description='Specifying channel type of lidar')
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar')

    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar')
    
    declare_rplidar_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value=rplidar_frame_id,
        description='Specifying frame_id of lidar')

    declare_inverted = DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data')

    declare_angle_compensate = DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data')
        
    declare_scan_mode = DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar')
    
    # Define SLLIDAR node
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port, 
            'serial_baudrate': serial_baudrate, 
            'frame_id': rplidar_frame_id,
            'inverted': inverted, 
            'angle_compensate': angle_compensate
        }],
        remappings=[
            ('/scan', '/b_scan')  # Remap /scan to /b_scan
        ],
        output='screen')

    # Define Sick Lidar node
    # ** Note : Sick lidar parameters are hardcoded in the sick_scan_xd package
    sick_scan_node = Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_generic_caller',
            output='screen',
            # Use separate arguments instead of string concatenation
            arguments=[
                './src/sick_scan_xd/launch/sick_tim_7xxS.launch',
                'hostname:=192.168.1.191',
                'frame_id:=front_lidar_link',
                'tf_base_frame_id:=chassis_link',
                'tf_base_lidar_xyz_rpy:=0.465,0.0,-0.2,3.14,0.0,0.0',
            ],
            remappings=[
                ('/scan', '/f_scan')
            ]
        )

    
    # Range Filter for Front Lidar
    f_scan_filtter = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_controller"),
                    "config", "lidar_range_fillter_sick.yaml",
                ])],
            remappings=[
                ('scan', 'f_scan'),
                ('scan_filtered', 'f_scan_filtered')
            ],
        )

    # Range Filter for Back Lidar
    b_scan_filtter = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_controller"),
                    "config", "lidar_range_fillter_a1.yaml",
                ])],
            remappings=[
                ('scan', 'b_scan'),
                ('scan_filtered', 'b_scan_filtered')
            ],
        )
    
    # Dual Laser Merger Node
    dual_laser_merger_node = ComposableNodeContainer(
        name='merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'laser_1_topic': '/f_scan_filtered'},
                    {'laser_2_topic': '/b_scan_filtered'},
                    {'merged_topic': '/scan'},
                    {'target_frame': 'base_footprint'},
                    # {'laser_1_x_offset': 0.465},
                    # {'laser_1_y_offset': 0.0},
                    # {'laser_1_yaw_offset': 3.14159},
                    # {'laser_2_x_offset': -0.465},
                    # {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': -1.57079632679},
                    {'publish_rate': 133},
                    {'tolerance': 0.15},
                    {'queue_size': 5},
                    {'angle_increment': 0.001},
                    {'scan_time': 0.067},
                    {'range_min': 0.275},
                    {'range_max': 25.0},
                    {'min_height': -1.0},
                    {'max_height': 1.0},
                    {'angle_min': -3.141592654},
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': False},
                    {'allowed_radius': 0.45},
                    {'enable_shadow_filter': True},
                    {'enable_average_filter': True},
                ],
                remappings=[
                    ('/merged', '/scan')  # Remapping /merged to /scan
                ],
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_channel_type,
        declare_serial_port,
        declare_serial_baudrate,
        declare_rplidar_frame_id,
        declare_inverted,
        declare_angle_compensate,
        declare_scan_mode,
        sllidar_node,
        sick_scan_node,
        dual_laser_merger_node,
        b_scan_filtter,
        f_scan_filtter,
    ])