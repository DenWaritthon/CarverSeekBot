#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RPLIDAR parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    rplidar_frame_id = LaunchConfiguration('frame_id', default='back_lidar')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Boost')

    # Sick Lidar parameters
    hostname = LaunchConfiguration('hostname', default='192.168.1.191')
    sick_frame_id = LaunchConfiguration('sick_frame_id', default='front_lidar')

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
                'frame_id:=front_lidar'
            ],
            remappings=[
                ('/scan', '/f_scan')
            ]
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
        sick_scan_node
    ])