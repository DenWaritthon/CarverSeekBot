#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Define LaunchConfiguration objects
    joy_type = LaunchConfiguration('joy_type', default='xbox')
    robot_navigation_pkg = get_package_share_directory('robot_navigation')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')

    # Create launch argument declarations
    declare_joy_type = DeclareLaunchArgument(
        'joy_type',
        default_value=joy_type,
        description='You type you using')
    
    # DiffDrive node
    diffdrive_node = Node(
        package='robot_controller',
        executable='diffdrive_controller.py',
        name='diff_drive_controller',
        output='screen',
    )

    #Teleop joy node
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('teleop_twist_joy') + '/launch/teleop-launch.py'
        ),
        launch_arguments={'joy_config': joy_type}.items()
    )

    # Robot description launch
    robot_decription_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('robot_description') + '/launch/carver_cap_description.launch.py'
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    #Modbus bridge node
    modbus_bridge_node = Node(
        package='modbus_bridge',
        executable='modbus_bridge.py',
        name='modbus_bridge',
        output='screen'
    )

    # Read sensor node
    read_sensor_node = Node(
        package='robot_controller',
        executable='read_sensor.py',
        name='read_sensor',
        output='screen'
    )

    # Lidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('robot_bringup') + '/launch/lidar.launch.py'
        )    
    )

    #EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(robot_navigation_pkg, 'config', 'ekf.yaml')
            # {'use_sim_time': 'false'},
             ],
        # remappings=[
        #     ('/odometry/filtered', '/odom')
        # ]
    )

    #IMU publisher node
    imu_puiblisher_node = Node(
        package='robot_controller',
        executable='imu_publisher.py',
        name='imu_publisher',
        output='screen'
    )
    

    #Face matching navigation node
    face_matching_navigation_node = Node(
        package='face_recognition',
        executable='face_matcher.py',
        name='face_matching_navigation_node',
        output='screen'
    )

    # Cancel navigation node
    cancel_navigation_node = Node(
        package='robot_navigation',
        executable='cancel_navigation_service.py',
        name='cancel_navigation_service',
        output='screen'
    )

    return LaunchDescription([
        declare_joy_type,
        teleop_launch,
        diffdrive_node,
        modbus_bridge_node,
        read_sensor_node,
        robot_decription_launch,
        lidar_launch,
        ekf_node,
        imu_puiblisher_node,
        face_matching_navigation_node,
        cancel_navigation_node
    ])