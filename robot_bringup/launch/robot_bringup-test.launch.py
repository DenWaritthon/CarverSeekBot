#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define LaunchConfiguration objects
    joy_type = LaunchConfiguration('joy_type', default='xbox')

    # Create launch argument declarations
    declare_joy_type = DeclareLaunchArgument(
        'joy_type',
        default_value=joy_type,
        description='You type you using')
    
    # DiffDrive node
    diffdrive_node = Node(
        package='robot_controller',
        executable='new_diff_drive_controller.py',
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

    return LaunchDescription([
        declare_joy_type,
        teleop_launch,
        diffdrive_node,
        modbus_bridge_node,
        read_sensor_node
    ])