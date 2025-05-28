#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():

    package_name = "face_recognition"
    
    rviz_file_name = "rviz_config.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )

    camera = Node(
    	package=package_name,
    	executable=f"camera.py"
    )

    face_recog = Node(
    	package=package_name,
    	executable=f"face_recog.py"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(rviz)
    launch_description.add_action(camera)
    launch_description.add_action(face_recog)
    return launch_description
