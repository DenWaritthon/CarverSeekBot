#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

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
                    {'laser_1_topic': '/f_scan'},
                    {'laser_2_topic': '/b_scan'},
                    {'merged_topic': '/scan'},
                    {'target_frame': 'base_footprint'},
                    # {'laser_1_x_offset': 0.465},
                    # {'laser_1_y_offset': 0.0},
                    # {'laser_1_yaw_offset': 3.14159},
                    # {'laser_2_x_offset': -0.465},
                    # {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': -1.57079632679},
                    {'tolerance': 0.01},
                    {'queue_size': 5},
                    {'angle_increment': 0.001},
                    {'scan_time': 0.067},
                    {'range_min': 0.01},
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
            )
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=[
            '-d',
            f"{get_package_share_directory('dual_laser_merger')}/config/rviz_config.rviz",
        ],
    )

    # Add all nodes to the launch description
    # ld.add_action(front_lidar_transform)
    # ld.add_action(back_lidar_transform)
    ld.add_action(dual_laser_merger_node)
    ld.add_action(rviz_node)  # Comment this line if you don't want RViz to start automatically

    return ld