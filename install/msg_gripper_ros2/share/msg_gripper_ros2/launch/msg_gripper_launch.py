# Copyright (c) 2026 UCLA Biomechatronics Lab — MIT License

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('msg_gripper_ros2')
    default_config = os.path.join(pkg_share, 'config', 'default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the gripper configuration YAML file',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='msg_gripper_node',
            description='Name of the gripper node',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS2 namespace for the node',
        ),
        Node(
            package='msg_gripper_ros2',
            executable='msg_gripper_node',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
