#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('livox_loam'),
        'config',
        'params.yaml'
    )

    livox_loam = Node(
        package='livox_loam',
        namespace='',
        executable='livox_loam_node',
        parameters=[
            config,
        ],
        # remappings=[
        #     ('', ''),
        # ],
        output='screen',
    )
    
    ld.add_action(livox_loam)

    return ld
    