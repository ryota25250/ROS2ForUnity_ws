#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    # args
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/moriokalab/ROS2ForUnity_ws/my_unity_map_2.yaml',
        description='Map YAML for nav2_map_server'
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # map_server (global)
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
            'frame_id': 'map',
        }],
        # /map を絶対名に固定
        remappings=[('map', '/map'), ('map_metadata', '/map_metadata')],
    )

    # lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    return LaunchDescription([
        map_yaml_arg, use_sim_time_arg,
        map_server, lifecycle_manager
    ])
