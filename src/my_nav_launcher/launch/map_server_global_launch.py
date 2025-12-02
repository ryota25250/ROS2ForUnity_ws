#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    # ==== Launch Arguments ====
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/moriokalab/ROS2ForUnity_ws/my_unity_map_2.yaml',
        description='Map YAML for nav2_map_server'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==== Global nav2_map_server (LifecycleNode) ====
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',                 # ← グローバルで起動（必須）
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
            'frame_id': 'map',        # ← header.frame_id をグローバル "map" に統一
        }],
        # Humbleではパラメータ topic_name より remappings の方が確実
        remappings=[
            ('map', '/map'),
            ('map_metadata', '/map_metadata'),
        ],
    )

    # ==== Lifecycle manager (自動で configure→activate) ====
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace='',                 # ← グローバル
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ==== Static TF: /map → /robot1/map, /map → /robot2/map ====
    # 恒等変換 (x y z qx qy qz qw parent child)
    static_map_to_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot1_map',
        namespace='',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'robot1/map'],
        output='screen',
    )

    static_map_to_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot2_map',
        namespace='',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'robot2/map'],
        output='screen',
    )

    return LaunchDescription([
        map_yaml_arg, use_sim_time_arg,
        map_server, lifecycle_manager,
        static_map_to_robot1, static_map_to_robot2,
    ])
