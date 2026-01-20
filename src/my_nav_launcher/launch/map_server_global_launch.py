#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            description='Full path to the map YAML (image, resolution, origin, etc.)'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # nav2_map_server を1つだけ（グローバル /map を配信）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,   # ここに地図YAMLを渡す
                'topic_name': '/map',        # グローバル /map に統一
                'frame_id': 'map'
            }]
        ),

        # ライフサイクル管理（map_server だけ管理）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
    ])
