#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ns = LaunchConfiguration('ns')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base_frame = [ns, TextSubstitution(text='/base_link')]
    odom_frame = [ns, TextSubstitution(text='/odom')]
    # map_frame  = [ns, TextSubstitution(text='/map')]
    map_frame  = 'map'

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': base_frame,    # robotX/base_link
            'odom_frame_id': odom_frame,    # robotX/odom
            'global_frame_id': map_frame,     # 例: robot1/map
            'scan_topic': 'scan',
        }],
        remappings=[('map','/map'), ('map_metadata','/map_metadata')]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']  # ★ amcl のみ管理
        }]
    )

    return LaunchDescription([ns_arg, use_sim_time_arg, amcl, lifecycle_manager_localization])
