#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    ns_arg        = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_arg   = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    start_arg     = DeclareLaunchArgument('start_amcl', default_value='true')

    ns        = LaunchConfiguration('ns')
    use_sim   = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start     = LaunchConfiguration('start_amcl')

    base_link = [ns, TextSubstitution(text='/base_link')]
    odom      = [ns, TextSubstitution(text='/odom')]

    amcl = LifecycleNode(
        condition=IfCondition(start),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'base_frame_id': base_link,
            'odom_frame_id': odom,
            'global_frame_id': 'map',
            'scan_topic': 'scan',   # <ns>/scan
            'map_topic': '/map',
        }],
        remappings=[('map', '/map'), ('map_metadata', '/map_metadata')],
    )

    lm = Node(
        condition=IfCondition(start),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([ns_arg, use_sim_arg, autostart_arg, start_arg, amcl, lm])