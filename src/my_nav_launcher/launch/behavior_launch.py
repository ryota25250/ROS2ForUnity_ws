#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml

PARAMS_FILE = '/home/morioka/ROS2ForUnity_ws/config/nav2_multi.yaml'

def generate_launch_description():
    ns_arg        = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_arg   = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    start_arg     = DeclareLaunchArgument('start_behavior', default_value='true')

    ns        = LaunchConfiguration('ns')
    use_sim   = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start     = LaunchConfiguration('start_behavior')

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites={},
        convert_types=True
    )

    behavior = LifecycleNode(
        condition=IfCondition(start),
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim}],
    )

    lm = Node(
        condition=IfCondition(start),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_behavior',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['behavior_server'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([ns_arg, use_sim_arg, autostart_arg, start_arg, behavior, lm])
