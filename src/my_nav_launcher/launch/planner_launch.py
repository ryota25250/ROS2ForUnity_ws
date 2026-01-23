#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml

PARAMS_FILE = '/home/moriokalab/ROS2ForUnity_ws/config/nav2_multi.yaml'

def generate_launch_description():
    ns_arg        = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_arg   = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    start_arg     = DeclareLaunchArgument('start_planner', default_value='true')

    ns        = LaunchConfiguration('ns')
    use_sim   = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start     = LaunchConfiguration('start_planner')

    base_link = [ns, TextSubstitution(text='/base_link')]

    param_rewrites = {
        # global_costmap（planner_server内で生成されるノード）の上書き
        'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': base_link,
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': '/map',
        'global_costmap.global_costmap.ros__parameters.static_layer.subscribe_to_updates': 'true',
        'global_costmap.global_costmap.ros__parameters.static_layer.map_subscribe_transient_local': 'true',
        'global_costmap.global_costmap.ros__parameters.transform_tolerance': '0.3',
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_rewrites,
        convert_types=True
    )

    planner = LifecycleNode(
        condition=IfCondition(start),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim}],
        remappings=[('map', '/map'), ('map_metadata', '/map_metadata')],
    )

    lm = Node(
        condition=IfCondition(start),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['planner_server'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([ns_arg, use_sim_arg, autostart_arg, start_arg, planner, lm])
