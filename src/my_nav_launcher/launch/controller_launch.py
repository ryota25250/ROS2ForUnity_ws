#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    start_arg     = DeclareLaunchArgument('start_controller', default_value='true')

    ns        = LaunchConfiguration('ns')
    use_sim   = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start     = LaunchConfiguration('start_controller')

    base_link = [ns, TextSubstitution(text='/base_link')]
    odom      = [ns, TextSubstitution(text='/odom')]

    param_rewrites = {
        # controller / local_costmap の上書き
        'controller_server.ros__parameters.odom_topic': [ns, TextSubstitution(text='/odom')],
        'local_costmap.local_costmap.ros__parameters.global_frame': odom,
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': base_link,
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': [ns, TextSubstitution(text='/scan')],
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.sensor_frame': [ns, TextSubstitution(text='/base_scan')],
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_rewrites,
        convert_types=True
    )

    controller = LifecycleNode(
        condition=IfCondition(start),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim}],
    )

    lm = Node(
        condition=IfCondition(start),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['controller_server'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([ns_arg, use_sim_arg, autostart_arg, start_arg, controller, lm])
