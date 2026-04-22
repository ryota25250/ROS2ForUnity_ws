#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

PARAMS_FILE = '/home/morioka/ROS2ForUnity_ws/config/nav2_multi.yaml'

def _pick_or_fallback_bt_xml():
    bt_dir = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    primary = os.path.join(bt_dir, 'navigate_to_pose.xml')
    repl   = os.path.join(bt_dir, 'navigate_to_pose_w_replanning.xml')
    if os.path.exists(primary):
        return primary
    if os.path.exists(repl):
        return repl
    tmp_path = '/tmp/nav2_min_bt.xml'
    minimal_xml = """<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="simple_nav_to_pose">
      <ComputePathToPose goal="{goal}" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>
"""
    with open(tmp_path, 'w', encoding='utf-8') as f:
        f.write(minimal_xml)
    return tmp_path

def generate_launch_description():
    ns_arg        = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_arg   = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    start_arg     = DeclareLaunchArgument('start_bt', default_value='true')

    ns        = LaunchConfiguration('ns')
    use_sim   = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start     = LaunchConfiguration('start_bt')

    base_link = [ns, TextSubstitution(text='/base_link')]
    safe_bt   = _pick_or_fallback_bt_xml()

    param_rewrites = {
        'bt_navigator.ros__parameters.global_frame': 'map',
        'bt_navigator.ros__parameters.robot_base_frame': base_link,
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': safe_bt,
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': safe_bt,
        'bt_navigator.ros__parameters.server_timeout': '5000',
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_rewrites,
        convert_types=True
    )

    btnav = LifecycleNode(
        condition=IfCondition(start),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {
            'use_sim_time': use_sim,
            'global_frame': 'map',
            'robot_base_frame': base_link,
            'default_nav_to_pose_bt_xml': safe_bt,
            'default_nav_through_poses_bt_xml': safe_bt,
            'server_timeout': 5000,
        }],
    )

    lm = Node(
        condition=IfCondition(start),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_bt',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['bt_navigator'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([ns_arg, use_sim_arg, autostart_arg, start_arg, btnav, lm])
