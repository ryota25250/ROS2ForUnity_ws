#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

PARAMS_FILE = '/home/morioka/ROS2ForUnity_ws/config/nav2_multi.yaml'


def _str2bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _robot_index_from_ns(ns: str) -> int:
    match = re.search(r'(\d+)$', ns)
    if not match:
        return 1
    return max(1, int(match.group(1)))


def _pick_or_fallback_bt_xml() -> str:
    bt_dir = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    candidates = [
        os.path.join(bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'),
        os.path.join(bt_dir, 'navigate_to_pose_w_replanning.xml'),
        os.path.join(bt_dir, 'navigate_to_pose.xml'),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path

    tmp_path = '/tmp/nav2_min_bt.xml'
    minimal_xml = """<?xml version=\"1.0\"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="simple_nav_to_pose">
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
"""
    with open(tmp_path, 'w', encoding='utf-8') as f:
        f.write(minimal_xml)
    return tmp_path


def _launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('ns').perform(context)
    use_sim = _str2bool(LaunchConfiguration('use_sim_time').perform(context))
    autostart = _str2bool(LaunchConfiguration('autostart').perform(context))

    # 互換性維持用（既存コマンドを壊さないため残す）
    _ = LaunchConfiguration('map_yaml').perform(context)
    _ = LaunchConfiguration('start_map_server').perform(context)
    _ = LaunchConfiguration('use_composition').perform(context)

    start_amcl = _str2bool(LaunchConfiguration('start_amcl').perform(context))
    start_plan = _str2bool(LaunchConfiguration('start_planner').perform(context))
    start_ctrl = _str2bool(LaunchConfiguration('start_controller').perform(context))
    start_beh = _str2bool(LaunchConfiguration('start_behavior').perform(context))
    start_bt = _str2bool(LaunchConfiguration('start_bt').perform(context))
    set_initial_pose = _str2bool(LaunchConfiguration('set_initial_pose').perform(context))

    startup_spacing = float(LaunchConfiguration('startup_spacing').perform(context))
    per_robot_offset = float(LaunchConfiguration('per_robot_offset').perform(context))
    robot_index = _robot_index_from_ns(ns)
    base_delay = per_robot_offset * float(robot_index - 1)

    base_link = f'{ns}/base_link'
    odom = f'{ns}/odom'
    scan = f'/{ns}/scan'
    safe_bt = _pick_or_fallback_bt_xml()

    # RewrittenYaml には string を渡し、convert_types=True で型変換させる
    param_rewrites = {
        # AMCL
        'amcl.ros__parameters.use_sim_time': 'true' if use_sim else 'false',
        'amcl.ros__parameters.base_frame_id': base_link,
        'amcl.ros__parameters.odom_frame_id': odom,
        'amcl.ros__parameters.global_frame_id': 'map',
        'amcl.ros__parameters.scan_topic': 'scan',
        'amcl.ros__parameters.map_topic': '/map',
        'amcl.ros__parameters.set_initial_pose': 'true' if set_initial_pose else 'false',
        'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_pose_x').perform(context),
        'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_pose_y').perform(context),
        'amcl.ros__parameters.initial_pose.z': '0.0',
        'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_pose_yaw').perform(context),
        'amcl.ros__parameters.transform_tolerance': '0.5',

        # planner / global_costmap
        'planner_server.ros__parameters.expected_planner_frequency': '0.5',
        'global_costmap.global_costmap.ros__parameters.use_sim_time': 'true' if use_sim else 'false',
        'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': base_link,
        'global_costmap.global_costmap.ros__parameters.transform_tolerance': '0.5',
        'global_costmap.global_costmap.ros__parameters.update_frequency': '0.5',
        'global_costmap.global_costmap.ros__parameters.publish_frequency': '0.2',
        'global_costmap.global_costmap.ros__parameters.always_send_full_costmap': 'false',
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': '/map',
        'global_costmap.global_costmap.ros__parameters.static_layer.subscribe_to_updates': 'true',
        'global_costmap.global_costmap.ros__parameters.static_layer.map_subscribe_transient_local': 'true',

        # controller / local_costmap
        'controller_server.ros__parameters.use_sim_time': 'true' if use_sim else 'false',
        'controller_server.ros__parameters.odom_topic': f'/{ns}/odom',
        'controller_server.ros__parameters.controller_frequency': '5.0',
        'controller_server.ros__parameters.failure_tolerance': '0.5',
        'local_costmap.local_costmap.ros__parameters.use_sim_time': 'true' if use_sim else 'false',
        'local_costmap.local_costmap.ros__parameters.global_frame': odom,
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': base_link,
        'local_costmap.local_costmap.ros__parameters.transform_tolerance': '0.5',
        'local_costmap.local_costmap.ros__parameters.update_frequency': '3.0',
        'local_costmap.local_costmap.ros__parameters.publish_frequency': '1.0',
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': scan,
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.sensor_frame': f'{ns}/base_scan',

        # behavior_server
        'behavior_server.ros__parameters.use_sim_time': 'true' if use_sim else 'false',
        'behavior_server.ros__parameters.global_frame': 'map',
        'behavior_server.ros__parameters.local_frame': odom,
        'behavior_server.ros__parameters.robot_base_frame': base_link,
        'behavior_server.ros__parameters.transform_tolerance': '0.5',
        'behavior_server.ros__parameters.cycle_frequency': '5.0',

        # bt_navigator
        'bt_navigator.ros__parameters.use_sim_time': 'true' if use_sim else 'false',
        'bt_navigator.ros__parameters.global_frame': 'map',
        'bt_navigator.ros__parameters.robot_base_frame': base_link,
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': safe_bt,
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': safe_bt,
        'bt_navigator.ros__parameters.server_timeout': '8000',
        'bt_navigator.ros__parameters.filter_duration': '0.2',
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    actions = []

    if start_amcl:
        amcl = LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[rewritten, {'use_sim_time': use_sim}],
            remappings=[('map', '/map'), ('map_metadata', '/map_metadata')],
        )
        lm_loc = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_sim_time': use_sim,
                'autostart': autostart,
                'node_names': ['amcl'],
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 60.0,
            }],
        )
        actions.append(TimerAction(period=base_delay + 0.0, actions=[amcl]))
        actions.append(TimerAction(period=base_delay + 1.5, actions=[lm_loc]))

    nav_nodes = []
    nav_node_names = []
    nav_start_base = base_delay + startup_spacing

    if start_plan:
        planner = LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[rewritten, {'use_sim_time': use_sim}],
            remappings=[('map', '/map'), ('map_metadata', '/map_metadata')],
        )
        nav_nodes.append(TimerAction(period=nav_start_base + 0.0, actions=[planner]))
        nav_node_names.append('planner_server')

    if start_ctrl:
        controller = LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[rewritten, {'use_sim_time': use_sim}],
        )
        nav_nodes.append(TimerAction(period=nav_start_base + 1.5, actions=[controller]))
        nav_node_names.append('controller_server')

    if start_beh:
        behavior = LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[rewritten, {'use_sim_time': use_sim}],
        )
        nav_nodes.append(TimerAction(period=nav_start_base + 3.0, actions=[behavior]))
        nav_node_names.append('behavior_server')

    if start_bt:
        btnav = LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[rewritten, {'use_sim_time': use_sim}],
        )
        nav_nodes.append(TimerAction(period=nav_start_base + 4.5, actions=[btnav]))
        nav_node_names.append('bt_navigator')

    actions.extend(nav_nodes)

    if nav_node_names:
        lm_nav = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_sim_time': use_sim,
                'autostart': autostart,
                'node_names': nav_node_names,
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 60.0,
            }],
        )
        actions.append(TimerAction(period=nav_start_base + 6.0, actions=[lm_nav]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='robot1', description='namespace (robot id)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),

        DeclareLaunchArgument('map_yaml', default_value='/home/morioka/ROS2ForUnity_ws/my_unity_map_2.yaml'),
        DeclareLaunchArgument('start_map_server', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='false'),

        DeclareLaunchArgument('start_amcl', default_value='true'),
        DeclareLaunchArgument('start_planner', default_value='true'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('start_behavior', default_value='true'),
        DeclareLaunchArgument('start_bt', default_value='true'),

        DeclareLaunchArgument('startup_spacing', default_value='4.0', description='amcl 起動後に nav stack を開始するまでの秒数'),
        DeclareLaunchArgument('per_robot_offset', default_value='0.0', description='robot番号ごとの追加遅延（例: 2.0 なら robot2 は +2秒, robot3 は +4秒）'),

        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),

        OpaqueFunction(function=_launch_setup),
    ])
