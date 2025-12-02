#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

# あなたの共通パラメータ（ロボット名を含まない版）
PARAMS_FILE = '/home/moriokalab/ROS2ForUnity_ws/config/nav2_multi.yaml'

def _pick_or_fallback_bt_xml():
    bt_dir = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    primary = os.path.join(bt_dir, 'navigate_to_pose.xml')
    repl   = os.path.join(bt_dir, 'navigate_to_pose_w_replanning.xml')
    if os.path.exists(primary):
        return primary
    if os.path.exists(repl):
        return repl
    # 最小構成を /tmp に生成（ComputePathToPose -> FollowPath）
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
    # ==== 引数 ====
    ns_arg          = DeclareLaunchArgument('ns', default_value='robot1', description='namespace (robot id)')
    use_sim_arg     = DeclareLaunchArgument('use_sim_time', default_value='true')
    map_yaml_arg    = DeclareLaunchArgument('map_yaml', default_value='/home/moriokalab/ROS2ForUnity_ws/my_unity_map_2.yaml')
    autostart_arg   = DeclareLaunchArgument('autostart', default_value='true')
    start_map_arg   = DeclareLaunchArgument('start_map_server', default_value='true', description='start global map_server')
    start_amcl_arg  = DeclareLaunchArgument('start_amcl', default_value='true')
    start_plan_arg  = DeclareLaunchArgument('start_planner', default_value='true')
    start_ctrl_arg  = DeclareLaunchArgument('start_controller', default_value='true')
    start_beh_arg   = DeclareLaunchArgument('start_behavior', default_value='true')
    start_bt_arg    = DeclareLaunchArgument('start_bt', default_value='true')

    ns          = LaunchConfiguration('ns')
    use_sim     = LaunchConfiguration('use_sim_time')
    map_yaml    = LaunchConfiguration('map_yaml')
    autostart   = LaunchConfiguration('autostart')
    start_map   = LaunchConfiguration('start_map_server')
    start_amcl  = LaunchConfiguration('start_amcl')
    start_plan  = LaunchConfiguration('start_planner')
    start_ctrl  = LaunchConfiguration('start_controller')
    start_beh   = LaunchConfiguration('start_behavior')
    start_bt    = LaunchConfiguration('start_bt')

    base_link = [ns, TextSubstitution(text='/base_link')]
    odom      = [ns, TextSubstitution(text='/odom')]

    # ==== 共通：ns 配下にYAMLを展開しつつ必要箇所を上書き ====
    #   - global_costmap は map フレーム＆/map を購読
    #   - local_costmap / controller は ns 付きフレーム＆トピック
    #   - BT は global_frame=map, robot_base_frame=<ns>/base_link, 安全なBT XMLを指定
    safe_bt = _pick_or_fallback_bt_xml()
    param_rewrites = {
        # planner / global_costmap
        'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': base_link,
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': '/map',
        'global_costmap.global_costmap.ros__parameters.static_layer.subscribe_to_updates': 'true',
        'global_costmap.global_costmap.ros__parameters.static_layer.map_subscribe_transient_local': 'true',
        'global_costmap.global_costmap.ros__parameters.transform_tolerance': '0.3',

        # controller / local_costmap
        'controller_server.ros__parameters.odom_topic': [ns, TextSubstitution(text='/odom')],
        'local_costmap.local_costmap.ros__parameters.global_frame': odom,
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': base_link,
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': [ns, TextSubstitution(text='/scan')],
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.sensor_frame': [ns, TextSubstitution(text='/base_scan')],

        # bt_navigator
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

    # ==== map_server（グローバル /map を配信）====
    map_server = LifecycleNode(
        condition=IfCondition(start_map),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'yaml_filename': map_yaml,
            'frame_id': 'map',        # /map フレーム
        }],
        remappings=[('map', '/map'), ('map_metadata', '/map_metadata')],
    )
    lm_map = Node(
        condition=IfCondition(start_map),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )

    # ==== AMCL（/map を購読、frames は ns 付き）====
    amcl = LifecycleNode(
        condition=IfCondition(start_amcl),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'base_frame_id': base_link,
            'odom_frame_id': odom,
            'global_frame_id': 'map',   # グローバル map
            'scan_topic': 'scan',       # ns が付く → <ns>/scan
            'map_topic': '/map',        # グローバル /map を読む
        }],
        remappings=[('map','/map'), ('map_metadata','/map_metadata')]
    )
    lm_loc = Node(
        condition=IfCondition(start_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['amcl'],
        }]
    )

    # ==== planner_server ====
    planner = LifecycleNode(
        condition=IfCondition(start_plan),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim}],
        remappings=[('map','/map'), ('map_metadata','/map_metadata')],
    )
    lm_plan = Node(
        condition=IfCondition(start_plan),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['planner_server'],
            'bond_timeout': 0.0
        }]
    )

    # ==== controller_server ====
    controller = LifecycleNode(
        condition=IfCondition(start_ctrl),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim}],
    )
    lm_ctrl = Node(
        condition=IfCondition(start_ctrl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['controller_server'],
            'bond_timeout': 0.0
        }]
    )

    # ==== behavior_server ====
    behavior = LifecycleNode(
        condition=IfCondition(start_beh),
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim}],
    )
    lm_beh = Node(
        condition=IfCondition(start_beh),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_behavior',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['behavior_server'],
            'bond_timeout': 0.0
        }]
    )

    # ==== bt_navigator ====
    btnav = LifecycleNode(
        condition=IfCondition(start_bt),
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
    lm_bt = Node(
        condition=IfCondition(start_bt),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_bt',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': autostart,
            'node_names': ['bt_navigator'],
            'bond_timeout': 0.0
        }]
    )

    return LaunchDescription([
        ns_arg, use_sim_arg, map_yaml_arg, autostart_arg,
        start_map_arg, start_amcl_arg, start_plan_arg, start_ctrl_arg, start_beh_arg, start_bt_arg,
        # 起動順の都合で個別 GroupAction にしてもOKだが、ここでは直列記述
        map_server, lm_map,
        amcl, lm_loc,
        planner, lm_plan,
        controller, lm_ctrl,
        behavior, lm_beh,
        btnav, lm_bt,
    ])
