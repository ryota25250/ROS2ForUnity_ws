#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

PARAMS_FILE = '/home/moriokalab/ROS2ForUnity_ws/config/nav2_multi.yaml'

def _pick_or_fallback_bt_xml():
    """
    1) nav2_bt_navigator の behavior_trees ディレクトリから
       'navigate_to_pose*.xml' を優先的に探す
    2) どちらも無ければ /tmp/nav2_min_bt.xml に最小構成のBTを生成して返す
    """
    bt_dir = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    primary = os.path.join(bt_dir, 'navigate_to_pose.xml')
    fallback = os.path.join(bt_dir, 'navigate_to_pose_w_replanning.xml')

    if os.path.exists(primary):
        return primary
    if os.path.exists(fallback):
        return fallback

    # --- 最小構成BT（ComputePathToPose -> FollowPath）を生成 ---
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
    tmp_path = '/tmp/nav2_min_bt.xml'
    with open(tmp_path, 'w', encoding='utf-8') as f:
        f.write(minimal_xml)
    return tmp_path

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ns = LaunchConfiguration('ns')
    use_sim_time = LaunchConfiguration('use_sim_time')

    safe_xml = _pick_or_fallback_bt_xml()

    # ns配下へYAMLを展開しつつ、確実に frame / BT を上書き
    param_substs = {
        'bt_navigator.ros__parameters.global_frame': 'map',
        'bt_navigator.ros__parameters.robot_base_frame': [ns, TextSubstitution(text='/base_link')],
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': safe_xml,
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': safe_xml,  # 複数ゴールも一旦同一
        'bt_navigator.ros__parameters.server_timeout': '5000',
        # plugin_lib_names は YAML 側が未設定なら省略可（Humble標準は自動ロード可）
        # 必要なら以下を有効化:
        # 'bt_navigator.ros__parameters.plugin_lib_names': [
        #     'nav2_compute_path_to_pose_action_bt_node',
        #     'nav2_follow_path_action_bt_node',
        #     'nav2_clear_costmap_service_bt_node',
        #     'nav2_goal_reached_condition_bt_node',
        #     'nav2_initial_pose_received_condition_bt_node',
        #     'nav2_goal_updated_condition_bt_node',
        #     'nav2_rate_controller_bt_node',
        #     'nav2_pipeline_sequence_bt_node',
        # ],
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_substs,
        convert_types=True
    )

    btnav = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=ns,
        output='screen',
        parameters=[
            rewritten,
            {
                'use_sim_time': use_sim_time,
                'global_frame': 'map',
                'robot_base_frame': [ns, TextSubstitution(text='/base_link')],
                'default_nav_to_pose_bt_xml': safe_xml,
                'default_nav_through_poses_bt_xml': safe_xml,
                'server_timeout': 5000,
            }
        ],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_bt',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['bt_navigator'],
            'bond_timeout': 0.0
        }]
    )

    return LaunchDescription([ns_arg, use_sim_time_arg, btnav, lifecycle_mgr])
