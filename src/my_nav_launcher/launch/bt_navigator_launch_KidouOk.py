#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from glob import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

PARAMS_FILE = '/home/moriokalab/ROS2ForUnity_ws/config/nav2_multi.yaml'

def _pick_bt(bt_dir: str, candidates: list[str]) -> str:
    """Return the first existing BT xml path from candidates (relative to bt_dir)."""
    for name in candidates:
        path = os.path.join(bt_dir, name)
        if os.path.exists(path):
            return path
    # fallback: pick any xml in the folder (for custom installations)
    xmls = sorted(glob(os.path.join(bt_dir, '*.xml')))
    if xmls:
        return xmls[0]
    # as a last resort, return the first candidate path (will error, but message is clear)
    return os.path.join(bt_dir, candidates[0])

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ns = LaunchConfiguration('ns')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # === 1) BTファイルを動的解決 ===
    bt_dir = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')

    # navigate_to_pose 用の候補（環境によってファイル名が揺れます）
    nav_to_pose_candidates = [
        'navigate_w_replanning_and_recovery.xml',
        'navigate_to_pose_w_replanning_and_recovery.xml',
        'navigate_w_replanning.xml',
        'navigate_to_pose_w_replanning.xml',
    ]
    nav_through_candidates = [
        'navigate_through_poses_w_replanning_and_recovery.xml',
        'navigate_through_poses_w_replanning.xml',
    ]

    nav_to_pose_xml = _pick_bt(bt_dir, nav_to_pose_candidates)
    nav_through_xml = _pick_bt(bt_dir, nav_through_candidates)

    # === 2) ns配下へYAML展開（必要最低限のリライトだけ） ===
    param_substs = {
        'bt_navigator.ros__parameters.global_frame': 'map',
        'bt_navigator.ros__parameters.robot_base_frame': [ns, TextSubstitution(text='/base_link')],
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': nav_to_pose_xml,
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': nav_through_xml,
        'bt_navigator.ros__parameters.server_timeout': '5000',
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_substs,
        convert_types=True
    )

    # === 3) LifecycleNode で “最終上書き” を直指定（確実に反映させる） ===
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
                'default_nav_to_pose_bt_xml': nav_to_pose_xml,
                'default_nav_through_poses_bt_xml': nav_through_xml,
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
