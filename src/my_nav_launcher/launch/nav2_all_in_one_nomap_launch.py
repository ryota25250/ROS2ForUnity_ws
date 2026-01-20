#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ns          = LaunchConfiguration('ns')
    use_simtime = LaunchConfiguration('use_sim_time')
    # 既定のパラメータYAML（必要なら自分のパスに合わせて変更）
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('ns', description='Robot namespace, e.g. robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/moriokalab/ROS2ForUnity_ws/config/nav2_multi.yaml',
            description='Nav2 common params (costmaps, controllers, etc.)'
        ),

        GroupAction([
            PushRosNamespace(ns),

            # --- Localization: AMCL ---
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[params_file, {
                    'use_sim_time': use_simtime
                    # フレームは params_file 側で map / <ns>/odom / <ns>/base_link を設定
                }]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_simtime,
                    'autostart': True,
                    'node_names': ['amcl']
                }]
            ),

            # --- Global planner ---
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[params_file, {
                    'use_sim_time': use_simtime
                }]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_planner',
                output='screen',
                parameters=[{
                    'use_sim_time': use_simtime,
                    'autostart': True,
                    'node_names': ['planner_server']
                }]
            ),

            # --- Controller (local planner + local costmap) ---
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[params_file, {
                    'use_sim_time': use_simtime,
                    # コントローラが使うオドムのトピックは多ロボ対応で明示
                    'odom_topic': PathJoinSubstitution(['/', ns, 'odom'])
                }]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': use_simtime,
                    'autostart': True,
                    'node_names': ['controller_server']
                }]
            ),

            # --- Behavior server ---
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[params_file, {
                    'use_sim_time': use_simtime
                }]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_behavior',
                output='screen',
                parameters=[{
                    'use_sim_time': use_simtime,
                    'autostart': True,
                    'node_names': ['behavior_server']
                }]
            ),

            # --- BT Navigator ---
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[params_file, {
                    'use_sim_time': use_simtime
                    # robot_base_frame は params_file 側（もしくは別途 set-param）で robotN/base_link を指す
                }]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_bt',
                output='screen',
                parameters=[{
                    'use_sim_time': use_simtime,
                    'autostart': True,
                    'node_names': ['bt_navigator']
                }]
            ),
        ])
    ])
