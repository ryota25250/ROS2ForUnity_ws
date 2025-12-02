#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml

PARAMS_FILE = '/home/moriokalab/ROS2ForUnity_ws/config/nav2_multi.yaml'

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='robot1')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ns = LaunchConfiguration('ns')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ★ 相対名YAMLを ns 付きに“確実”変換（controller_server と local_costmap）
    param_substs = {
        # controller_server 側：/robotX/odom を明示
        'controller_server.ros__parameters.odom_topic': [ns, TextSubstitution(text='/odom')],

        # local_costmap 側：フレーム & センサ系を ns 付きに
        'local_costmap.local_costmap.ros__parameters.global_frame': [ns, TextSubstitution(text='/odom')],
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': [ns, TextSubstitution(text='/base_link')],
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': [ns, TextSubstitution(text='/scan')],
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.sensor_frame': [ns, TextSubstitution(text='/base_scan')],
    }

    rewritten = RewrittenYaml(
        source_file=PARAMS_FILE,
        root_key=ns,
        param_rewrites=param_substs,
        convert_types=True
    )

    controller = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=ns,
        output='screen',
        parameters=[rewritten, {'use_sim_time': use_sim_time}],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server'],
            'bond_timeout': 0.0
        }]
    )

    return LaunchDescription([ns_arg, use_sim_time_arg, controller, lifecycle_mgr])
