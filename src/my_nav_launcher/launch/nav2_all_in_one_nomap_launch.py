# launch/nav2_all_in_one_nomap_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from pathlib import Path

def generate_launch_description():
    # 引数
    namespace     = LaunchConfiguration('ns')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    params_file   = LaunchConfiguration('params_file')

    declare_ns = DeclareLaunchArgument(
        'ns', default_value=TextSubstitution(text='robot1'),
        description='Robot namespace (e.g., robot1)'
    )
    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value=TextSubstitution(text='true'),
        description='Use /clock (sim time)'
    )
    default_params_path = str(
        Path(get_package_share_directory('my_nav_launcher')) / 'config' / 'nav2_multi.yaml'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=TextSubstitution(text=default_params_path),
        description='Nav2 params yaml (generic, no robot name inside)'
    )

    # YAML の書き換え（ロボット固有フレームと共有mapをここで注入）
    param_rewrites = {
        'use_sim_time': use_sim_time,
        # フレーム・トピック
        'robot_base_frame': [namespace, TextSubstitution(text='/base_link')],
        'base_frame_id':    [namespace, TextSubstitution(text='/base_link')],
        'odom_frame':       [namespace, TextSubstitution(text='/odom')],
        'odom_frame_id':    [namespace, TextSubstitution(text='/odom')],
        'global_frame':     'map',
        'map_topic':        '/map',    # 念のため（存在すれば置換）
        'scan_topic':       'scan',
        'odom_topic':       'odom',    # ns付きの /<ns>/odom を購読
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,            # ルートに <ns>: を作って各ノードに適用
        param_rewrites=param_rewrites,
        convert_types=True
    )

    # --- 各ノード（全てライフサイクルノード） ---
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    global_costmap = LifecycleNode(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='global_costmap',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    local_costmap = LifecycleNode(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='local_costmap',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    # --- Lifecycle Manager（mapは外部なので含めない） ---
    lm_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']
        }]
    )

    lm_planner = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['planner_server', 'global_costmap']
        }]
    )

    lm_controller = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server', 'local_costmap']
        }]
    )

    lm_behavior = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_behavior',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['behavior_server']
        }]
    )

    lm_bt = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_bt',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['bt_navigator']
        }]
    )

    group = GroupAction([
        PushRosNamespace(namespace),
        amcl,
        global_costmap,
        planner_server,
        local_costmap,
        controller_server,
        behavior_server,
        bt_navigator,
        lm_localization,
        lm_planner,
        lm_controller,
        lm_behavior,
        lm_bt
    ])

    return LaunchDescription([
        declare_ns,
        declare_use_sim,
        declare_params,
        group
    ])
