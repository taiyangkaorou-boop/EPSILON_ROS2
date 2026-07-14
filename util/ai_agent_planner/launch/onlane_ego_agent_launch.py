# 启动 ID 0 的单个 MPDM/on-lane ego 仿真参与者。

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # 地图输入 topic、ego 速度/自动驾驶等级和 playground。
    arena_info_topic = DeclareLaunchArgument(
        'arena_info_topic', default_value='/arena_info'
    )
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    global_desired_vel = DeclareLaunchArgument(
        'global_desired_vel', default_value='10.0'
    )
    global_autonomous_level = DeclareLaunchArgument(
        'global_autonomous_level', default_value='3'
    )
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # 单 ego launch 未显式传 aggressiveness_level，使用节点默认值 3。
    onlane_ai_agent_node_0 = Node(
        package='ai_agent_planner',
        executable='onlane_ai_agent',
        name='onlane_ai_agent_0',
        output='screen',
        parameters=[{
            'ego_id': 0,
            'agent_config_path': PathJoinSubstitution([
                get_package_share_directory('playgrounds'),
                LaunchConfiguration('playground'),
                'agent_config.json'
            ]),
            'desired_vel': LaunchConfiguration('global_desired_vel'),
            'autonomous_level': LaunchConfiguration('global_autonomous_level')
        }],
        remappings=[
            ('arena_info', LaunchConfiguration('arena_info_topic')),
            ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
            ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
            ('ctrl', '/ctrl/agent_0')
        ]
    )

    # 注册 launch 参数和诊断日志后启动 ego 节点。
    return LaunchDescription([
        arena_info_topic,
        arena_info_static_topic,
        arena_info_dynamic_topic,
        global_desired_vel,
        global_autonomous_level,
        playground,
        LogInfo(msg=['arena_info_topic: ', LaunchConfiguration('arena_info_topic')]),
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['global_desired_vel: ', LaunchConfiguration('global_desired_vel')]),
        LogInfo(msg=['global_autonomous_level: ', LaunchConfiguration('global_autonomous_level')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),
        onlane_ai_agent_node_0
    ])
