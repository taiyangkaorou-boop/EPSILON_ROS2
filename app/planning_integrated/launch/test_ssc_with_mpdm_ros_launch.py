"""启动 MPDM 行为规划与 SSC 轨迹规划的 ROS2 集成节点。

该启动入口与 EUDM 入口保持相同的语义地图和 SSC 下层，主要用于比较不同
行为决策器。所有路径均从 ROS2 包共享目录解析，避免绑定具体工作空间。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """构造 MPDM + SSC 集成实验的启动描述。

    Returns:
        LaunchDescription: 参数声明、诊断日志及 MPDM 集成节点的组合。
    """
    return LaunchDescription([
        # 可在命令行覆盖的输入/输出话题与测试场景。
        DeclareLaunchArgument(
            'arena_info_static_topic', default_value='/arena_info_static'
        ),
        DeclareLaunchArgument(
            'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
        ),
        DeclareLaunchArgument(
            'ctrl_topic', default_value='/ctrl/agent_0'
        ),
        DeclareLaunchArgument(
            'playground', default_value='highway_lite'
        ),

        # 输出最终启动配置，为后续批量实验保存可追溯的场景信息。
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['ctrl_topic: ', LaunchConfiguration('ctrl_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),

        # MPDM 行为层与 SSC 运动层共处一个集成进程，由参数指定场景和规划配置。
        Node(
            package='planning_integrated',
            executable='test_ssc_with_mpdm',
            name='test_ssc_with_mpdm_0',
            output='screen',
            parameters=[{
                'ego_id': 0,
                'desired_vel': 60.0,
                'use_sim_state': True,
                'agent_config_path': PathJoinSubstitution([
                    get_package_share_directory('playgrounds'),
                    LaunchConfiguration('playground'),
                    'agent_config.json'
                ]),
                'ssc_config_path': PathJoinSubstitution([
                    get_package_share_directory('ssc_planner'),
                    'config',
                    'ssc_config.pb.txt'
                ])
            }],
            # 对齐仿真器发布的地图话题和车辆控制话题。
            remappings=[
                ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('ctrl', LaunchConfiguration('ctrl_topic'))
            ]
        )
    ])
