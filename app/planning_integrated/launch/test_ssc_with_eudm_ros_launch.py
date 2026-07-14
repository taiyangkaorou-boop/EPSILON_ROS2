"""启动 EUDM 行为决策与 SSC 轨迹规划的 ROS2 集成节点。

本文件只定义节点参数、包内配置路径和话题重映射，不包含规划算法。
场景资源由 ``playgrounds`` 包提供，行为与运动配置分别来自对应规划包。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """构造 EUDM + SSC 集成实验的启动描述。

    Returns:
        LaunchDescription: 包含四个可覆盖参数、启动日志和集成节点的描述对象。
    """
    # 声明外部可覆盖的 ROS2 话题和场景名称，默认使用高速公路交互场景。
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    ctrl_topic = DeclareLaunchArgument(
        'ctrl_topic', default_value='/ctrl/agent_0'
    )
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # 节点参数使用包共享目录拼接，避免依赖开发机上的绝对路径。
    test_ssc_with_eudm_node = Node(
        package='planning_integrated',
        executable='test_ssc_with_eudm',
        name='test_ssc_with_eudm_0',
        output='screen',
        parameters=[{
            'ego_id': 0,
            'desired_vel': 20.0,
            'use_sim_state': True,
            'agent_config_path': PathJoinSubstitution([
                get_package_share_directory('playgrounds'),
                LaunchConfiguration('playground'),
                'agent_config.json'
            ]),
            'bp_config_path': PathJoinSubstitution([
                get_package_share_directory('eudm_planner'),
                'config',
                'eudm_config.pb.txt'
            ]),
            'ssc_config_path': PathJoinSubstitution([
                get_package_share_directory('ssc_planner'),
                'config',
                'ssc_config.pb.txt'
            ])
        }],
        # 将节点内部约定的话题名映射到仿真器/控制器实际使用的话题。
        remappings=[
            ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
            ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
            ('ctrl', LaunchConfiguration('ctrl_topic'))
        ]
    )

    # 启动前打印最终替换值，便于复现实验时核对话题和场景配置。
    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        ctrl_topic,
        playground,
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['ctrl_topic: ', LaunchConfiguration('ctrl_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),
        test_ssc_with_eudm_node
    ])
