"""启动 playground 物理仿真器，并可重映射静态/动态 ArenaInfo topic。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """构造 joy_node、场景路径参数和 phy_simulator 节点的启动描述。"""

    # 声明可覆盖的真值 topic 和 playground 场景目录名。
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # 从安装后的 phy_simulator share/launch 包含 joystick 启动文件。
    joy_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('phy_simulator'),
                'launch',
                'joy_ctrl_launch.py'
            ])
        ])
    )

    # 三份场景资源都从 playgrounds 包共享目录下的同一场景子目录解析。
    vehicle_info_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'vehicle_set.json'
    ])
    map_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'obstacles_norm.json'
    ])
    lane_net_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'lane_net_norm.json'
    ])

    # 把场景路径注入仿真节点，并重映射静态/动态真值输出。
    phy_simulator_planning_node = Node(
        package='phy_simulator',
        executable='phy_simulator_planning_node',
        name='phy_simulator_planning_node',
        output='screen',
        parameters=[{
            'vehicle_info_path': vehicle_info_path,
            'map_path': map_path,
            'lane_net_path': lane_net_path
        }],
        remappings=[
            ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
            ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic'))
        ]
    )

    # 启动前打印最终 topic、场景和资源路径，便于实验复现。
    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        playground,
        joy_ctrl_launch,
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg=['vehicle_info_path: ', vehicle_info_path]),
        LogInfo(msg=['map_path: ', map_path]),
        LogInfo(msg=['lane_net_path: ', lane_net_path]),
        LogInfo(msg="Launching node..."),
        phy_simulator_planning_node
    ])
