"""启动固定 Linux joystick 设备 `/dev/input/js0` 的 ROS2 joy_node。"""

from launch import LaunchDescription
from launch_ros.actions import Node
# 以下保留的模板 import 展示其它 launch 能力，当前文件均未启用。
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """构造只包含 joy_node 的 LaunchDescription。"""

    # joystick 设备路径固定，发布 topic 和 QoS 使用 joy 包默认值。
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]
    )

    # 返回单节点启动描述。
    return LaunchDescription([
        joy_node
    ])
