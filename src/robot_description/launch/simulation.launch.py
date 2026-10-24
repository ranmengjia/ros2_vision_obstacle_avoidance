"""Gazebo Harmonic 仿真启动：加载 world 场景并生成差速机器人。

实现要点（相对迁移前的 Gazebo classic 版本）：
- 用 ros_gz_sim 的 gz_sim.launch.py 启动 Gazebo Sim（Harmonic）
- 用 ros_gz_sim create 生成机器人实体
- 用 ros_gz_bridge 把 Gazebo Transport 话题桥接到 ROS 2
  （/clock、/cmd_vel、/odom、/tf、/scan、相机图像）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file


def _launch_gz_sim(context):
    """按 world_name / headless 参数拼出 gz sim 命令行并启动。"""
    pkg_share = get_package_share_directory('robot_description')
    # world_name 传文件名时相对 worlds/ 目录解析；传绝对路径时
    # os.path.join 会自动丢弃前面的部分
    world_path = os.path.join(
        pkg_share, 'worlds',
        context.perform_substitution(LaunchConfiguration('world_name')))

    gz_args = '-r -v4 '
    if context.perform_substitution(LaunchConfiguration('headless')) == 'true':
        # 只启动服务器、不弹 GUI；渲染仍走当前 X/EGL 显示
        gz_args += '-s '
    gz_args += f'"{world_path}"'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py',
                )
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')

    # xacro → URDF 字符串（launch 时展开，不用预编译）
    robot_desc = process_file(xacro_path).toxml()

    # 发布机器人 TF 树（base_link → 各轮子/相机/雷达）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 在 Gazebo Sim 中生成机器人实体（直接传 URDF 字符串，避免话题竞态）
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'diffbot',
            '-x', '-2.0', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    # Gazebo Transport ↔ ROS 2 话题桥接
    # 方向符号：] = ROS → Gazebo，[ = Gazebo → ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 机器人生成后先发一小段零速度指令：
    # gz 差速插件在收到第一条命令前不会锁住轮子，轮子自由滚动会导致机器人
    # 在静止时缓慢漂移/自转，收到零指令后插件才把轮速保持在 0。
    init_cmd_vel = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-r', '10', '-t', '5',
            '/cmd_vel', 'geometry_msgs/msg/Twist', '{}',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='obstacle_course.world',
            description='Gazebo world 文件名（相对 worlds/ 目录）或绝对路径',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true 时以无界面服务器模式运行 Gazebo',
        ),
        OpaqueFunction(function=_launch_gz_sim),
        robot_state_publisher,
        spawn_entity,
        bridge,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[init_cmd_vel]),
        ),
    ])
