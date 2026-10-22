"""Gazebo 仿真启动：加载 world 场景并生成差速机器人。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from xacro import process_file

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')

    # world_name 传文件名时相对 worlds/ 目录解析；传绝对路径时 os.path.join
    # 会自动丢弃前面的部分，两种写法都能得到正确的 world 路径
    world_path = PathJoinSubstitution(
        [pkg_share, 'worlds', LaunchConfiguration('world_name')])

    # xacro → URDF 字符串（launch 时展开，不用预编译）
    robot_desc = process_file(xacro_path).toxml()

    # Gazebo 自带的 launch：拉起 gzserver（+ 可选 gzclient）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py',
            )
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 发布机器人 TF 树（base_link → 各轮子/相机）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 在 Gazebo 中生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diffbot',
            '-x', '-2.0', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='obstacle_course.world',
            description='Gazebo world 文件名（相对 worlds/ 目录）或绝对路径',
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])