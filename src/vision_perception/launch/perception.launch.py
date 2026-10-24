"""视觉感知节点启动文件。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory('vision_perception'),
        'param', 'perception.yaml')

    # control_mode := reactive（三扇区视觉+雷达避障）/ maze（右手法则走出迷宫）
    control_mode = LaunchConfiguration('control_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='reactive',
            description='reactive：三扇区避障；maze：右手法则找迷宫出口',
        ),
        Node(
            package='vision_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[param_path, {'control_mode': control_mode}],
            output='screen',
        ),
    ])
