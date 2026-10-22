"""视觉感知节点启动文件。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory('vision_perception'),
        'param', 'perception.yaml')

    return LaunchDescription([
        Node(
            package='vision_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[param_path],
            output='screen',
        ),
    ])