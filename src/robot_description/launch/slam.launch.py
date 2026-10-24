"""SLAM 在线建图启动：Gazebo 仿真 + SLAM Toolbox + Nav2 导航栈 + RViz。

与 nav2.launch.py 的区别：
- nav2.launch.py 用静态地图 + AMCL 定位（world 改了就必须重画地图）
- 本文件用 SLAM Toolbox 实时建图：它发布 map->odom TF 和 /map 栅格图，
  Nav2 只启动纯导航栈（navigation_launch.py，不含 map_server / amcl）

用法：
    ros2 launch robot_description slam.launch.py
    ros2 launch robot_description slam.launch.py use_rviz:=false
    ros2 launch robot_description slam.launch.py world_name:=detour_test.world
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    # ---------- 1) Gazebo 场景 + 机器人 ----------
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    # ---------- 2) SLAM Toolbox（online_async 异步建图） ----------
    # 订阅 /scan + odom->base_link TF，发布 map->odom TF 和 /map 栅格图。
    # 等 Gazebo 把机器人 spawn 出来、/scan 和 TF 就绪后再启动，否则首帧
    # 扫描匹配会因为拿不到 TF 而丢弃。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    slam_delayed = TimerAction(period=5.0, actions=[slam])

    # ---------- 3) Nav2 导航栈（不含定位） ----------
    # navigation_launch.py 只拉起 planner / controller / behavior /
    # bt_navigator / velocity_smoother 等纯导航节点；
    # global_costmap 的 static_layer 直接订阅 SLAM 发布的 /map。
    # 延迟到 SLAM 发布 map->odom 之后再激活，避免代价地图激活阶段拿不到 TF。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    # ---------- 4) RViz（可直接用 2D Goal Pose 发目标） ----------
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='detour_test.world',
            description='Gazebo world 文件名（相对 worlds/ 目录）'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_share, 'param', 'slam_toolbox.yaml'),
            description='SLAM Toolbox 参数文件绝对路径'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                pkg_share, 'param', 'nav2_params.yaml'),
            description='Nav2 参数文件绝对路径'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用 Gazebo /clock'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true 时 Gazebo 以无界面服务器模式运行'),
        simulation,
        rviz,
        slam_delayed,
        nav2_delayed,
    ])
