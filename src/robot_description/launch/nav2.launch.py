"""Nav2 导航启动：Gazebo 仿真 + 静态地图定位（AMCL）+ 导航栈 + RViz。

用法：
    ros2 launch robot_description nav2.launch.py
    ros2 launch robot_description nav2.launch.py use_rviz:=false
    ros2 launch robot_description nav2.launch.py world_name:=detour_test.world
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

    # ---------- 1) Gazebo 场景 + 机器人 ----------
    # world_name / 机器人初始位姿 (-2, 0) 必须与地图和 AMCL 初始位姿一致
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
        }.items(),
    )

    # ---------- 2) Nav2 导航栈 ----------
    # bringup_launch.py 内部会拉起 map_server + amcl + planner + controller
    # + behavior + bt_navigator + velocity_smoother，并由 lifecycle_manager 自动激活
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            # 不开组合模式：每个 Nav2 节点独立进程，日志和崩溃定位更直观
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    # 等 Gazebo 把机器人 spawn 出来、/scan 和 odom->base_link 就绪后再启动 Nav2，
    # 否则 AMCL/代价地图在激活阶段可能因为拿不到 TF 而报错。
    nav2_delayed = TimerAction(period=5.0, actions=[nav2])

    # ---------- 3) RViz（可直接用 2D Goal Pose 发目标） ----------
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
            description='Gazebo world 文件名（相对 worlds/）'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_share, 'maps', 'detour_test_map.yaml'),
            description='静态地图 yaml 的绝对路径（必须与 world 匹配）'),
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
        simulation,
        rviz,
        nav2_delayed,
    ])
