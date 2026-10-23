# ROS2 视觉避障与可通行区域检测

基于 ROS 2 Humble + Gazebo classic 11 的差速机器人避障学习项目：
从单目 RGB 传统视觉基线起步，逐步加入 2D 激光雷达融合、Nav2 导航与 SLAM 在线建图。

## 版本历程

| Tag | 版本 | 内容 |
|---|---|---|
| v1.0.0 | 基础闭环 | 单目 RGB → 亮度阈值 + 形态学 → 可通行掩膜 → 左/中/右评分 → 滞回决策 → `/cmd_vel`；图像超时强制停车；50 条 pytest 单测 |
| v1.1.0 | 鲁棒性 | 弱光/低对比度多场景、角速度斜坡平滑 + 双端滞回、FPS/延迟统计、三场景 A/B 评估（[实验报告](docs/experiments/v1.1_ab_report.md)） |
| v2.0.0 | 融合感知 + 导航 | 2D 激光雷达接入、视觉+雷达安全闸门融合（双看门狗）、Nav2（静态地图 + AMCL + DWB）、SLAM Toolbox 在线建图、DL 引入评估（结论：暂不引入） |

## 技术基线

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 发行版 | ROS 2 Humble |
| 仿真器 | Gazebo classic 11.10.2 |
| 图像处理 | OpenCV 4.5.4 + NumPy 1.21.5 |
| 导航 | Nav2 + SLAM Toolbox |

机器人：0.30m × 0.20m 差速底盘，单目相机（0.08m 高，FOV 60°），
2D 激光雷达（360 点 × 10Hz，量程 0.1~8.0m），限速 0.2m/s / 0.8rad/s。

## 仓库结构

```
src/
  robot_description/    # xacro 模型、world 场景、Nav2/SLAM 参数与地图、launch
  vision_perception/    # 视觉+雷达融合避障节点（感知/决策/控制/安全一体化）
docs/
  requirements.md       # M0 需求、接口、风险与验收标准
  experiments/          # v1.1 三场景 A/B 实验报告与数据
  decisions/            # 关键技术决策记录（ADR）
PLAN.md                 # 总体路线图与各版本验收记录
```

## 快速开始

```bash
# 依赖（ROS 2 Humble 已装的前提下）
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# 构建
cd ~/ros2_vision_obstacle_avoidance
colcon build
source install/setup.bash
```

三种运行方式（注意：都会发布 `/cmd_vel`，同一时间只能跑一种）：

```bash
# 1) 视觉+雷达融合避障演示（两个终端）
ros2 launch robot_description simulation.launch.py            # Gazebo + 机器人
ros2 launch vision_perception perception.launch.py            # 融合避障节点

# 2) Nav2 导航（静态地图 + AMCL），RViz 里用 2D Goal Pose 发目标
ros2 launch robot_description nav2.launch.py

# 3) SLAM 在线建图 + 导航（无需预画地图）
ros2 launch robot_description slam.launch.py
```

## 测试

```bash
colcon test --packages-select vision_perception
# 或直接在源码目录
python3 -m pytest src/vision_perception/test/ -v
```

50 条单元测试覆盖：掩膜生成、三分评分、滞回决策、全受阻停车、
雷达扇区映射、融合闸门、双看门狗等纯函数逻辑。

## 文档

- [PLAN.md](PLAN.md)：版本路线图、接口定义、各步骤验收数据
- [docs/decisions/](docs/decisions/)：关键决策记录（为什么不用深度学习、为什么融合用安全闸门等）
- [docs/experiments/v1.1_ab_report.md](docs/experiments/v1.1_ab_report.md)：鲁棒性 A/B 实验

## License

见 [LICENSE](LICENSE)。
