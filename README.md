# ROS2 视觉避障与可通行区域检测

基于 ROS 2 Jazzy + Gazebo Harmonic 8 的差速机器人避障学习项目：
从单目 RGB 传统视觉基线起步，逐步加入 2D 激光雷达融合、Nav2 导航与 SLAM 在线建图。

## 版本历程

| Tag | 版本 | 内容 |
|---|---|---|
| v1.0.0 | 基础闭环 | 单目 RGB → 亮度阈值 + 形态学 → 可通行掩膜 → 左/中/右评分 → 滞回决策 → `/cmd_vel`；图像超时强制停车；50 条 pytest 单测 |
| v1.1.0 | 鲁棒性 | 弱光/低对比度多场景、角速度斜坡平滑 + 双端滞回、FPS/延迟统计、三场景 A/B 评估（[实验报告](docs/experiments/v1.1_ab_report.md)） |
| v2.0.0 | 融合感知 + 导航 | 2D 激光雷达接入、视觉+雷达安全闸门融合（双看门狗）、Nav2（静态地图 + AMCL + SmacPlanner2D + RPP）、SLAM Toolbox 在线建图、DL 引入评估（结论：暂不引入） |
| v2.2.0 | 迷宫自主脱困 | 5×5 完美迷宫 world（唯一出口）；`control_mode:=maze` 只用雷达前/左前/右前距离 + 左右两侧最大开阔度 + 视觉中路评分走出迷宫，全程不启动 Nav2、不下发目标点 |
| v3.0.0 | 反应式 vs Nav2 基准 | 5 臂（纯 OpenCV / 纯雷达 / 视觉+雷达 / Nav2 / Nav2+视觉护栏）× 5 场景的自动基准：统一指标采集、一键出表出图，量化成功率、路程、间距与 CPU 开销 |

## 技术基线

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 24.04.5 LTS |
| ROS 发行版 | ROS 2 Jazzy |
| 仿真器 | Gazebo Harmonic 8.15.0（`gz sim`） |
| 仿真桥接 | `ros_gz_bridge/parameter_bridge` |
| 图像处理 | Python 3.12 + OpenCV 4.6.0 + NumPy 1.26.4 |
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
tools/benchmark/        # v3.0 五臂基准：场景矩阵、指标采集、报告生成
PLAN.md                 # 总体路线图与各版本验收记录
```

## 快速开始

```bash
# 依赖（ROS 2 Jazzy 已装的前提下）
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
                 ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-cv-bridge

# 构建
cd ~/ros2_vision_obstacle_avoidance
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

三种运行方式（注意：都会发布 `/cmd_vel`，同一时间只能跑一种）：

```bash
# 1) 视觉+雷达融合避障演示（两个终端）
ros2 launch robot_description simulation.launch.py            # Gazebo 界面 + 机器人
ros2 launch vision_perception perception.launch.py            # 融合避障节点

# 2) Nav2 导航（静态地图 + AMCL），RViz 里用 2D Goal Pose 发目标
ros2 launch robot_description nav2.launch.py

# 3) SLAM 在线建图 + 导航（无需预画地图）
ros2 launch robot_description slam.launch.py

# 4) 迷宫自主脱困（纯视觉 + 雷达，无 Nav2、无目标点）
ros2 launch robot_description simulation.launch.py \
         world_name:=maze_course.world spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
ros2 launch vision_perception perception.launch.py control_mode:=maze
```

迷宫模式说明：5×5 完美迷宫，走廊 1.2m、墙高 1.0m，外墙上只有东侧
`(3.0, -1.2)` 一个缺口。机器人贴右墙走，遇墙用左右各 125° 大扇区的最远距离
比较哪边更开阔再决定转向，前后带滞回（`maze_front_stop` 0.6m 进入转向、
`maze_front_clear` 1.0m 才退出），避免在墙角反复蹭墙。前/左/右三方向同时
大于 `maze_exit_distance`(4.0m) 并保持 0.5s 即判定找到出口并停车。

常用启动参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `world_name` | `detour_test.world` | 换场景，取值在 `src/robot_description/worlds/` |
| `headless` | `false` | `true` 时只跑 `gz sim -s`（无 Gazebo 窗口，省 CPU） |
| `use_rviz` | `true` | `false` 时不自动开 RViz |
| `spawn_x` / `spawn_y` / `spawn_yaw` | `-2.0` / `0.0` / `0.0` | 机器人生成位姿；迷宫场景请用 `0.0 / 0.0 / 0.0` |
| `control_mode` | `reactive` | `reactive` 三扇区避障；`maze` 迷宫贴墙脱困 |

```bash
# 例：无界面换场景
ros2 launch robot_description simulation.launch.py headless:=true world_name:=dim_course.world
```

> Gazebo Harmonic 的 GUI 需要能访问 `/dev/dri/*`。若日志出现
> `libEGL warning: failed to open /dev/dri/card*`，说明退化成 llvmpipe 软件渲染
> （能显示但 CPU 占用高），把用户加入 `video`/`render` 组后重新登录即可。

## 基准：OpenCV 反应式 vs Nav2

`tools/benchmark/` 把「纯 OpenCV 反应式避障」和「Nav2 导航」放在同一套指标下比较，
并额外提供两条消融/融合臂：

| 臂 | 决策方式 | 说明 |
|---|---|---|
| `vision` | OpenCV 掩膜 → 左/中/右评分 | 纯反应式，无地图、无定位 |
| `lidar` | 雷达三扇区 | 只验证视觉在感知中的贡献 |
| `fused` | 视觉 + 雷达安全闸门 | 项目默认臂 |
| `nav2` | AMCL + SmacPlanner2D + RPP | 先验地图 + 全局规划 |
| `hybrid` | Nav2 输出 → OpenCV 近距护栏 | Nav2 的 `/cmd_vel` 改走 `/cmd_vel_raw`，`vision_guard.py` 在正前方过近时接管降速/转向 |

```bash
# 全量（5 场景 × 各场景配置的臂），自动起停 Gazebo/Nav2 并采集指标
python3 tools/benchmark/run_benchmark.py --runs 1

# 单场景 / 单臂
python3 tools/benchmark/run_benchmark.py --scenario detour_test --arm vision --runs 1

# 汇总成 Markdown 报告与分组柱状图
python3 tools/benchmark/report.py --chart
```

场景矩阵在 [scenarios.yaml](tools/benchmark/scenarios.yaml)（`detour_test`、
`obstacle_course`、`low_contrast_course`、`dim_course`、`maze_course`），
单次结果写入 `results/<场景>/<臂>/run_*.json`，报告与图表写入
`results/report.md`、`results/chart.png`。当前结论见
[results/report.md](tools/benchmark/results/report.md)：

- 结构化场景（detour / obstacle / 弱光 / 低对比度）：纯 OpenCV 与 Nav2 都能通过，
  但纯 OpenCV 对「必须先横移到缺口再前进」的绕行会撞上障碍（detour_test 碰撞 2 次）；
- 迷宫：反应式臂靠贴墙脱困需 ~154s / 22m，Nav2 用先验地图 ~92s / 17.7m，最小间距 0.41m；
- 融合臂在所有场景成功率 100%，说明雷达补上了单目测距缺失的那一半；
- 加视觉护栏的 hybrid 与纯 Nav2 用时基本一致（46.9s vs 47.0s，共同场景均值），
  因为正常导航时护栏不介入，只在危险距离兜底。

## 测试

```bash
colcon test --packages-select vision_perception
# 或直接在源码目录
python3 -m pytest src/vision_perception/test/ -v
# 基准指标纯函数
python3 -m pytest tools/benchmark/test_metrics.py -v
```

76 条单元测试覆盖：掩膜生成、三分评分、滞回决策、全受阻停车、
雷达扇区映射、融合闸门、双看门狗、迷宫贴墙转向与出口判定等纯函数逻辑。
另有 38 条基准指标单测（成功率判定、最小间距、碰撞计数、卡死占比、jerk）。

## 文档

- [PLAN.md](PLAN.md)：版本路线图、接口定义、各步骤验收数据
- [docs/decisions/](docs/decisions/)：关键决策记录（为什么不用深度学习、为什么融合用安全闸门等）
- [docs/experiments/v1.1_ab_report.md](docs/experiments/v1.1_ab_report.md)：鲁棒性 A/B 实验
- [tools/benchmark/results/report.md](tools/benchmark/results/report.md)：v3.0 五臂基准报告（场景 × 臂指标表与图表）

## License

见 [LICENSE](LICENSE)。
