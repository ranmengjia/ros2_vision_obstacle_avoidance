# ROS2 视觉避障与可通行区域检测 · 总体规划

> 本文件是整个项目的路线图，也是每个版本对话的交接文档。
> 开新对话时只需说：**"按 PLAN.md 执行 vX.Y 第 N 步"**，不用翻历史记录。

## 工程约定

- 每个版本一个独立对话。
- 每个功能一个 feature 分支，一个 commit。
- 分支验证通过后才用 `--no-ff` 合回 main。
- Conventional Commits：`feat` / `fix` / `docs` / `chore` / `test` / `refactor` / `perf` / `style` / `revert`。
- 每步必须验证：`colcon build` 通过；涉及 xacro 的验证展开；涉及 Python 的验证 `py_compile`。
- 不自行执行 git commit / push，必须用户确认。

## 技术基线（已验证环境）

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 发行版 | ROS 2 Humble |
| 仿真器 | Gazebo classic 11.10.2 |
| 图像处理 | OpenCV 4.5.4 + NumPy 1.21.5 |
| 桥接 | cv_bridge |
| 构建 | colcon |

## v1.0 —— 基础闭环（单目 RGB + OpenCV 传统基线）

| 步骤 | 分支 | type | 内容 | 验收 |
|---|---|---|---|---|
| 0 | main | chore | 仓库骨架：.gitignore / LICENSE / PLAN.md / 目录占位 | 目录结构正确 |
| 1 | feat/m0-docs | docs | docs/requirements.md：环境、接口、风险、验收标准 | 覆盖接口与 6 项风险 |
| 2 | feat/robot-description | feat | 差速底盘 xacro + 摄像头 + Gazebo world + launch | colcon build 通过 + xacro 展开无错 |
| 3 | feat/vision-perception-core | feat | 感知/决策/控制/安全一体化节点 + 顶层演示 launch | build + py_compile + launch 语法通过 |
| 4 | feat/pytest-unit-tests | test | 掩膜 / 三分评分 / 滞回决策 / 全受阻停止 单测 | pytest 全绿 |
| 5 | — | — | 创建 GitHub 新仓库并 push v1.0 | clone 后能 colcon build |

### v1.0 功能边界

- 做：单目 RGB → 亮度阈值 + 形态学 → 可通行掩膜 → 左/中/右评分 → 滞回决策 → /cmd_vel
- 做：图像超时 1s 强制停车、全部方向受阻停止
- 不做：深度摄像头、激光雷达、深度学习、精确测距

### v1.0 接口

| 话题 | 方向 | 类型 |
|---|---|---|
| /camera/image_raw | 仿真 → 感知 | sensor_msgs/Image |
| /cmd_vel | 控制 → 底盘 | geometry_msgs/Twist |

### v1.0 包结构

- `src/robot_description`：xacro 模型、world、spawn launch
- `src/vision_perception`：感知 + 决策 + 控制 + 安全（单节点，接口稳定后再拆）
- `launch/simulation_demo.launch.py`：顶层演示入口

## v1.1 —— 鲁棒性优化

| 步骤 | 内容 |
|---|---|
| 6 | 多场景 world：不同光照、不同地面配色 |
| 7 | 方向平滑 + 滞回调优 + FPS / 延迟统计 |
| 8 | 评估脚本 + A/B 对照组实验记录 |

## v2.0 —— 融合感知

| 步骤 | 内容 |
|---|---|
| 9 | 增加深度摄像头或激光雷达 |
| 10 | 视觉与距离信息融合 + 坐标变换 |
| 11 | 接入 Nav2（可选） |
| 12 | 评估语义分割 / 目标检测是否值得引入 |

## 关键技术决策

1. v1.0 只用传统 OpenCV 基线，不引入深度学习 —— 先打通链路，可解释、易调试。
2. v1.0 不加深度 / 激光 —— 保持依赖最小，接受单目无真实距离的限制。
3. v1.0 先两包结构 —— 单节点串行便于定位问题，接口稳定后再拆。
