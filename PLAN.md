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
| 操作系统 | Ubuntu 24.04.5 LTS |
| ROS 发行版 | ROS 2 Jazzy |
| 仿真器 | Gazebo Harmonic 8.15.0（`gz sim`） |
| 图像处理 | Python 3.12 + OpenCV 4.6.0 + NumPy 1.26.4 |
| 桥接 | cv_bridge + ros_gz_bridge |
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

| 步骤 | 内容 | 分支 | 状态 |
|---|---|---|---|
| 9 | 2D 激光雷达 `/scan`（360 点 × 10Hz × ±8.0m） | feat/lidar-sensor | ✅ 完成 |
| 10 | 视觉 + 雷达融合（安全闸门 + 双看门狗） | feat/sensor-fusion | ✅ 完成 |
| 11 | 接入 Nav2（静态地图 + AMCL + DWB 导航） | — | ✅ 完成 |
| 12 | SLAM 在线建图（SLAM Toolbox 替代静态地图 + AMCL） | feat/slam-mapping | ✅ 完成 |
| 13 | 评估语义分割 / 目标检测是否值得引入 | — | ✅ 完成（结论：暂不引入） |

### v2.0 功能边界

- 做：2D ray sensor（360° 全向，10Hz）→ `/scan` 话题
- 做：雷达前方 60° 三扇区最小距离映射到视觉三分区（±30°，均分三份）
- 做：安全闸门融合策略——某扇区 min_distance < safety_distance(0.3m) → 该方向强制不可通行
- 做：双看门狗（图像超时 1s 或雷达超时 1s 均触发安全停车）
- 不做：深度摄像头、3D 点云
- 不做：Nav2（可选，依赖重，先验证融合避障效果）

### v2.0 接口

| 话题 | 方向 | 类型 |
|---|---|---|
| /camera/image_raw | 仿真 → 感知 | sensor_msgs/Image |
| /scan | 仿真 → 感知 | sensor_msgs/LaserScan |
| /cmd_vel | 控制 → 底盘 | geometry_msgs/Twist |

### v2.0 新增纯函数

| 函数 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `sector_min_distances(ranges, angle_min, angle_increment, fov_deg)` | LaserScan 的参数 | (左, 中, 右) 最小距离 | 360° 全向 → 前方 60° 三扇区 |
| `fuse_scores(zone_scores, sector_dists, safety_distance)` | 视觉三分 + 雷达三分 + 安全距离 | (左, 中, 右) 融合后 | 雷达闸门：< 0.5m → 0 |

### v2.0 绕行验证（detour_test.world，已完成）

- 障碍物 `detour_box`（0.5m 方块）置于直行路径正中 `(0.5, 0)`。
- 验收（2026-10-23）：从 (-2,0) 出发绕行方块抵达东侧 (3.7, 0)，全程零碰撞，
  车体与方块最小间隔 0.10m；50 条单测全绿。
- 关键教训：相机 0.08m 高 + ROI 底部 40% → 视觉盲区约 0.34m，
  雷达闸门 `safety_distance` 必须大于盲区（取 0.5m）；
  仅"中路畅通就直行"会擦障卡死，需 `side_guard_distance`(0.25m) 侧身护卫。

### v2.0 步骤 11：Nav2 接入（已完成）

| 文件 | 作用 |
|---|---|
| `src/robot_description/param/nav2_params.yaml` | AMCL / Navfn / DWB / 代价地图参数；坐标系 `base_link`，footprint `0.32×0.26`，限速 0.2m/s、0.8rad/s |
| `src/robot_description/maps/detour_test_map.yaml` + `.pgm` | 与 `detour_test.world` 对齐的 9m×9m、0.05m/px 静态地图 |
| `src/robot_description/launch/nav2.launch.py` | Gazebo + Nav2 bringup + RViz 一键启动；Nav2 延迟 5s 等 TF/scan 就绪 |

启动与验证：

```bash
ros2 launch robot_description nav2.launch.py            # 带 RViz，用 2D Goal Pose 发目标
ros2 launch robot_description nav2.launch.py use_rviz:=false
```

- 验收（2026-10-23）：生命周期节点全部 active；从 (-2,0) 导航到 (2,0) 再返回，绕过中心 0.5m 方块，实测最小车体间隙 0.186m，无碰撞。
- 注意：Nav2 与 `vision_perception` 都会发布 `/cmd_vel`，两个 launch 不能同时启动。
- 已知：差速底盘静止漂移约 5.5mm/s，AMCL 会持续修正；静止等待越久，初始位姿误差越大。

### v2.0 步骤 12：SLAM 在线建图（已完成）

| 文件 | 作用 |
|---|---|
| `src/robot_description/param/slam_toolbox.yaml` | SLAM Toolbox online_async 参数；坐标系 `base_link`/`odom`/`map`，量程对齐雷达 8.0m，建图触发距离/转角收紧到 0.2m / 0.2rad |
| `src/robot_description/launch/slam.launch.py` | Gazebo + SLAM Toolbox（延迟 5s）+ Nav2 纯导航栈 navigation_launch.py（延迟 8s）+ RViz 一键启动 |

启动与验证：

```bash
ros2 launch robot_description slam.launch.py            # 带 RViz，用 2D Goal Pose 发目标
ros2 launch robot_description slam.launch.py use_rviz:=false
```

- 与步骤 11 的区别：不再加载静态地图和 AMCL，`map->odom` TF 与 `/map` 栅格图由
  SLAM Toolbox 实时发布，world 改动后无需重画地图。
- 验收（2026-10-23）：导航生命周期全部 active；从 (-2,0) 发 navigate_to_pose (2,0)，
  绕开中心方块成功到达（终点 (1.94, 0.11)，容差 0.15m）；SLAM 建图覆盖全部四面围墙
  （±3.95m，与 world 精确对齐）及中心方块轮廓。
- 保存地图：`ros2 run nav2_map_server map_saver_cli -f <路径> -t /map`，
  或 SLAM Toolbox 自带的 `/slam_toolbox/save_map` 服务。

### v2.0 步骤 13：语义分割 / 目标检测引入评估（已完成，结论：暂不引入）

**现有方案已覆盖的能力**：亮度阈值+形态学（视觉可通行区域）+ 2D 雷达安全闸门
（几何障碍兜底）+ Nav2/SLAM（全局导航）。三场景 A/B 与绕行验证均零碰撞。

**现有方案的真实短板（DL 能补多少）**：

| 短板 | DL 能否解决 |
|---|---|
| 视觉基线对光照/地面配色敏感 | 语义分割理论上更鲁棒，但 Gazebo classic 纹理简单，sim 内训练/评估域差距大，结论难迁移真实场景 |
| 无法区分障碍类别（人 vs 箱子应采取不同策略） | 目标检测可补，属于"增强"而非"必需" |
| 雷达在 0.15m 高度，看不到悬空障碍（桌沿、横杆） | **补不了**——这需要深度相机或 3D 雷达，不是 2D 语义问题 |

**硬件与依赖成本（2026-10-23 实测本机）**：无独立 GPU（Intel UHD 核显），
torch / onnxruntime / tensorflow 均未安装；YOLOv8n 级模型 CPU 推理约
100~200ms/帧，会把感知从 15Hz 拖到 5~10Hz（当前单帧处理仅 0.3ms），
并引入 ~800MB（torch CPU）级依赖。

**结论**：
1. 语义分割——不引入。主要收益（光照鲁棒性）在 sim 内无法有效验证，违背"可解释、易调试"基线。
2. 目标检测——可作为 v3.0 可选增强层（如 YOLOv8n + onnxruntime CPU，识别"人"做减速/让行），
   但只叠加在现有融合基线之上，不替换。
3. 若继续投入感知硬件方向，**深度相机**（补悬空障碍 + 真实深度）比 DL 收益更大、
   更贴合"融合感知"主线。

## v2.2 —— 迷宫自主脱困（纯视觉 + 雷达，已完成）

| 步骤 | 分支 | 内容 | 验收 |
|---|---|---|---|
| 14 | feat/maze-escape | 迷宫 world + `control_mode:=maze`：贴右墙跟随，靠左右大扇区开阔度选转向，三向开阔判出口 | 无界面仿真中从起点自主走到出口并停车，无 Nav2、无目标点 |

### v2.2 迷宫方案

- `src/robot_description/worlds/maze_course.world`：DFS 递归回溯生成的 5×5 完美迷宫，
  走廊 1.2m、墙厚 0.1m、墙高 1.0m；外墙 x/y ∈ [-3, 3]，仅东侧 `(3.0, -1.2)` 留缺口。
- 出生点 `(0, 0, 0)`（格 2,2）；`simulation.launch.py` 新增
  `spawn_x` / `spawn_y` / `spawn_yaw`，默认值保持原 `(-2, 0, 0)`。
- 决策（`maze_command`，纯函数）只用三类输入：
  - 正前 40° 最小距离 → 是否需要转向；
  - 左前/右前 45°±5° 最小距离 → 沿右墙保持 `maze_target_gap` 的比例修正；
  - 左右各 125° 扇区的**最远**距离 → 两边都堵时朝更开阔的一侧转；
  - 视觉中路可通行率 < 0.5 或前方 < 1.0m 时降速到 0.10m/s。
- 转向滞回：`maze_front_stop` 0.6m 进入原地转向，`maze_front_clear` 1.0m 才退出，
  消除"转一下就往前蹭"的墙角死循环。
- 出口判定：前/左/右同时 > `maze_exit_distance` 4.0m 且保持 0.5s；
  迷宫内部最大净空 6m，不会误判。雷达"整帧 inf"用粘性标志 `lidar_seen_hit`
  区分"刚启动无回波"与"真开阔"，早退时发布零速度防止沿用上一帧速度盲走。

### v2.2 验收（2026-10-24 实测）

- `colcon build --packages-select vision_perception` 通过；76 条单测全绿。
- `perception.launch.py control_mode:=maze` + `simulation.launch.py
  world_name:=maze_course.world spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0`（无界面）：
  154s 后打印 `已找到出口：前=infm 左=infm 右=infm，停车`，
  终点位姿 `[2.821, -1.590] yaw=0.331`，停在东侧缺口门口，全程无 Nav2、无目标点。
- 关键教训：只"正前 < 0.6m 就转"没有滞回会在墙角抖到卡死；必须等前方
  真正清空（1.0m）再恢复直行。

## v3.0 —— OpenCV 反应式 vs Nav2 基准（已完成）

| 步骤 | 分支 | 内容 | 验收 |
|---|---|---|---|
| 15 | feat/benchmark-suite | 5 场景 × 5 臂自动基准：`run_benchmark.py` 起停 Gazebo/Nav2、采集指标，`report.py` 出表出图 | 全量一次跑通，`results/report.md` 与 `chart.png` 自动生成 |

### v3.0 对比设计

- 场景矩阵（`tools/benchmark/scenarios.yaml`）：
  - `detour_test`：起步就要求横移到缺口再前进，测"能不能绕"；
  - `obstacle_course`：直线障碍，测"能不能避"；
  - `low_contrast_course` / `dim_course`：弱光与低对比度，测视觉退化；
  - `maze_course`：5×5 迷宫，测"只有局部观测时的长程脱困"。
- 五条臂（`--arm`）：
  - `vision`（纯 OpenCV）、`lidar`（纯雷达）、`fused`（视觉+雷达，默认）
    —— 三条反应式臂共用 `perception_node` 的 `sensor_mode` 消融；
  - `nav2`：先验地图 + AMCL + SmacPlanner2D + RPP，由脚本用 `NavigateToPose` 下发目标；
  - `hybrid`：Nav2 的 `collision_monitor` 输出改名 `cmd_vel_raw`，`vision_guard.py`
    订阅相机、正前方过近时覆盖为降速/转向，验证"规划 + 视觉兜底"。
- 指标（`tools/benchmark/metrics.py`，38 条单测）：成功率、到达时间、路程、
  最小雷达间距、碰撞次数、卡死占比、角速度 jerk、CPU 占比、失败归因。
- 到达判定：Nav2 臂用终点容差圆；反应式臂不收敛到点，额外接受
  "越过终点线且横向偏差 ≤ `lane_tolerance`"，否则直行通过的场景会被误判超时。

### v3.0 验收（2026-10-24 实测，每格 1 次）

| 场景 | 纯 OpenCV | 纯雷达 | 视觉+雷达 | Nav2 | Nav2+视觉护栏 |
|---|---|---|---|---|---|
| detour_test | 失败（碰撞 2） | 54.6s / 9.53m | 62.3s / 9.93m | 23.1s / 4.48m | 23.3s / 4.52m |
| obstacle_course | 25.9s / 4.90m | 25.8s / 4.90m | 26.0s / 4.90m | 25.1s / 4.93m | 25.2s / 4.94m |
| low_contrast_course | 25.8s / 4.90m | — | 25.9s / 4.90m | 25.0s / 4.92m | — |
| dim_course | 26.0s / 4.90m | — | 26.0s / 4.90m | 25.1s / 4.93m | — |
| maze_course | — | 154.2s / 22.18m | 153.9s / 22.21m | 92.7s / 17.71m | 92.1s / 17.67m |

- 纯 OpenCV 在 `detour_test` 因连续 2 帧 < 0.13m 回波判碰撞失败；其余结构化场景
  与 Nav2 打平（约 26s），说明"直线/单障碍"类问题反应式已足够。
- 迷宫差距最大：反应式贴墙 22m，Nav2 用先验地图 17.7m，时间少 40%。
- `hybrid` 与 `nav2` 用时基本一致（共同场景均值 46.9s vs 47.0s），
  正常导航时视觉护栏不介入，只在危险距离兜底；CPU 占比略高（3.21 vs 2.99）。
- 见 `tools/benchmark/results/report.md` 与 `chart.png`。

### v3.0 踩坑与修正

1. **NavFn 有路径长度上限**：`NavFn::calcPath` 每次迭代前进 0.5 栅格，
   `max_cycles = max(nx, ny) * 4`，即上限约为地图宽度的 2 倍。迷宫地图宽 7.1m，
   上限 14.2m，而实际路线 22m → 规划恒返回 0，日志为
   `Failed to create a plan from potential when a legal potential was found.`。
   换 `nav2_smac_planner::SmacPlanner2D` 后解决。
2. **DWB 在 1.1m 窄走廊失效**：表现为原地反复微转（`angular_reversals` 32、
   `stopped_fraction` 0.84）、反复 `Failed to make progress` 后触发恢复行为。
   换 `RegulatedPurePursuitController`（`use_rotate_to_heading: true`，拐角先原地对正
   再前进）后迷宫成功。
3. **Nav2 生命周期偶发激活超时**：同一 ROS_DOMAIN_ID 下残留旧 Gazebo/Nav2 进程会造成
   TF 串扰（`Invalid frame ID "odom"`）、目标被拒。基准逐次起停，跑前需确认无残留
   `gz sim` / `nav2_*` 进程。

## 已知限制

- 本机用户不在 `video` / `render` 组，Gazebo GUI 退化为 llvmpipe 软件渲染（约 122% CPU），
  能正常显示但吃 CPU；`sudo usermod -aG video,render $USER` 后重新登录可解决。
- `navigation_launch.py` 无条件拉起的 `docking_server` 也会发布 `/cmd_vel`，
  与 `collision_monitor` 并存；当前不影响导航，暂未关闭。
- SLAM 模式下地图边跑边建，`map` 坐标系下的目标不要超出已建图范围，否则容易被未知区域卡住。

## 关键技术决策

1. v1.0 只用传统 OpenCV 基线，不引入深度学习 —— 先打通链路，可解释、易调试。
2. v1.0 不加深度 / 激光 —— 保持依赖最小，接受单目无真实距离的限制。
3. v1.0 先两包结构 —— 单节点串行便于定位问题，接口稳定后再拆。
4. v2.0 评估后不引入语义分割/目标检测 —— sim 域差距大、无 GPU、收益不匹配；
   下一步感知升级优先深度相机（详见步骤 13 评估）。
5. v2.2 迷宫脱困不引入 Nav2 / 全局规划 —— 目的是验证"纯反应式视觉 + 雷达"的
   边界能力；贴墙跟随 + 开阔度选边已足够，且无需地图与定位。
6. v3.0 用同一套指标横向比较反应式与 Nav2 —— 不追求"谁更好"，而是划清边界：
   结构化短路线两者都能过，差距出现在需要全局信息的绕行与迷宫中。
7. v3.0 全局规划器用 SmacPlanner2D 而非默认 NavFn —— NavFn 的路径长度上限约为
   地图宽度 2 倍，长路线场景会静默返回空路径；局部控制器用 RPP 而非 DWB ——
   1.1m 窄走廊下 DWB 的弧线采样找不到可行解，会退化成原地微转。
8. v3.0 把 hybrid 做成"Nav2 + 近距视觉护栏"而不是端到端融合 —— 保持 Nav2 的
   规划能力不变，只在安全层叠加视觉，便于单独度量护栏的收益与开销。
