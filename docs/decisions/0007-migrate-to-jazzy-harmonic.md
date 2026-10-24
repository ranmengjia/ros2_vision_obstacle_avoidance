# ADR 0007：平台迁移到 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

- 状态：已接受（v2.1），替代 ADR 0001~0006 中的运行环境描述
- 日期：2026-10-24

## 背景

项目原先跑在 Ubuntu 22.04 + ROS 2 Humble + Gazebo classic 11 上。
开发机升级到 Ubuntu 24.04.5 LTS 后，Humble 不再有对应二进制包，
必须迁移到 Jazzy；而 Jazzy 生态默认的仿真器是 Gazebo Harmonic，
`gazebo_ros` 系列包在 Harmonic 下不可用。

## 决策

整体迁移到 ROS 2 Jazzy + Gazebo Harmonic 8.15，不做双版本兼容：

- 仿真启动改用 `ros_gz_sim`：`gz_sim.launch.py` 起 Gazebo，`ros_gz_sim create` 生成实体。
- 话题桥接改用 `ros_gz_bridge/parameter_bridge`，显式声明每个话题的类型和方向
  （`@` 分隔类型，`[` 为 Gazebo→ROS，`]` 为 ROS→Gazebo）。
- world 文件改用 `gz::sim::systems::*` 系统插件
  （physics / user_commands / scene_broadcaster / sensors / diff_drive）。
- URDF 传感器从"`<gazebo>` 里挂 gazebo_ros 插件"改成
  "`<gazebo reference>` 里写 `<sensor type="camera|gpu_lidar">`"，并用 `<gz_frame_id>` 指定坐标系。
- Nav2 参数适配 Jazzy：`collision_monitor`、`docking_server` 在 `navigation_launch.py` 中
  无条件启动，`polygons` 与 `dock_plugins` 必须给非空值。
- 迁移前的 Humble 版本地备份（`*.classic.bak`）不入库，仓库只保留 Jazzy 一套代码。

## 理由

- Humble 在 Ubuntu 24.04 上没有官方二进制，源码编译 ROS 2 的维护成本远高于迁移。
- `ros_gz` 是 Jazzy 官方推荐的 Gazebo 集成方式，也是后续 Gazebo 版本升级的唯一路径。
- 感知算法（OpenCV 阈值 + 雷达闸门）与 ROS 版本无关，迁移只动仿真和导航配置层，
  可以保住 v1.0~v2.0 的全部功能与验收结论。

## 踩过的坑（迁移时必看）

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2 launch` 报 `Package 'xxx' not found` | 终端没 source 工作区 `install/setup.bash` | 先 source `/opt/ros/jazzy` 再 source `install/setup.bash` |
| `gz sim` 一开 GUI 就段错误 | `--headless-rendering` 与当前 EGL 环境冲突 | headless 只加 `-s`（纯服务器），不加渲染参数 |
| `map_server` 报 `yaml-filename parameter is empty` | 节点专用段里的 `yaml_filename: ""` 覆盖了 launch 注入值 | 删掉 `nav2_params.yaml` 里那一行 |
| Nav2 起不来，报 polygons 为空 | Jazzy 的 `collision_monitor` 必填 `polygons` | 给出 `FootprintApproach` 多边形 |
| Nav2 起不来，报 dock_plugins 为空 | Jazzy 的 `docking_server` 必填 `dock_plugins` | 配置 `simple_charging_dock` |
| 机器人静止时缓慢漂移 | DiffDrive 在零速下仍受接触力影响 | spawn 后发 5s 零速 `init_cmd_vel` |

## 验证结果

- `colcon build` 通过；`/camera/camera_info` 15.1Hz、`/scan` 10.0Hz；
  `dim_course.world` / `low_contrast_course.world` 实测 0 条错误。
- `nav2.launch.py`：生命周期节点全部 active，(1.0, 0) 与 (2.0, 0) 均导航 `SUCCEEDED`。
- `slam.launch.py`：`/map` 正常发布，`map_saver_cli` 保存成功。
- `tools/eval/run_ab.sh` 与 `eval_session.py` 在 Jazzy 下可跑通。

## 已知限制

- 开发机用户不在 `video` / `render` 组，GUI 退化为 llvmpipe 软件渲染，功能正常但 CPU 占用高。
- `docking_server` 会同时发布 `/cmd_vel`，与 `collision_monitor` 并存，当前未关闭。
- 本 ADR 之前的决策记录仍按 Humble 时代的措辞描述环境，内容结论不受影响。
