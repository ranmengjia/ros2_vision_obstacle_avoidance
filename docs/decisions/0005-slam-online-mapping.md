# ADR 0005：SLAM Toolbox 在线建图替代静态地图 + AMCL

- 状态：已接受（v2.0 步骤 12）
- 日期：2026-10-23

## 背景

ADR 0004 的静态地图方案有一个硬伤：每次改 world（挪动/增删障碍物）
都必须重新手绘 9m×9m、0.05m/px 的 pgm 地图，仿真迭代成本高。

## 决策

新增 `slam.launch.py`：用 SLAM Toolbox（online_async）实时建图，
Nav2 侧改用 `navigation_launch.py` 纯导航栈（不含 map_server / AMCL）：

- SLAM Toolbox 订阅 `/scan` + `odom->base_link` TF，发布 `map->odom` TF
  和 `/map` 占据栅格；Nav2 全局代价地图的 static_layer 直接订阅该 `/map`。
- 参数适配（`param/slam_toolbox.yaml`，基于官方 online_async 模板）：
  - `base_frame: base_link`（默认是 base_footprint，本项目没有）；
  - `max_laser_range: 8.0`（对齐雷达实际量程，默认 20.0 会把 inf 射线
    也拿去栅格化）；
  - `minimum_travel_distance/heading: 0.2/0.2`（默认 0.5 对 9×9m 小场地、
    0.2m/s 慢速机器人太稀疏）。
- 启动时序：SLAM 延迟 5s（等 /scan 与 TF），Nav2 延迟 8s（等 map->odom）。
- 静态地图方案（`nav2.launch.py`）保留，两套 launch 并存。

## 理由

- world 改动零地图成本，仿真场景迭代自由度大幅提升。
- SLAM Toolbox 是 Humble 生态成熟组件，自带回环检测与地图保存服务。

## 验证结果

- `map->odom` TF 正常，导航生命周期全部 active。
- (-2,0) 发 navigate_to_pose (2,0)，绕开中心方块成功，终点 (1.94, 0.11)
  在 0.15m 容差内。
- 实时建图覆盖全部四面围墙（±3.95m，与 world 精确对齐）及中心方块轮廓，
  用 map_saver_cli 保存并与静态地图比对确认。
