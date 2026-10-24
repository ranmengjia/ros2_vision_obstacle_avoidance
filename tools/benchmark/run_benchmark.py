#!/usr/bin/env python3
"""对比基准调度器：按 scenarios.yaml 逐场景 × 逐臂 × 多次运行仿真并采集指标。

五条臂：
- vision / lidar / fused：先启动 Gazebo，再启动 perception_node（不同 sensor_mode）
- nav2：Gazebo + Nav2 全栈（AMCL 初始位姿按场景生成临时参数），由本脚本下发目标点
- hybrid：Nav2 输出改名为 /cmd_vel_raw，交给 vision_guard.py 用 OpenCV 做近距护栏

用法（需先 source install/setup.bash）：
    python3 tools/benchmark/run_benchmark.py --runs 1
    python3 tools/benchmark/run_benchmark.py --scenario detour_test --arm vision --runs 1
    python3 tools/benchmark/run_benchmark.py --scenarios tools/benchmark/scenarios.yaml

结果写入 tools/benchmark/results/<scenario>/<arm>/run_<i>.json，
原始进程日志写入 tools/benchmark/results/_logs/。
"""
import argparse
import glob
import json
import math
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
WORLDS_DIR = os.path.join(ROOT, 'src', 'robot_description', 'worlds')
NAV2_PARAMS = os.path.join(
    ROOT, 'src', 'robot_description', 'param', 'nav2_params.yaml')

sys.path.insert(0, HERE)
from metrics import (  # noqa: E402  （同目录纯函数模块）
    aggregate,
    angular_jerk,
    angular_reversals,
    classify_failure,
    collision_episodes,
    distance,
    estimate_cpu_time,
    first_reach_time,
    min_clearance,
    path_length,
    reached_goal,
    stopped_fraction,
    success_rate,
)

# 臂 → 启动方式
ARMS = {
    'vision': {'kind': 'reactive', 'sensor_mode': 'vision'},
    'lidar': {'kind': 'reactive', 'sensor_mode': 'lidar'},
    'fused': {'kind': 'reactive', 'sensor_mode': 'fused'},
    'nav2': {'kind': 'nav2'},
    'hybrid': {'kind': 'hybrid'},
}


# ---------------------------------------------------------------- 场景 / 参数

def load_scenarios(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    defaults = data.get('defaults', {})
    scenarios = []
    for raw in data['scenarios']:
        scn = dict(defaults)
        scn.update(raw)
        scenarios.append(scn)
    return scenarios


def build_nav2_params(scenario, out_path, cmd_vel_out='cmd_vel'):
    """复制 nav2 参数，仅覆盖 AMCL 初始位姿与 collision_monitor 输出话题。"""
    with open(NAV2_PARAMS, 'r', encoding='utf-8') as f:
        text = f.read()

    x, y, yaw = scenario['spawn']
    text = re.sub(r'(\n\s*initial_pose\.x:\s*)[-\d.]+', rf'\g<1>{x}', text)
    text = re.sub(r'(\n\s*initial_pose\.y:\s*)[-\d.]+', rf'\g<1>{y}', text)
    text = re.sub(r'(\n\s*initial_pose\.yaw:\s*)[-\d.]+', rf'\g<1>{yaw}', text)
    if cmd_vel_out != 'cmd_vel':
        text = re.sub(r'(\n\s*cmd_vel_out_topic:\s*)cmd_vel\b',
                      rf'\g<1>{cmd_vel_out}', text)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(text)
    return out_path


def scenario_map(scenario, out_dir):
    """按需从 world 生成静态地图，返回 yaml 路径（按 mtime 缓存）。"""
    from generate_maps import generate_for_world

    world_path = os.path.join(WORLDS_DIR, scenario['world'])
    stem = os.path.splitext(scenario['world'])[0]
    yaml_path = os.path.join(out_dir, f'{stem}_map.yaml')
    pgm_path = os.path.join(out_dir, f'{stem}_map.pgm')
    if (os.path.exists(yaml_path) and os.path.exists(pgm_path)
            and os.path.getmtime(yaml_path) >= os.path.getmtime(world_path)):
        return yaml_path
    return generate_for_world(world_path, out_dir)


def map_to_odom(point, spawn):
    """地图坐标 → odom 坐标。

    Gazebo 的 DiffDrive 插件用编码器积分里程计，/odom 以出生点为原点，
    而 nav2 目标点与地图都在世界系；两者只差一个刚体变换（出生位姿）。
    """
    dx = point[0] - spawn[0]
    dy = point[1] - spawn[1]
    yaw = spawn[2] if len(spawn) > 2 else 0.0
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    return (dx * cos_y - dy * sin_y, dx * sin_y + dy * cos_y)


def _vision_perception_path():
    hits = glob.glob(os.path.join(
        ROOT, 'install', 'vision_perception', 'lib',
        f'python3.{sys.version_info.minor}', 'site-packages'))
    return hits[0] if hits else None


# ---------------------------------------------------------------- 进程管理

def _spawn(cmd, log_path, env):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    handle = open(log_path, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        cmd, stdout=handle, stderr=subprocess.STDOUT,
        cwd=ROOT, env=env, start_new_session=True)
    proc._bench_log = handle
    return proc


def _stop(procs, grace=4.0):
    """先 SIGINT 让 launch/gz 正常收敛，再 SIGTERM/SIGKILL 兜底。"""
    for proc in procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        if all(p.poll() is not None for p in procs):
            break
        time.sleep(0.2)
    for proc in procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        if all(p.poll() is not None for p in procs):
            break
        time.sleep(0.2)
    for proc in procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        handle = getattr(proc, '_bench_log', None)
        if handle is not None:
            handle.close()


def _leftover_sim_pids():
    """列出残留的 gz sim 进程（只用于告警，不自动杀别人的实例）。"""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\x00', b' ').decode('utf-8', 'ignore')
        except OSError:
            continue
        if 'gz sim' in cmd:
            pids.append((int(entry), cmd.strip()))
    return pids


def _group_cpu_sec(procs):
    """按进程组累加 utime+stime（秒）。

    `resource.getrusage(RUSAGE_CHILDREN)` 只统计已回收的子进程，运行中的
    launch/gz/nav2 全部不计，所以这里直接读 /proc/<pid>/stat。`_spawn`
    用 start_new_session=True 让每个进程组组长 pid 即 pgid，组内所有后代
    （launch 拉起的节点、gz 子进程）都会继承该 pgid。
    """
    groups = set()
    for proc in procs:
        try:
            groups.add(os.getpgid(proc.pid))
        except (ProcessLookupError, PermissionError):
            continue

    clk = os.sysconf('SC_CLK_TCK')
    ticks = 0
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat', 'r', encoding='utf-8') as f:
                raw = f.read()
        except OSError:
            continue
        # comm 可能含空格/括号，先按最后一个 ')' 切掉；其后第 3 项是 pgrp，
        # 第 14/15 项（切分后下标 11/12）分别是 utime、stime。
        tail = raw[raw.rfind(')') + 2:].split()
        if len(tail) < 13:
            continue
        try:
            if int(tail[2]) not in groups:
                continue
            ticks += int(tail[11]) + int(tail[12])
        except ValueError:
            continue
    return ticks / clk


# ---------------------------------------------------------------- 采集节点

def make_recorder(goal, tolerance, use_action, map_goal=None,
                  direction=None, lane_tolerance=None):
    """goal 用 odom 坐标（判定到达），map_goal 用地图坐标（下发给 Nav2）。"""
    import rclpy
    import rclpy.action
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan

    class Recorder(Node):
        def __init__(self):
            super().__init__('bench_recorder')
            self.goal = goal
            self.map_goal = map_goal if map_goal is not None else goal
            self.tolerance = tolerance
            self.direction = direction
            self.lane_tolerance = lane_tolerance
            self.use_action = use_action

            self.has_odom = False
            self.has_scan = False
            self.reached = False
            self.t0 = None
            self.t_last = 0.0
            self.poses = []
            self.times = []
            self.min_ranges = []
            self.cmd_vs = []
            self.cmd_ws = []
            self.cmd_times = []

            self.create_subscription(Odometry, '/odom', self._odom, 20)
            self.create_subscription(LaserScan, '/scan', self._scan, 10)
            self.create_subscription(Twist, '/cmd_vel', self._cmd, 20)

            self.action_client = None
            if use_action:
                from nav2_msgs.action import NavigateToPose
                self.NavigateToPose = NavigateToPose
                self.action_client = rclpy.action.ActionClient(
                    self, NavigateToPose, 'navigate_to_pose')

        # ---- 回调
        def _stamp(self, msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def _odom(self, msg):
            self.has_odom = True
            t = self._stamp(msg)
            if self.t0 is None:
                self.t0 = t
            if t < self.t0:
                return
            p = msg.pose.pose.position
            self.t_last = t - self.t0
            self.poses.append((p.x, p.y))
            self.times.append(self.t_last)
            if reached_goal((p.x, p.y), self.goal, self.tolerance,
                            self.direction, self.lane_tolerance):
                self.reached = True

        def _scan(self, msg):
            self.has_scan = True
            valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
            self.min_ranges.append(round(min(valid), 3) if valid else math.inf)

        def _cmd(self, msg):
            self.cmd_vs.append(msg.linear.x)
            self.cmd_ws.append(msg.angular.z)
            self.cmd_times.append(time.monotonic())

        # ---- 控制
        def start_run(self):
            self.poses.clear()
            self.times.clear()
            self.min_ranges.clear()
            self.cmd_vs.clear()
            self.cmd_ws.clear()
            self.cmd_times.clear()
            self.reached = False
            self.t0 = None
            self.t_last = 0.0

        def send_goal(self, executor, timeout=90.0, retry_period=2.0):
            """等待 Nav2 action server 就绪并下发目标；返回是否成功接受。

            bt_navigator 的 action server 在生命周期激活前后都会对外可见，
            未激活时调用会被拒绝，所以这里按 retry_period 重试直到超时。
            """
            if self.action_client is None:
                return False
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if not self.action_client.server_is_ready():
                    executor.spin_once(timeout_sec=0.1)
                    continue

                goal = self.NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = float(self.map_goal[0])
                goal.pose.pose.position.y = float(self.map_goal[1])
                goal.pose.pose.orientation.w = 1.0

                future = self.action_client.send_goal_async(goal)
                while not future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                handle = future.result() if future.done() else None
                if handle is not None and handle.accepted:
                    return True

                wait_until = time.monotonic() + retry_period
                while time.monotonic() < wait_until:
                    executor.spin_once(timeout_sec=0.1)

            self.get_logger().error('Nav2 未在超时内接受目标点')
            return False

    return Recorder


def _collect(node, goal, tolerance, wall_sec, cpu_sec,
             direction=None, lane_tolerance=None):
    """把采样序列交给 metrics 纯函数，返回一条结果记录。"""
    scans = [[m] for m in node.min_ranges]
    reach = first_reach_time(node.times, node.poses, goal, tolerance,
                             direction, lane_tolerance)
    success = reach is not None
    traveled = path_length(node.poses)
    clearance = min_clearance(scans)
    collisions = collision_episodes(scans)
    stopped = stopped_fraction(node.cmd_vs)
    reversals = angular_reversals(node.cmd_ws)

    if len(node.cmd_times) >= 3:
        deltas = [b - a for a, b in zip(node.cmd_times, node.cmd_times[1:])]
        dt = statistics.median(deltas)
    else:
        dt = 0.1
    jerk = angular_jerk(node.cmd_ws, dt if dt > 0 else 0.1)

    final_distance = (distance(node.poses[-1], goal) if node.poses else math.inf)
    return {
        'success': success,
        'reach_time_sec': reach,
        'traveled_m': round(traveled, 3),
        'final_distance_m': round(final_distance, 3),
        'final_pose': ([round(v, 3) for v in node.poses[-1]]
                       if node.poses else None),
        'min_clearance_m': (None if math.isinf(clearance) else round(clearance, 3)),
        'collision_count': collisions,
        'stopped_fraction': round(stopped, 3),
        'angular_reversals': reversals,
        'angular_jerk': round(jerk, 3),
        'sim_duration_sec': round(node.t_last, 2),
        'wall_duration_sec': round(wall_sec, 2),
        'cpu_time_ratio': round(estimate_cpu_time(wall_sec, cpu_sec), 3),
        'samples': {
            'poses': len(node.poses),
            'scans': len(node.min_ranges),
            'cmds': len(node.cmd_vs),
        },
        'failure': classify_failure(
            success, collisions, stopped, traveled, clearance),
    }


# ---------------------------------------------------------------- 单次运行

def run_once(scenario, arm, run_idx, out_dir, args, env):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    kind = ARMS[arm]['kind']
    sid = scenario['id']
    run_dir = os.path.join(out_dir, '_logs', sid, arm, f'run_{run_idx}')
    os.makedirs(run_dir, exist_ok=True)

    nav2_params = None
    if kind in ('nav2', 'hybrid'):
        cmd_vel_out = 'cmd_vel_raw' if kind == 'hybrid' else 'cmd_vel'
        nav2_params = build_nav2_params(
            scenario, os.path.join(run_dir, 'nav2_params.yaml'), cmd_vel_out)

    procs = []
    rclpy.init(args=[])
    goal_odom = map_to_odom(scenario['goal'], scenario['spawn'])
    norm = math.hypot(*goal_odom)
    direction = ((goal_odom[0] / norm, goal_odom[1] / norm)
                 if norm > 1e-6 else (1.0, 0.0))
    lane_tol = scenario.get('lane_tolerance')
    Recorder = make_recorder(
        goal_odom, scenario['goal_tolerance'],
        kind in ('nav2', 'hybrid'), map_goal=scenario['goal'],
        direction=direction, lane_tolerance=lane_tol)
    node = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    wall0 = time.monotonic()
    try:
        spawn = scenario['spawn']
        common = [
            'world_name:=' + scenario['world'],
            f'spawn_x:={spawn[0]}', f'spawn_y:={spawn[1]}',
            f'spawn_yaw:={spawn[2]}',
            f'headless:={"true" if args.headless else "false"}',
        ]

        if kind == 'reactive':
            # 先只起仿真；控制器等稳定期过后再起，否则机器人在 settle 期间
            # 就会开始移动，导致 traveled_m 从出生点算起会少一段。
            procs.append(_spawn(
                ['ros2', 'launch', 'robot_description', 'simulation.launch.py']
                + common,
                os.path.join(run_dir, 'sim.log'), env))
        else:
            procs.append(_spawn(
                ['ros2', 'launch', 'robot_description', 'nav2.launch.py']
                + common + [
                    'use_rviz:=false',
                    'map:=' + scenario_map(scenario, args.maps_dir),
                    'nav2_params_file:=' + nav2_params,
                ],
                os.path.join(run_dir, 'nav2.log'), env))
            if kind == 'hybrid':
                guard_env = dict(env)
                vp = _vision_perception_path()
                if vp:
                    guard_env['PYTHONPATH'] = (
                        vp + os.pathsep + guard_env.get('PYTHONPATH', ''))
                procs.append(_spawn(
                    [sys.executable, os.path.join(HERE, 'vision_guard.py'),
                     '--ros-args', '-p', f'guard_threshold:={args.guard_threshold}'],
                    os.path.join(run_dir, 'guard.log'), guard_env))

        # ---- 等传感器就绪
        ready_deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < ready_deadline:
            if node.has_odom and node.has_scan:
                break
            if any(p.poll() is not None for p in procs):
                node.get_logger().error('启动阶段有进程提前退出，见 _logs 目录')
                break
            executor.spin_once(timeout_sec=0.1)

        # ---- 稳定期：让 AMCL 收敛 / 机器人静止
        settle_deadline = time.monotonic() + scenario.get('settle_sec', 4.0)
        while time.monotonic() < settle_deadline:
            executor.spin_once(timeout_sec=0.1)

        node.start_run()
        if kind == 'reactive':
            procs.append(_spawn(
                ['ros2', 'launch', 'vision_perception', 'perception.launch.py',
                 'control_mode:=' + scenario.get('control_mode', 'reactive'),
                 'sensor_mode:=' + ARMS[arm]['sensor_mode']],
                os.path.join(run_dir, 'controller.log'), env))
        elif kind in ('nav2', 'hybrid'):
            if node.send_goal(executor, timeout=args.startup_timeout):
                # 以目标被接受为计时起点，避免把 Nav2 生命周期激活时间算进到达时间
                node.start_run()
            else:
                node.get_logger().error('目标点下发失败，本次按超时处理')

        # ---- 运行直到到达或超时
        deadline = time.monotonic() + scenario.get('timeout_sec', 90.0)
        while time.monotonic() < deadline and not node.reached:
            executor.spin_once(timeout_sec=0.05)
            if all(p.poll() is not None for p in procs):
                node.get_logger().error('运行阶段所有进程已退出')
                break

        wall_sec = time.monotonic() - wall0
        cpu_sec = _group_cpu_sec(procs)
        record = _collect(node, goal_odom,
                          scenario['goal_tolerance'], wall_sec, cpu_sec,
                          direction, lane_tol)
    finally:
        _stop(procs)
        executor.remove_node(node)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    record.update({
        'scenario': sid,
        'arm': arm,
        'run': run_idx,
        'kind': kind,
        'goal': scenario['goal'],
        'goal_odom': [round(v, 3) for v in goal_odom],
        'goal_tolerance': scenario['goal_tolerance'],
        'lane_tolerance': lane_tol,
        'spawn': scenario['spawn'],
        'world': scenario['world'],
    })
    result_dir = os.path.join(out_dir, sid, arm)
    os.makedirs(result_dir, exist_ok=True)
    result_path = os.path.join(result_dir, f'run_{run_idx}.json')
    with open(result_path, 'w', encoding='utf-8') as f:
        json.dump(record, f, ensure_ascii=False, indent=2)

    status = ('到达 %.1fs' % record['reach_time_sec']) if record['success'] \
        else '失败(%s)' % record['failure']
    print(f'[{sid}/{arm} #{run_idx}] {status} '
          f'路程 {record["traveled_m"]}m 碰撞 {record["collision_count"]} '
          f'最小间距 {record["min_clearance_m"]}m', flush=True)
    return record


# ---------------------------------------------------------------- 主流程

def _merge(results, keys):
    """同一 (scenario, arm) 的多次运行聚合。"""
    grouped = {}
    for r in results:
        grouped.setdefault((r['scenario'], r['arm']), []).append(r)
    summary = []
    for (sid, arm), records in grouped.items():
        item = {
            'scenario': sid,
            'arm': arm,
            'runs': len(records),
            'success_rate': round(success_rate(records), 3),
            'metrics': aggregate(records, keys),
            'failures': [r['failure'] for r in records if r['failure']],
        }
        summary.append(item)
    return summary


def _load_all(out_dir):
    """读取 out_dir 下已有全部 run_*.json。

    基准经常按场景/臂分批补跑，summary 必须是全量聚合，而不是只含本批次。
    """
    records = []
    for path in sorted(glob.glob(os.path.join(out_dir, '*', '*', 'run_*.json'))):
        with open(path, 'r', encoding='utf-8') as f:
            records.append(json.load(f))
    return records


def main():
    parser = argparse.ArgumentParser(description='OpenCV vs Nav2 避障对比基准')
    parser.add_argument('--scenarios', default=os.path.join(HERE, 'scenarios.yaml'))
    parser.add_argument('--scenario', action='append', default=None,
                        help='只跑指定场景 id（可多次指定）')
    parser.add_argument('--arm', action='append', default=None,
                        choices=sorted(ARMS), help='只跑指定臂（可多次指定）')
    parser.add_argument('--runs', type=int, default=3, help='每个组合重复次数')
    parser.add_argument('--out', default=os.path.join(HERE, 'results'))
    parser.add_argument('--maps-dir', default=os.path.join(HERE, 'maps'))
    parser.add_argument('--startup-timeout', type=float, default=120.0)
    parser.add_argument('--guard-threshold', type=float, default=0.3)
    parser.add_argument('--no-headless', dest='headless', action='store_false',
                        help='显示 Gazebo GUI（默认 headless）')
    args = parser.parse_args()

    if shutil.which('ros2') is None:
        parser.error('找不到 ros2，请先 source /opt/ros/jazzy/setup.bash '
                     '和 install/setup.bash')

    leftovers = _leftover_sim_pids()
    if leftovers:
        print('警告：检测到已存在的 gz sim 进程，它们会与本基准抢占端口：')
        for pid, cmd in leftovers:
            print(f'  {pid}  {cmd[:100]}')
        print('请先结束这些进程再运行。\n')

    scenarios = load_scenarios(args.scenarios)
    if args.scenario:
        scenarios = [s for s in scenarios if s['id'] in args.scenario]
    if not scenarios:
        parser.error('没有匹配的场景')

    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '43'))
    print(f'ROS_DOMAIN_ID={env["ROS_DOMAIN_ID"]}  结果目录 {args.out}')

    results = []
    for scenario in scenarios:
        arms = [a for a in scenario.get('arms', []) if ARMS.get(a)]
        if args.arm:
            arms = [a for a in arms if a in args.arm]
        for arm in arms:
            for i in range(1, args.runs + 1):
                try:
                    results.append(run_once(
                        scenario, arm, i, args.out, args, env))
                except Exception as exc:  # 单次失败不应中断整个矩阵
                    print(f'[{scenario["id"]}/{arm} #{i}] 异常: {exc}',
                          file=sys.stderr, flush=True)
                    time.sleep(2.0)

    keys = ['reach_time_sec', 'traveled_m', 'final_distance_m',
            'min_clearance_m', 'collision_count', 'stopped_fraction',
            'angular_reversals', 'angular_jerk', 'sim_duration_sec',
            'cpu_time_ratio']
    summary = _merge(_load_all(args.out), keys)
    summary_path = os.path.join(args.out, 'summary.json')
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f'\n完成 {len(results)} 次运行，汇总（全部 {len(_load_all(args.out))} 条）'
          f'写入 {summary_path}')


def shutil_which(name):
    from shutil import which
    return which(name)


if __name__ == '__main__':
    main()
