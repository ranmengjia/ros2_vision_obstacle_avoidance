"""基准指标计算：纯函数，不依赖 ROS，可直接 pytest。

输入均为 Python 原生序列（由 run_benchmark.py 从 ROS 话题采样后传入）：
- poses:   [(x, y), ...]               里程计位姿（按时间顺序）
- scans:   [[r, r, ...], ...]          每帧 LaserScan 的 ranges
- cmd_ws:  [w, w, ...]                 每帧 /cmd_vel 的角速度
- cmd_vs:  [v, v, ...]                 每帧 /cmd_vel 的线速度
"""
import math


def path_length(points):
    """折线累计长度（米）。"""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def distance(p, q):
    """两点欧氏距离。"""
    return math.hypot(q[0] - p[0], q[1] - p[1])


def min_clearance(scans):
    """所有扫描帧中的最小有效回波距离（米）；无有效回波返回 inf。"""
    best = math.inf
    for frame in scans:
        for r in frame:
            if math.isfinite(r) and 0.0 < r < best:
                best = r
    return best


def collision_episodes(scans, threshold=0.13, min_consecutive=2):
    """碰撞次数：连续 min_consecutive 帧出现回波 < threshold 记一次。

    车体半宽 0.10m、轴距方向半长 0.15m，雷达在车体中心；
    0.13m 意味着障碍已经进入车体外廓附近，属于真实碰撞/擦碰。
    """
    episodes = 0
    streak = 0
    for frame in scans:
        hit = any(math.isfinite(r) and 0.0 < r < threshold for r in frame)
        streak = streak + 1 if hit else 0
        if streak == min_consecutive:
            episodes += 1
    return episodes


def stopped_fraction(linear_speeds, eps=0.01):
    """线速度近似为 0（< eps）的采样占比，用于衡量卡死程度。"""
    if not linear_speeds:
        return 0.0
    count = sum(1 for v in linear_speeds if abs(v) < eps)
    return count / len(linear_speeds)


def angular_reversals(angulars, eps=0.05):
    """角速度方向反转次数（排除 |w| < eps 的抖动噪声）。"""
    reversals = 0
    last_sign = 0
    for w in angulars:
        if abs(w) < eps:
            continue
        sign = 1 if w > 0 else -1
        if last_sign != 0 and sign != last_sign:
            reversals += 1
        last_sign = sign
    return reversals


def angular_jerk(angulars, dt=0.1):
    """角速度二阶差分的均方根（rad/s³），衡量控制平滑度。"""
    n = len(angulars)
    if n < 3 or dt <= 0.0:
        return 0.0
    accels = [(angulars[i + 1] - angulars[i]) / dt for i in range(n - 1)]
    jerks = [(accels[i + 1] - accels[i]) / dt for i in range(len(accels) - 1)]
    return math.sqrt(sum(j * j for j in jerks) / len(jerks))


def crossed_finish(point, goal, tolerance=0.0, direction=(1.0, 0.0),
                   lane_tolerance=None):
    """是否越过终点半平面。

    反应式臂不收敛到目标点，只会沿走廊继续前进；只要沿行进方向越过终点
    （允许 tolerance 的提前量）且横向没跑偏（|lateral| ≤ lane_tolerance）
    就算通过。direction 为单位向量，指向行进方向。
    """
    ux, uy = direction
    dx, dy = point[0] - goal[0], point[1] - goal[1]
    along = dx * ux + dy * uy
    lateral = -dx * uy + dy * ux
    if along < -tolerance:
        return False
    return lane_tolerance is None or abs(lateral) <= lane_tolerance


def reached_goal(point, goal, tolerance, direction=None, lane_tolerance=None):
    """进入终点容差圆，或越过终点线且横向在容差带内。"""
    if distance(point, goal) <= tolerance:
        return True
    if direction is None or lane_tolerance is None:
        return False
    return crossed_finish(point, goal, tolerance, direction, lane_tolerance)


def first_reach_time(times, points, goal, tolerance, direction=None,
                     lane_tolerance=None):
    """首次判为到达的时间（秒）；未到达返回 None。"""
    for t, p in zip(times, points):
        if reached_goal(p, goal, tolerance, direction, lane_tolerance):
            return t
    return None


def estimate_cpu_time(wall_sec, cpu_sec):
    """CPU 时间占比（多核累计 / 墙钟），用于粗略衡量算法开销。"""
    if wall_sec <= 0.0:
        return 0.0
    return cpu_sec / wall_sec


def classify_failure(success, collision_count, stopped_frac, traveled,
                     clearance=math.inf, collision_threshold=0.13):
    """失败归因：success 时返回 None。"""
    if success:
        return None
    if collision_count > 0 and clearance < collision_threshold:
        return 'collision'
    if stopped_frac >= 0.5:
        return 'stuck'
    if traveled < 0.5:
        return 'no_motion'
    return 'timeout'


def aggregate(records, keys):
    """把多次重复实验聚合成 mean/std/min/max；缺失值按 None 处理。"""
    result = {}
    for key in keys:
        values = [r[key] for r in records
                  if r.get(key) is not None and isinstance(r[key], (int, float))]
        if not values:
            result[key] = {'mean': None, 'std': None, 'min': None, 'max': None}
            continue
        mean = sum(values) / len(values)
        var = sum((v - mean) ** 2 for v in values) / len(values)
        result[key] = {
            'mean': mean,
            'std': math.sqrt(var),
            'min': min(values),
            'max': max(values),
        }
    return result


def success_rate(records):
    """成功率（0~1）。"""
    if not records:
        return 0.0
    return sum(1 for r in records if r.get('success')) / len(records)
