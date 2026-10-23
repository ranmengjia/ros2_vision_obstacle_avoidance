"""v2.0 融合感知节点：视觉三分评分 + 激光雷达安全闸门 + 斜坡平滑 + 滞回。"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist


def compute_zone_scores(bgr_img, gray_threshold, roi_bottom_ratio):
    """输入 BGR 图像，返回 (左, 中, 右) 三个区域的可通行率（0~1）。

    纯函数：不依赖 ROS，方便 pytest 直接测试。
    """
    h, w = bgr_img.shape[:2]

    # 1) 灰度化：只保留亮度
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    # 2) 二值化：> 阈值 → 255(白=可通行)，否则 0(黑=障碍)
    _, binary = cv2.threshold(gray, gray_threshold, 255, cv2.THRESH_BINARY)

    # 3) ROI：只留画面底部一段（正前方地面）
    roi_top = int(h * (1.0 - roi_bottom_ratio))
    roi = binary[roi_top:h, :]

    # 4) 横向均分三份：图像左 = 机器人左（光学坐标系约定）
    third = w // 3
    zones = (roi[:, :third], roi[:, third:2 * third], roi[:, 2 * third:])

    # 5) 白像素占比 = 该区域可通行率
    return tuple(float(np.count_nonzero(z)) / z.size for z in zones)


def sector_min_distances(ranges, angle_min, angle_increment, fov_deg=60.0):
    """LaserScan 的 ranges 数组 → 三扇区最小距离 (左, 中, 右)。

    纯函数：不依赖 ROS。
    雷达是 360° 全向扫描，但视觉只有前方 fov_deg°（默认 60°，与相机 FOV 对齐）。
    在这 ±fov/2 范围内再均分成左/中/右三个扇区，每个扇区取最小距离。

    关键：扇区顺序与视觉三分对齐——
      视觉左 = 机器人左 = 北 = 雷达角度正方向 → 对应 [+fov/6, +fov/2]
      视觉中 = 正前方 → 对应 [-fov/6, +fov/6]
      视觉右 = 机器人右 = 南 = 雷达角度负方向 → 对应 [-fov/2, -fov/6]

    ranges: list/ndarray of float, 长度 N
    angle_min: 第一根射线的角度（弧度）
    angle_increment: 相邻射线的角度步进（弧度）
    fov_deg: 需要映射的前方扇形角度（度），默认 60°
    返回: (left_min, center_min, right_min) 三扇区最小距离（inf 表示无障碍）
    """
    n = len(ranges)
    half_fov_rad = math.radians(fov_deg / 2.0)  # 默认 ±30°
    third_fov_rad = half_fov_rad / 3.0           # 每个扇区 20°

    # 扇区边界按 [左, 中, 右] 顺序，其中：
    #   左 = [+third, +half]  （视觉左=北=角度正方向）
    #   中 = [-third, +third]
    #   右 = [-half, -third]  （视觉右=南=角度负方向）
    sectors = [
        (+third_fov_rad, +half_fov_rad),
        (-third_fov_rad, +third_fov_rad),
        (-half_fov_rad, -third_fov_rad),
    ]

    result = []
    for lo, hi in sectors:
        lo_idx = max(0, int(math.floor((lo - angle_min) / angle_increment)))
        hi_idx = min(n - 1, int(math.floor((hi - angle_min) / angle_increment)))
        if lo_idx > hi_idx:
            result.append(float('inf'))
            continue
        vals = [ranges[i] for i in range(lo_idx, hi_idx + 1)
                if math.isfinite(ranges[i])
                and ranges[i] > 0.0]
        result.append(min(vals) if vals else float('inf'))

    return tuple(result)


def fuse_scores(zone_scores, sector_dists, safety_distance,
                lidar_clear_floor=0.5, lidar_clear_distance=1.0):
    """视觉可通行率 + 雷达最小距离 → 融合后可通行率。

    纯函数：不依赖 ROS。

    融合策略三层：
    1. 雷达近距离（< safety_distance）→ 强制 0（硬墙闸门）
    2. 雷达无障碍（dist > lidar_clear_distance 或 inf）→ max(视觉, lidar_clear_floor)
       避免障碍物遮挡画面导致视觉评分过低，拖死决策
    3. 雷达中距离 → 视觉主导

    这样视觉给"远距离可通行率"，雷达给"近距离硬约束"和"无障碍兜底"，三者互补。

    zone_scores: (左, 中, 右) 视觉可通行率 0~1
    sector_dists: (左, 中, 右) 雷达最小距离（米），inf 表示无障碍
    safety_distance: 安全距离阈值（米），雷达距离 < 此值则强制不可通行
    lidar_clear_floor: 雷达确认无障碍时的评分下限（默认 0.5）
    lidar_clear_distance: 超过此距离视为无障碍（默认 1.0m）
    返回: (左, 中, 右) 融合后可通行率
    """
    fused = []
    for score, dist in zip(zone_scores, sector_dists):
        if math.isfinite(dist) and dist < safety_distance:
            fused.append(0.0)   # 闸门：近距离硬墙
        elif not math.isfinite(dist) or dist > lidar_clear_distance:
            fused.append(max(score, lidar_clear_floor))  # 雷达兜底无障碍
        else:
            fused.append(score)  # 中距离，视觉主导
    return tuple(fused)


def smooth_step(target, current, max_delta):
    """把 current 向 target 逼近一步，单步变化量不超过 max_delta（斜坡限幅）。

    纯函数：不依赖 ROS，方便 pytest 直接测试。
    """
    diff = target - current
    if diff > max_delta:
        return current + max_delta
    if diff < -max_delta:
        return current - max_delta
    return target


def decide_command(scores, turn_dir, prev_action, clear_threshold, stop_threshold,
                   hysteresis_margin, straight_exit_margin, stop_exit_margin,
                   sector_dists=None, side_guard_distance=0.25):
    """三分评分 → 动作决策（带滞回）。

    scores: (左, 中, 右) 可通行率（已融合雷达闸门）
    turn_dir: 转向记忆，+1=左转, -1=右转, 0=未转向
    prev_action: 上一拍动作（直行/停车退出滞回用，首次为 None）
    sector_dists: (左, 中, 右) 雷达扇区最小距离（可选，用于死锁逃逸与侧身护卫）
    side_guard_distance: 侧身护卫距离（米）。直行时若任一侧雷达距离小于此值，
        视为贴障过近，禁止直行（否则会擦着障碍边缘走，最终卡死在角落）
    返回 (action, new_turn_dir)，action ∈ straight/left/right/stop
    """
    left, center, right = scores

    # 停车退出滞回：已停车时，需要某侧回升到 stop_threshold + 该余量
    # 才允许重新起步，避免可通行率在停车阈值附近抖动导致
    # 停车/转向交替抽搐。
    stop_line = stop_threshold
    if prev_action == 'stop':
        stop_line = stop_threshold + stop_exit_margin

    # 侧身护卫：中路通畅但侧面贴障时不允许直行。
    # 雷达距离 inf/无数据视为通畅（不干预纯视觉场景）。
    side_clear = True
    if sector_dists is not None:
        for d in (sector_dists[0], sector_dists[2]):
            if math.isfinite(d) and d < side_guard_distance:
                side_clear = False
                break

    # 优先级 1：中路畅通 → 直行，清除转向记忆。
    # 双阈值滞回：进入直行要 >= clear_threshold；已在直行时只要
    # >= 退出阈值就保持，避免可通行率在临界值附近波动时
    # 直行/转向来回抽搐。
    straight_exit = clear_threshold - straight_exit_margin
    if side_clear and (
            (prev_action == 'straight' and center >= straight_exit)
            or center >= clear_threshold):
        return 'straight', 0

    # 优先级 2：三路全堵 → 停车（无安全方向）。
    # 例外：若三路是被雷达闸门堵死（正前方障碍物占满整个雷达视野），
    # 停车等于死锁——原地朝雷达距离更远的一侧自转，转出通路。
    # 平局默认左（保证确定性）。视觉堵死（雷达无数据）仍停车。
    if (left < stop_line and center < stop_line
            and right < stop_line):
        if sector_dists is not None:
            ld, rd = sector_dists[0], sector_dists[2]
            if math.isfinite(ld) or math.isfinite(rd):
                lval = ld if math.isfinite(ld) else float('inf')
                rval = rd if math.isfinite(rd) else float('inf')
                if lval >= rval:
                    return 'left', 1
                return 'right', -1
        return 'stop', 0

    # 优先级 3：正在转的方向被堵死、另一侧还行 → 直接换边（安全优先于滞回）
    if turn_dir == 1 and left < stop_threshold and right >= stop_line:
        return 'right', -1
    if turn_dir == -1 and right < stop_threshold and left >= stop_line:
        return 'left', 1

    # 优先级 4：滞回——保持原方向，除非另一侧明显更好
    if turn_dir == 1:
        if right > left + hysteresis_margin:
            return 'right', -1
        return 'left', 1
    if turn_dir == -1:
        if left > right + hysteresis_margin:
            return 'left', 1
        return 'right', -1

    # 优先级 5：首次决策——选分数高的一侧，平局默认左（保证确定性）
    if left >= right:
        return 'left', 1
    return 'right', -1


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ===== 视觉参数 =====
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('log_period_frames', 30)
        self.declare_parameter('gray_threshold', 150)
        self.declare_parameter('roi_bottom_ratio', 0.4)
        self.image_topic = self.get_parameter('image_topic').value
        self.log_period = self.get_parameter('log_period_frames').value
        self.gray_threshold = self.get_parameter('gray_threshold').value
        self.roi_bottom_ratio = self.get_parameter('roi_bottom_ratio').value

        # ===== 雷达参数（v2.0 新增）=====
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('side_guard_distance', 0.25)
        self.declare_parameter('scan_fov_deg', 60.0)
        self.declare_parameter('scan_timeout', 1.0)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.side_guard_distance = self.get_parameter('side_guard_distance').value
        self.scan_fov_deg = self.get_parameter('scan_fov_deg').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # ===== 决策参数 =====
        self.declare_parameter('clear_threshold', 0.7)
        self.declare_parameter('stop_threshold', 0.3)
        self.declare_parameter('hysteresis_margin', 0.2)
        self.declare_parameter('straight_exit_margin', 0.15)
        self.declare_parameter('stop_exit_margin', 0.1)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('turn_linear_speed', 0.05)
        self.declare_parameter('turn_angular_speed', 0.5)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('safety_timeout', 1.0)
        self.clear_threshold = self.get_parameter('clear_threshold').value
        self.stop_threshold = self.get_parameter('stop_threshold').value
        self.hysteresis_margin = self.get_parameter('hysteresis_margin').value
        self.straight_exit_margin = self.get_parameter('straight_exit_margin').value
        self.stop_exit_margin = self.get_parameter('stop_exit_margin').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.turn_linear_speed = self.get_parameter('turn_linear_speed').value
        self.turn_angular_speed = self.get_parameter('turn_angular_speed').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.safety_timeout = self.get_parameter('safety_timeout').value

        # ===== ROS 通信 =====
        self.bridge = CvBridge()
        self.frame_count = 0
        self.error_count = 0

        # 视觉订阅
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        self.get_logger().info(f'已订阅 {self.image_topic}，等待图像...')

        # 雷达订阅（v2.0 新增）
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.get_logger().info(
            f'已订阅 {self.scan_topic}，安全距离 {self.safety_distance}m...')

        # /cmd_vel 发布者 + 10 Hz 控制定时器
        self.control_period = 0.1
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ctrl_timer = self.create_timer(self.control_period, self.control_loop)

        # ===== 控制状态 =====
        self.turn_dir = 0
        self.prev_action = None
        self.current_angular_z = 0.0
        self.last_scores = None       # 最近融合后三分
        self.last_sector_dists = None # 最近雷达三扇区最小距离（用于日志）
        self.last_image_time = None
        self.last_scan_time = None    # 最近有效 scan 到达时间
        self.stopped_by_watchdog = False

        # FPS / 延迟统计
        self.fps_frames = 0
        self.latency_sum = 0.0
        self.processing_sum = 0.0
        self.fps_window_start = None

    # ------------------------------------------------------------------ #
    # 图像回调                                                            #
    # ------------------------------------------------------------------ #
    def image_callback(self, msg: Image):
        cb_start = time.perf_counter()
        if not msg.data:
            self.error_count += 1
            self.get_logger().warn('收到空图像帧')
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.error_count += 1
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        if cv_img.shape[0] == 0 or cv_img.shape[1] == 0:
            self.error_count += 1
            self.get_logger().warn('图像尺寸为零')
            return

        now = self.get_clock().now()

        # 视觉纯函数：三分可通行率
        vision_scores = compute_zone_scores(
            cv_img, self.gray_threshold, self.roi_bottom_ratio)

        # 如果已有雷达数据，做融合；否则直接用视觉评分（降级行为）
        if self.last_sector_dists is not None:
            self.last_scores = fuse_scores(
                vision_scores, self.last_sector_dists, self.safety_distance)
        else:
            self.last_scores = vision_scores

        self.frame_count += 1
        self.last_image_time = now

        # FPS / 延迟统计
        stamp = Time.from_msg(msg.header.stamp)
        latency = (now.nanoseconds - stamp.nanoseconds) / 1e9
        self.latency_sum += max(0.0, latency)
        self.processing_sum += (time.perf_counter() - cb_start) * 1000
        self.fps_frames += 1
        if self.fps_window_start is None:
            self.fps_window_start = now

        if self.frame_count % self.log_period == 0:
            elapsed = (now - self.fps_window_start).nanoseconds / 1e9
            fps = self.fps_frames / elapsed if elapsed > 0 else 0.0
            avg_latency_ms = self.latency_sum / self.fps_frames * 1000
            avg_proc_ms = self.processing_sum / self.fps_frames
            h, w = cv_img.shape[:2]
            dist_str = ('雷达 左={:.2f} 中={:.2f} 右={:.2f}'
                        .format(*(self.last_sector_dists or (0, 0, 0))))
            self.get_logger().info(
                f'第 {self.frame_count} 帧 ({w}x{h}) 异常 {self.error_count} | '
                f'融合后 左={self.last_scores[0]:.2f} 中={self.last_scores[1]:.2f} 右={self.last_scores[2]:.2f} | '
                f'{dist_str} | '
                f'FPS={fps:.1f} 延迟={avg_latency_ms:.0f}ms 耗时={avg_proc_ms:.1f}ms')
            self.fps_frames = 0
            self.latency_sum = 0.0
            self.processing_sum = 0.0
            self.fps_window_start = now

    # ------------------------------------------------------------------ #
    # 雷达回调（v2.0 新增）                                               #
    # ------------------------------------------------------------------ #
    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.get_logger().warn('收到空 scan 帧')
            return
        self.last_sector_dists = sector_min_distances(
            msg.ranges, msg.angle_min, msg.angle_increment, self.scan_fov_deg)
        self.last_scan_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    # 10 Hz 控制循环                                                      #
    # ------------------------------------------------------------------ #
    def control_loop(self):
        """10 Hz 控制循环：双看门狗（图像 + 雷达）+ 决策 + 发布。"""
        twist = Twist()

        # 还没收到过图像：不发任何指令
        if self.last_image_time is None:
            return

        now = self.get_clock().now()

        # 看门狗 1：图像超时
        img_age = (now - self.last_image_time).nanoseconds / 1e9
        scan_age = (float('inf') if self.last_scan_time is None
                    else (now - self.last_scan_time).nanoseconds / 1e9)

        # 雷达超时也触发安全停车——融合节点不能丢了雷达
        if img_age > self.safety_timeout or scan_age > self.scan_timeout:
            if not self.stopped_by_watchdog:
                reason = []
                if img_age > self.safety_timeout:
                    reason.append(f'图像 {img_age:.1f}s')
                if scan_age > self.scan_timeout:
                    reason.append(f'雷达 {scan_age:.1f}s')
                self.get_logger().warn(
                    f'安全超时: {", ".join(reason)}，强制停车')
                self.stopped_by_watchdog = True
            self.current_angular_z = 0.0
            self.cmd_pub.publish(twist)
            return
        self.stopped_by_watchdog = False

        # 正常决策（self.last_scores 已在 image_callback 中融合完毕）
        action, self.turn_dir = decide_command(
            self.last_scores, self.turn_dir, self.prev_action,
            self.clear_threshold, self.stop_threshold,
            self.hysteresis_margin, self.straight_exit_margin,
            self.stop_exit_margin,
            sector_dists=self.last_sector_dists,
            side_guard_distance=self.side_guard_distance)
        self.prev_action = action

        if action == 'straight':
            twist.linear.x = self.linear_speed
        elif action in ('left', 'right'):
            # 正前方有近距离障碍（center 被雷达闸门置 0）时原地转向，
            # 避免边前进边撞；正前方无障碍时低速前进绕行。
            center = self.last_scores[1] if self.last_scores else 1.0
            twist.linear.x = 0.0 if center <= 0.0 else self.turn_linear_speed
            twist.angular.z = (self.turn_angular_speed
                               if action == 'left' else -self.turn_angular_speed)

        max_delta = self.max_angular_accel * self.control_period
        self.current_angular_z = smooth_step(
            twist.angular.z, self.current_angular_z, max_delta)
        twist.angular.z = self.current_angular_z

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Humble 中 Ctrl+C 后 shutdown 可能已被内部调用
