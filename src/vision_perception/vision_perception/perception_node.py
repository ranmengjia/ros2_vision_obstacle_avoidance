"""v1.1 视觉感知节点：斜坡平滑 + 直行滞回 + FPS/延迟统计。"""
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

def compute_zone_scores(bgr_img, gray_threshold, roi_bottom_ratio):
    """输入 BGR 图像，返回 (左, 中, 右) 三个区域的可通行率（0~1）。

    纯函数：不依赖 ROS，方便步骤 4 用 pytest 直接测试。
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
                   hysteresis_margin, straight_exit_margin, stop_exit_margin):
    """三分评分 → 动作决策（带滞回）。

    scores: (左, 中, 右) 可通行率
    turn_dir: 转向记忆，+1=左转, -1=右转, 0=未转向
    prev_action: 上一拍动作（直行/停车退出滞回用，首次为 None）
    返回 (action, new_turn_dir)，action ∈ straight/left/right/stop
    """
    left, center, right = scores

    # 停车退出滞回：已停车时，需要某侧回升到 stop_threshold + 该余量
    # 才允许重新起步，避免可通行率在停车阈值附近抖动导致
    # 停车/转向交替抽搐。
    stop_line = stop_threshold
    if prev_action == 'stop':
        stop_line = stop_threshold + stop_exit_margin

    # 优先级 1：中路畅通 → 直行，清除转向记忆。
    # 双阈值滞回：进入直行要 >= clear_threshold；已在直行时只要
    # >= 退出阈值就保持，避免可通行率在临界值附近波动时
    # 直行/转向来回抽搐。
    straight_exit = clear_threshold - straight_exit_margin
    if (prev_action == 'straight' and center >= straight_exit) \
            or center >= clear_threshold:
        return 'straight', 0

    # 优先级 2：三路全堵 → 停车（无安全方向）
    if (left < stop_line and center < stop_line
            and right < stop_line):
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

        # 参数集中声明，launch/yaml 可覆盖
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('log_period_frames', 30)

        self.declare_parameter('gray_threshold', 150)
        self.declare_parameter('roi_bottom_ratio', 0.4)
        self.gray_threshold = self.get_parameter('gray_threshold').value
        self.roi_bottom_ratio = self.get_parameter('roi_bottom_ratio').value

        self.image_topic = self.get_parameter('image_topic').value
        self.log_period = self.get_parameter('log_period_frames').value

        self.bridge = CvBridge()
        self.frame_count = 0
        self.error_count = 0

        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(f'已订阅 {self.image_topic}，等待图像...')

        # 决策参数
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

        # 控制状态
        self.turn_dir = 0            # 滞回记忆：+1 左, -1 右, 0 无
        self.prev_action = None      # 上一拍动作（直行退出滞回用）
        self.current_angular_z = 0.0  # 平滑后的当前角速度
        self.last_scores = None      # 最近有效帧的三分
        self.last_image_time = None  # 最近有效帧的到达时间
        self.stopped_by_watchdog = False

        # FPS / 端到端延迟统计（每个日志周期重置一次窗口）
        self.fps_frames = 0
        self.latency_sum = 0.0
        self.processing_sum = 0.0
        self.fps_window_start = None

        # /cmd_vel 发布者 + 10 Hz 控制定时器
        self.control_period = 0.1
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ctrl_timer = self.create_timer(self.control_period, self.control_loop)

    def image_callback(self, msg: Image):
        cb_start = time.perf_counter()
        # 第 1 层：空图像检查
        if not msg.data:
            self.error_count += 1
            self.get_logger().warn('收到空图像帧')
            return

        # 第 2 层：编码转换（异常编码抛 CvBridgeError）
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.error_count += 1
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        # 第 3 层：尺寸检查
        if cv_img.shape[0] == 0 or cv_img.shape[1] == 0:
            self.error_count += 1
            self.get_logger().warn('图像尺寸为零')
            return

        now = self.get_clock().now()

        scores = compute_zone_scores(
            cv_img, self.gray_threshold, self.roi_bottom_ratio)

        self.frame_count += 1
        self.last_scores = scores
        self.last_image_time = now

        # FPS / 端到端延迟统计：延迟 = 图像时间戳 → 处理完成。
        # msg.header.stamp 是 builtin_interfaces/Time，与 rclpy 的 Time
        # 不是同一个类、不能直接相减，先转换再取纳秒差。
        # Gazebo 的 /clock 到达节点存在数十毫秒滞后，相机 stamp 可能
        # 比节点当前时钟更"新"，此时差值为负——按 0 处理，避免统计失真。
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
            self.get_logger().info(
                f'第 {self.frame_count} 帧 ({w}x{h}) 异常 {self.error_count} | '
                f'可通行率 左={scores[0]:.2f} 中={scores[1]:.2f} 右={scores[2]:.2f} | '
                f'FPS={fps:.1f} 端到端延迟={avg_latency_ms:.0f}ms 处理耗时={avg_proc_ms:.1f}ms')
            # 重置统计窗口
            self.fps_frames = 0
            self.latency_sum = 0.0
            self.processing_sum = 0.0
            self.fps_window_start = now

    def control_loop(self):
        """10 Hz 控制循环：看门狗 + 决策 + 发布。"""
        twist = Twist()

        # 还没收到过图像：不发任何指令，等第一帧
        if self.last_image_time is None:
            return

        # 看门狗：图像超时 → 强制停车（持续发全零）
        age = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if age > self.safety_timeout:
            if not self.stopped_by_watchdog:
                self.get_logger().warn(f'图像超时 {age:.1f}s，强制停车')
                self.stopped_by_watchdog = True
            self.current_angular_z = 0.0  # 看门狗立即清零，恢复后从 0 重新起坡
            self.cmd_pub.publish(twist)
            return
        self.stopped_by_watchdog = False

        # 正常决策
        action, self.turn_dir = decide_command(
            self.last_scores, self.turn_dir, self.prev_action,
            self.clear_threshold, self.stop_threshold,
            self.hysteresis_margin, self.straight_exit_margin,
            self.stop_exit_margin)
        self.prev_action = action

        if action == 'straight':
            twist.linear.x = self.linear_speed
        elif action == 'left':
            twist.linear.x = self.turn_linear_speed
            twist.angular.z = self.turn_angular_speed
        elif action == 'right':
            twist.linear.x = self.turn_linear_speed
            twist.angular.z = -self.turn_angular_speed
        # 'stop' 时 twist 保持全零

        # 角速度斜坡限幅：目标值跳变 → 实际输出缓坡逼近
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