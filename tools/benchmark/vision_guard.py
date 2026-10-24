"""混合臂（hybrid）：Nav2 输出 + OpenCV 近距安全护栏。

Nav2 负责全局规划与路径跟踪（依赖先验地图和雷达），
本节点把 collision_monitor 的输出（/cmd_vel_raw）转发到 /cmd_vel，
并在视觉判定正前方近距被堵时强制停车——用视觉补上雷达安装高度看不到的
低矮/贴地障碍，同时保留 Nav2 的全局最优性。

用法（由 run_benchmark.py 启动）：
    python3 tools/benchmark/vision_guard.py --ros-args -p guard_threshold:=0.3
"""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from vision_perception.perception_node import compute_zone_scores


def apply_guard(linear, angular, center_score, threshold):
    """视觉护栏纯函数：正前方可通行率低于阈值时输出零速度。"""
    if center_score < threshold:
        return 0.0, 0.0
    return linear, angular


class VisionGuard(Node):
    def __init__(self):
        super().__init__('vision_guard')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')
        self.declare_parameter('gray_threshold', 150)
        self.declare_parameter('roi_bottom_ratio', 0.4)
        self.declare_parameter('guard_threshold', 0.3)
        self.declare_parameter('image_timeout', 0.5)

        self.gray_threshold = self.get_parameter('gray_threshold').value
        self.roi_bottom_ratio = self.get_parameter('roi_bottom_ratio').value
        self.guard_threshold = self.get_parameter('guard_threshold').value
        self.image_timeout = self.get_parameter('image_timeout').value

        self.bridge = CvBridge()
        self.center_score = 1.0
        self.last_image_time = None
        self.last_cmd = Twist()
        self.has_cmd = False

        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.image_callback, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_in_topic').value,
            self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_out_topic').value, 10)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            f'视觉护栏已启动：中路评分 < {self.guard_threshold} 时强制停车')

    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'图像转换失败: {exc}')
            return
        scores = compute_zone_scores(
            bgr, self.gray_threshold, self.roi_bottom_ratio)
        self.center_score = scores[1]
        self.last_image_time = time.monotonic()

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.has_cmd = True

    def timer_callback(self):
        if not self.has_cmd:
            return
        image_fresh = (self.last_image_time is not None
                       and time.monotonic() - self.last_image_time
                       <= self.image_timeout)
        center = self.center_score if image_fresh else 1.0
        linear, angular = apply_guard(
            self.last_cmd.linear.x, self.last_cmd.angular.z,
            center, self.guard_threshold)
        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
