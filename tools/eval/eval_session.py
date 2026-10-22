"""单场景评估采集：订阅图像/里程计/控制指令，跑固定时长后输出 JSON 指标。

用法:
    python3 tools/eval/eval_session.py --scene obstacle_course --run 1 \
        --duration 40 --out docs/experiments/data/obstacle_course_r1.json

必须在 source 过工作区 install/setup.bash 的终端里运行（需 import vision_perception）。
"""
import argparse
import json
import math
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image

from vision_perception.perception_node import compute_zone_scores

# 三个场景的障碍物布局一致（步骤6 的变量唯一性原则）
BOXES = [
    (1.5, 0.5, 0.4, 0.4),
    (3.0, -0.8, 0.3, 0.3),
    (3.6, 1.0, 0.5, 0.5),
]
ROBOT_HALF_X, ROBOT_HALF_Y = 0.15, 0.10  # base_link 0.30 x 0.20 的一半


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EvalSession(Node):
    def __init__(self, duration, gray_threshold, roi_bottom_ratio):
        super().__init__('eval_session')
        self.duration = duration
        self.gray_threshold = gray_threshold
        self.roi_bottom_ratio = roi_bottom_ratio

        self.bridge = CvBridge()

        # 图像统计
        self.frames = 0
        self.image_stamps = []
        self.proc_ms_sum = 0.0
        self.latency_ms_sum = 0.0
        self.zone_sums = [0.0, 0.0, 0.0]
        self.zone_min = [1.0, 1.0, 1.0]
        self.last_frame_wall = None
        self.max_image_gap = 0.0

        # 里程计统计
        self.last_xy = None
        self.distance = 0.0
        self.pose = (0.0, 0.0, 0.0)
        self.collision_events = []

        # 控制统计
        self.cmd_count = 0
        self.cmd_changes = 0
        self.stop_count = 0
        self.max_abs_angular = 0.0
        self.last_cmd = None

        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

    def on_image(self, msg):
        t0 = time.perf_counter()
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        scores = compute_zone_scores(
            cv_img, self.gray_threshold, self.roi_bottom_ratio)
        self.proc_ms_sum += (time.perf_counter() - t0) * 1000

        wall = time.time()
        if self.last_frame_wall is not None:
            self.max_image_gap = max(self.max_image_gap, wall - self.last_frame_wall)
        self.last_frame_wall = wall

        self.frames += 1
        self.image_stamps.append(Time.from_msg(msg.header.stamp).nanoseconds)
        for i in range(3):
            self.zone_sums[i] += scores[i]
            self.zone_min[i] = min(self.zone_min[i], scores[i])

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.last_xy is not None:
            self.distance += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)
        self.pose = (p.x, p.y, yaw)

        # AABB 碰撞判定：机器人中心进入障碍物外扩余量范围即记一次
        for bx, by, bw, bh in BOXES:
            inside = (abs(p.x - bx) < bw / 2 + ROBOT_HALF_X
                      and abs(p.y - by) < bh / 2 + ROBOT_HALF_Y)
            tag = f'{bx},{by}'
            if inside and tag not in [e['box'] for e in self.collision_events]:
                self.collision_events.append(
                    {'box': tag, 't': round(time.time() % 10000, 2),
                     'pose': [round(p.x, 2), round(p.y, 2)]})

    def on_cmd(self, msg):
        self.cmd_count += 1
        az, lx = msg.angular.z, msg.linear.x
        self.max_abs_angular = max(self.max_abs_angular, abs(az))
        if abs(lx) < 1e-6 and abs(az) < 1e-6:
            self.stop_count += 1
        if self.last_cmd is not None:
            prev_az, prev_lx = self.last_cmd
            if abs(az - prev_az) > 0.05 or (abs(lx) > 1e-6) != (abs(prev_lx) > 1e-6):
                self.cmd_changes += 1
        self.last_cmd = (az, lx)

    def report(self):
        n = max(self.frames, 1)
        elapsed = self.duration
        return {
            'frames': self.frames,
            'fps': round(self.frames / elapsed, 1),
            'proc_ms_mean': round(self.proc_ms_sum / n, 2),
            'zone_mean': [round(s / n, 3) for s in self.zone_sums],
            'zone_min': [round(m, 3) for m in self.zone_min],
            'cmd_msgs': self.cmd_count,
            'cmd_changes': self.cmd_changes,
            'stop_ratio': round(self.stop_count / max(self.cmd_count, 1), 3),
            'max_abs_angular': round(self.max_abs_angular, 2),
            'distance_m': round(self.distance, 2),
            'final_pose': [round(v, 2) for v in self.pose],
            'collisions': len(self.collision_events),
            'collision_events': self.collision_events,
            'max_image_gap_s': round(self.max_image_gap, 2),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True)
    ap.add_argument('--run', type=int, default=1)
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--gray-threshold', type=int, default=150)
    ap.add_argument('--roi-bottom-ratio', type=float, default=0.4)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rclpy.init()
    node = EvalSession(args.duration, args.gray_threshold, args.roi_bottom_ratio)

    t_start = time.time()
    while rclpy.ok() and time.time() - t_start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    metrics = node.report()
    metrics.update({'scene': args.scene, 'run': args.run,
                    'duration_s': args.duration})
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    text = json.dumps(metrics, ensure_ascii=False, indent=2)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text + '\n')


if __name__ == '__main__':
    main()
