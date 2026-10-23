"""v1.1 感知、决策与平滑纯函数的单元测试。"""
import math

import numpy as np
import pytest

from vision_perception.perception_node import (
    compute_zone_scores,
    decide_command,
    smooth_step,
    sector_min_distances,
    fuse_scores,
)

W, H = 320, 240
CLEAR, STOP, MARGIN, EXIT_MARGIN, STOP_EXIT_MARGIN = 0.7, 0.3, 0.2, 0.15, 0.1

def make_image(bright=255):
    """生成纯色 BGR 图像（三通道同值，灰度化后不变）。"""
    return np.full((H, W, 3), bright, dtype=np.uint8)

# ========== compute_zone_scores：ROI 掩膜与三分评分 ==========

def test_all_bright_is_fully_passable():
    scores = compute_zone_scores(make_image(255), 150, 0.4)
    assert scores == pytest.approx((1.0, 1.0, 1.0))

def test_left_third_blocked():
    img = make_image(255)
    img[:, :W // 3] = 0
    left, center, right = compute_zone_scores(img, 150, 0.4)
    assert left == pytest.approx(0.0)
    assert center == pytest.approx(1.0)
    assert right == pytest.approx(1.0)

def test_center_third_blocked():
    img = make_image(255)
    third = W // 3
    img[:, third:2 * third] = 0
    left, center, right = compute_zone_scores(img, 150, 0.4)
    assert left == pytest.approx(1.0)
    assert center == pytest.approx(0.0)
    assert right == pytest.approx(1.0)

def test_top_area_ignored_by_roi():
    """画面上半部分变黑不应影响分数——ROI 只取底部 40%。"""
    img = make_image(255)
    roi_top = int(H * (1.0 - 0.4))
    img[:roi_top, :] = 0
    scores = compute_zone_scores(img, 150, 0.4)
    assert scores == pytest.approx((1.0, 1.0, 1.0))

def test_threshold_is_strictly_greater():
    """阈值语义：> 150 才算可通行，等于 150 算障碍。"""
    assert compute_zone_scores(make_image(150), 150, 0.4) == pytest.approx((0.0, 0.0, 0.0))
    assert compute_zone_scores(make_image(151), 150, 0.4) == pytest.approx((1.0, 1.0, 1.0))

def test_scores_are_three_floats_in_range():
    scores = compute_zone_scores(make_image(255), 150, 0.4)
    assert len(scores) == 3
    assert all(0.0 <= s <= 1.0 for s in scores)

# ========== decide_command：决策优先级与滞回 ==========

def decide(scores, turn_dir=0, prev_action=None):
    """测试内的小包装，省去每次传六个阈值。"""
    return decide_command(scores, turn_dir, prev_action,
                          CLEAR, STOP, MARGIN, EXIT_MARGIN, STOP_EXIT_MARGIN)

def test_center_clear_goes_straight():
    action, turn = decide((0.2, 0.9, 0.2))
    assert (action, turn) == ('straight', 0)

def test_all_blocked_stops():
    action, turn = decide((0.1, 0.1, 0.1), turn_dir=1)
    assert (action, turn) == ('stop', 0)

def test_first_decision_picks_higher_side():
    assert decide((0.6, 0.5, 0.2)) == ('left', 1)
    assert decide((0.2, 0.5, 0.6)) == ('right', -1)

def test_first_decision_tie_goes_left():
    """平局默认左转，保证决策确定性。"""
    assert decide((0.5, 0.5, 0.5)) == ('left', 1)

def test_hysteresis_keeps_turning_left():
    """右侧只略好（未超过 margin），继续左转，不被噪声带着抖。"""
    assert decide((0.55, 0.5, 0.65), turn_dir=1) == ('left', 1)

def test_hysteresis_switches_when_clearly_better():
    """右侧好出 0.2 以上才切换方向。"""
    assert decide((0.4, 0.5, 0.75), turn_dir=1) == ('right', -1)

def test_blocked_turn_side_switches_immediately():
    """左转中左侧被堵死，安全优先于滞回，立刻换边。"""
    assert decide((0.1, 0.5, 0.5), turn_dir=1) == ('right', -1)

def test_mirror_cases_for_right_turn():
    assert decide((0.65, 0.5, 0.55), turn_dir=-1) == ('right', -1)
    assert decide((0.75, 0.5, 0.4), turn_dir=-1) == ('left', 1)
    assert decide((0.5, 0.5, 0.1), turn_dir=-1) == ('left', 1)

# ========== decide_command：直行退出滞回（v1.1 新增） ==========

def test_straight_exit_hysteresis_stays_straight():
    """正在直行时中路降到 0.55~0.7 之间：保持在直行，不抽搐。"""
    assert decide((0.2, 0.6, 0.2), prev_action='straight') == ('straight', 0)

def test_straight_exit_hysteresis_exits_below():
    """中路跌破退出阈值 0.55：允许退出直行。"""
    action, _ = decide((0.6, 0.5, 0.6), prev_action='straight')
    assert action != 'straight'

def test_straight_enter_requires_full_threshold():
    """非直行状态必须达到完整进入阈值 0.7 才开始直行。"""
    assert decide((0.2, 0.6, 0.2), prev_action=None) == ('left', 1)

def test_straight_exit_boundary_is_exact():
    """退出边界语义：center 恰好等于退出阈值时仍保持直行。"""
    assert decide((0.2, 0.55, 0.2), prev_action='straight') == ('straight', 0)
    assert decide((0.2, 0.5499, 0.2), prev_action='straight') == ('left', 1)

# ========== decide_command：停车退出滞回（v1.1 新增） ==========

def test_stop_hysteresis_ignores_borderline_recovery():
    """已停车时，某侧只回升到 0.3~0.4 之间：保持停车，不来回抽搐。"""
    assert decide((0.05, 0.19, 0.33), prev_action='stop') == ('stop', 0)

def test_stop_hysteresis_resumes_when_clearly_recovered():
    """某侧回升超过 0.4：允许重新起步转向。"""
    action, _ = decide((0.05, 0.19, 0.45), prev_action='stop')
    assert action == 'right'

def test_stop_threshold_applies_without_previous_stop():
    """未处于停车状态时仍用原阈值 0.3：右侧 0.33 足以起步。"""
    assert decide((0.05, 0.19, 0.33), prev_action='left') == ('right', -1)

# ========== smooth_step：斜坡限幅（v1.1 新增） ==========

def test_smooth_step_clamps_large_increase():
    """目标比当前大得多：一步最多走 max_delta。"""
    assert smooth_step(0.5, 0.0, 0.2) == pytest.approx(0.2)

def test_smooth_step_passes_small_change():
    """目标在限幅范围内：一步到位。"""
    assert smooth_step(0.1, 0.0, 0.2) == pytest.approx(0.1)

def test_smooth_step_clamps_large_decrease():
    assert smooth_step(-0.5, 0.3, 0.2) == pytest.approx(0.1)

def test_smooth_step_no_change_when_equal():
    assert smooth_step(0.3, 0.3, 0.2) == pytest.approx(0.3)

def test_smooth_step_symmetric_around_zero():
    """正负方向限幅对称。"""
    assert smooth_step(-0.1, 0.0, 0.2) == pytest.approx(-0.1)
    assert smooth_step(0.0, -0.3, 0.2) == pytest.approx(-0.1)


# ========== sector_min_distances：雷达三扇区最小距离 ==========

# 仿真环境参数：360 点，1° 步进，-π ~ +π
SCAN_N = 360
SCAN_ANGLE_MIN = -math.pi
SCAN_ANGLE_INC = math.pi / 180.0
FOV_DEG = 60.0   # 相机 FOV，与雷达前方映射一致


def _make_scan(ranges_dict, default=10.0):
    """构造一个模拟 ranges 数组。

    ranges_dict: {角度(度): 距离}，指定角度设为指定距离，其余用 default。
    """
    arr = [default] * SCAN_N
    for deg, dist in ranges_dict.items():
        i = int(round((math.radians(deg) - SCAN_ANGLE_MIN) / SCAN_ANGLE_INC))
        if 0 <= i < SCAN_N:
            arr[i] = dist
    return arr


def test_no_obstacle_gives_all_inf():
    """全无障碍时三扇区都是 inf。"""
    ranges = _make_scan({}, default=float('inf'))
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert left == math.inf
    assert center == math.inf
    assert right == math.inf


def test_front_obstacle_lands_in_center_sector():
    """正前方 0° 障碍 → 中扇区距离最小。"""
    ranges = _make_scan({0: 1.5}, default=float('inf'))
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert center == pytest.approx(1.5)
    assert left == math.inf
    assert right == math.inf


def test_left_obstacle_lands_in_left_sector():
    """左前方 +20° 障碍 → 左扇区距离最小（视觉左=北=角度正方向）。"""
    ranges = _make_scan({20: 2.0}, default=float('inf'))
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert left == pytest.approx(2.0)
    assert center == math.inf
    assert right == math.inf


def test_right_obstacle_lands_in_right_sector():
    """右前方 -20° 障碍 → 右扇区距离最小（视觉右=南=角度负方向）。"""
    ranges = _make_scan({-20: 2.0}, default=float('inf'))
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert right == pytest.approx(2.0)
    assert left == math.inf
    assert center == math.inf


def test_obstacle_outside_fov_ignored():
    """后向 180° 障碍不在 ±30° 范围内，三扇区全 inf。"""
    ranges = _make_scan({180: 0.5}, default=float('inf'))
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert left == math.inf
    assert center == math.inf
    assert right == math.inf


def test_multi_sector_obstacles():
    """左 +15° 和中 0° 都有障碍 → 两扇区各自记录。"""
    ranges = _make_scan({15: 2.5, 0: 1.0}, default=float('inf'))
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert left == pytest.approx(2.5)
    assert center == pytest.approx(1.0)
    assert right == math.inf


def test_zero_and_negative_and_inf_filtered():
    """ranges 中的 0 / 负数 / inf 都应该被过滤，不参与 min。"""
    ranges = _make_scan({}, default=math.inf)  # 先全填 inf
    ranges[180] = -0.1    # 负数
    ranges[181] = 0.0     # 零
    ranges[182] = 1.2     # 有效障碍（0° 附近）
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert center == pytest.approx(1.2)   # 只有 1.2 有效
    assert left == math.inf
    assert right == math.inf


def test_sector_min_takes_closest_in_range():
    """一个扇区内多根射线有值时取最小。"""
    ranges = _make_scan({}, default=float('inf'))
    # 左扇区（+10° ~ +30°）内放两根障碍射线（视觉左=北=角度正方向）
    ranges[190] = 3.0   # +10°
    ranges[200] = 1.8   # +20°
    left, center, right = sector_min_distances(
        ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_INC, FOV_DEG)
    assert left == pytest.approx(1.8)
    assert center == math.inf
    assert right == math.inf


# ========== fuse_scores：雷达安全闸门融合 ==========

SAFE = 0.3  # 安全距离阈值


def test_no_lidar_obstacle_keeps_vision():
    """三扇区全 inf（无障碍）→ 融合结果 = 视觉原评分。"""
    scores = (0.8, 0.6, 0.9)
    dists = (math.inf, math.inf, math.inf)
    assert fuse_scores(scores, dists, SAFE) == pytest.approx(scores)


def test_close_obstacle_forces_to_zero():
    """某扇区距离 < SAFE → 该方向强制 0，其余保持视觉。"""
    scores = (0.8, 0.6, 0.9)
    dists = (0.2, math.inf, math.inf)
    fused = fuse_scores(scores, dists, SAFE)
    assert fused[0] == 0.0            # 左方向被闸门
    assert fused[1] == pytest.approx(0.6)   # 中保持
    assert fused[2] == pytest.approx(0.9)   # 右保持


def test_exactly_at_safety_distance_keeps_vision():
    """距离恰好 = SAFE → 不触发闸门（严格 < 才触发）。"""
    scores = (0.8, 0.6, 0.9)
    dists = (0.3, math.inf, math.inf)   # 刚好 0.3
    fused = fuse_scores(scores, dists, SAFE)
    assert fused[0] == pytest.approx(0.8)   # 保持
    assert fused[1] == pytest.approx(0.6)
    assert fused[2] == pytest.approx(0.9)


def test_just_above_safety_keeps_vision():
    """距离 = SAFE + 0.01 → 保持视觉评分。"""
    scores = (0.8, 0.6, 0.9)
    dists = (0.31, math.inf, math.inf)
    fused = fuse_scores(scores, dists, SAFE)
    assert fused[0] == pytest.approx(0.8)


def test_lidar_clear_lifts_vision_floor():
    """雷达无障碍（dist > clear_distance）→ 融合评分不低于 lidar_clear_floor。
    场景：障碍物遮挡画面，视觉评分 0.04，但雷达说该方向 3.88m 无障碍，
    融合后应为 max(0.04, 0.5) = 0.5，保证可转向绕行。"""
    scores = (0.04, 0.04, 0.04)
    dists = (3.88, 0.24, 0.16)  # 左无障碍，中/右近距离
    fused = fuse_scores(scores, dists, SAFE, lidar_clear_floor=0.5, lidar_clear_distance=1.0)
    assert fused[0] == pytest.approx(0.5)  # 雷达兜底
    assert fused[1] == 0.0  # 闸门
    assert fused[2] == 0.0  # 闸门


def test_lidar_clear_floor_keeps_high_vision():
    """雷达无障碍且视觉评分高于 floor → 保留视觉高分。"""
    scores = (0.9, 0.1, 0.1)
    dists = (float('inf'), float('inf'), float('inf'))
    fused = fuse_scores(scores, dists, SAFE, lidar_clear_floor=0.5)
    assert fused[0] == pytest.approx(0.9)
    assert fused[1] == pytest.approx(0.5)  # 视觉 0.1 < 0.5，被兜底
    assert fused[2] == pytest.approx(0.5)


def test_all_three_sectors_blocked_by_lidar():
    """三扇区近距离障碍 → 融合后全 0，强制停车。"""
    scores = (0.9, 0.8, 0.7)
    dists = (0.15, 0.2, 0.25)
    fused = fuse_scores(scores, dists, SAFE)
    assert fused == (0.0, 0.0, 0.0)


def test_vision_says_clear_but_lidar_says_close():
    """视觉认为中路畅通（0.9），但雷达发现 0.2m 内有障碍 → 强制 0。

    这是雷达闸门的核心价值：视觉受光照/对比度限制可能漏判，
    雷达给出真实距离兜底。
    """
    scores = (0.5, 0.9, 0.5)   # 视觉：中路畅通
    dists = (math.inf, 0.2, math.inf)  # 雷达：中路 0.2m 有墙
    fused = fuse_scores(scores, dists, SAFE)
    assert fused[1] == 0.0     # 闸门覆盖视觉误判