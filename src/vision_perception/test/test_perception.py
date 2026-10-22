"""v1.0 感知与决策纯函数的单元测试。"""
import numpy as np
import pytest

from vision_perception.perception_node import compute_zone_scores, decide_command

W, H = 320, 240
CLEAR, STOP, MARGIN = 0.7, 0.3, 0.2

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

def decide(scores, turn_dir=0):
    """测试内的小包装，省去每次传四个阈值。"""
    return decide_command(scores, turn_dir, CLEAR, STOP, MARGIN)

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