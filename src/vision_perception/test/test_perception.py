"""v1.1 感知、决策与平滑纯函数的单元测试。"""
import numpy as np
import pytest

from vision_perception.perception_node import (
    compute_zone_scores,
    decide_command,
    smooth_step,
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