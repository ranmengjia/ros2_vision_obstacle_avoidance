"""metrics.py 的纯函数单测（不依赖 ROS/Gazebo）。"""
import math

import pytest

from metrics import (
    aggregate,
    angular_jerk,
    angular_reversals,
    classify_failure,
    collision_episodes,
    crossed_finish,
    distance,
    first_reach_time,
    min_clearance,
    path_length,
    reached_goal,
    stopped_fraction,
    success_rate,
)

INF = float('inf')


# ========== path_length / distance ==========

def test_path_length_straight_line():
    assert path_length([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]) == pytest.approx(2.0)


def test_path_length_empty_and_single():
    assert path_length([]) == 0.0
    assert path_length([(1.0, 2.0)]) == 0.0


def test_path_length_diagonal():
    assert path_length([(0.0, 0.0), (3.0, 4.0)]) == pytest.approx(5.0)


def test_distance_is_symmetric():
    assert distance((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)
    assert distance((3.0, 4.0), (0.0, 0.0)) == pytest.approx(5.0)


# ========== min_clearance ==========

def test_min_clearance_picks_smallest_valid():
    assert min_clearance([[INF, 1.0], [0.4, 2.0]]) == pytest.approx(0.4)


def test_min_clearance_ignores_zero_negative_and_nan():
    scans = [[0.0, -1.0, float('nan'), INF, 0.8]]
    assert min_clearance(scans) == pytest.approx(0.8)


def test_min_clearance_no_echo_is_inf():
    assert min_clearance([[INF, INF], []]) == INF


# ========== collision_episodes ==========

def test_collision_requires_consecutive_frames():
    """单帧擦碰（雷达噪声）不算碰撞。"""
    assert collision_episodes([[0.10], [1.0], [0.10]], threshold=0.13) == 0


def test_collision_counts_once_per_episode():
    """持续贴障 5 帧只算 1 次碰撞。"""
    scans = [[0.10]] * 5
    assert collision_episodes(scans, threshold=0.13, min_consecutive=2) == 1


def test_collision_counts_two_separate_episodes():
    scans = [[0.10], [0.10], [1.0], [0.10], [0.10]]
    assert collision_episodes(scans, threshold=0.13, min_consecutive=2) == 2


def test_collision_ignores_inf_and_zero():
    scans = [[INF], [0.0], [0.0], [INF]]
    assert collision_episodes(scans, threshold=0.13) == 0


# ========== stopped_fraction ==========

def test_stopped_fraction_half():
    assert stopped_fraction([0.0, 0.005, 0.2, 0.1], eps=0.01) == pytest.approx(0.5)


def test_stopped_fraction_empty():
    assert stopped_fraction([]) == 0.0


# ========== angular_reversals ==========

def test_reversals_ignore_near_zero_noise():
    assert angular_reversals([0.5, 0.01, 0.5, -0.02, 0.6]) == 0


def test_reversals_counts_alternation():
    assert angular_reversals([0.5, -0.5, 0.5, -0.5]) == 3


def test_reversals_single_sample():
    assert angular_reversals([0.5]) == 0


# ========== angular_jerk ==========

def test_jerk_zero_for_constant_signal():
    assert angular_jerk([0.3] * 10, dt=0.1) == pytest.approx(0.0)


def test_jerk_zero_for_ramp():
    """线性斜坡的二阶差分为 0。"""
    assert angular_jerk([0.0, 0.1, 0.2, 0.3], dt=0.1) == pytest.approx(0.0)


def test_jerk_positive_for_step():
    assert angular_jerk([0.0, 0.0, 1.0, 1.0], dt=0.1) > 0.0


def test_jerk_short_sequence_is_zero():
    assert angular_jerk([0.0, 1.0], dt=0.1) == 0.0
    assert angular_jerk([0.0, 1.0], dt=0.0) == 0.0


# ========== first_reach_time ==========

def test_first_reach_time_hits_goal():
    times = [0.0, 1.0, 2.0, 3.0]
    points = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.2, 0.0)]
    assert first_reach_time(times, points, (1.0, 0.0), 0.25) == pytest.approx(2.0)


def test_first_reach_time_none_when_far():
    assert first_reach_time([0.0], [(0.0, 0.0)], (5.0, 0.0), 0.25) is None


def test_first_reach_time_boundary_inclusive():
    assert first_reach_time([1.0], [(0.25, 0.0)], (0.0, 0.0), 0.25) == pytest.approx(1.0)


# ========== 终点线判定（反应式臂不收敛到点） ==========

def test_crossed_finish_passes_behind_goal_in_lane():
    # 越过 x=2.5 且 y 偏 0.8m（在 1.2m 容差带内）
    assert crossed_finish((2.6, 0.8), (2.5, 0.0), 0.35, (1.0, 0.0), 1.2)


def test_crossed_finish_fails_outside_lane():
    assert not crossed_finish((2.6, 1.5), (2.5, 0.0), 0.35, (1.0, 0.0), 1.2)


def test_crossed_finish_fails_before_goal():
    assert not crossed_finish((2.0, 0.0), (2.5, 0.0), 0.35, (1.0, 0.0), 1.2)


def test_crossed_finish_respects_tolerance_lead():
    # 距终点 0.3m（< tolerance 0.35）时允许提前判过线
    assert crossed_finish((2.2, 0.0), (2.5, 0.0), 0.35, (1.0, 0.0), 1.2)


def test_reached_goal_circle_without_lane_config():
    assert reached_goal((0.2, 0.0), (0.0, 0.0), 0.25)
    assert not reached_goal((1.0, 0.0), (0.0, 0.0), 0.25)


def test_first_reach_time_via_finish_line():
    times = [0.0, 1.0, 2.0]
    points = [(0.0, 0.0), (0.5, 0.5), (1.5, 0.5)]
    assert first_reach_time(
        times, points, (1.0, 0.0), 0.2, (1.0, 0.0), 1.2) == pytest.approx(2.0)


# ========== classify_failure ==========

def test_success_has_no_failure():
    assert classify_failure(True, 3, 1.0, 0.0) is None


def test_failure_collision():
    assert classify_failure(False, 1, 0.2, 3.0, clearance=0.05) == 'collision'


def test_failure_stuck():
    assert classify_failure(False, 0, 0.8, 3.0) == 'stuck'


def test_failure_no_motion():
    assert classify_failure(False, 0, 0.2, 0.1) == 'no_motion'


def test_failure_timeout():
    assert classify_failure(False, 0, 0.2, 3.0) == 'timeout'


# ========== aggregate / success_rate ==========

def test_aggregate_stats():
    records = [{'t': 1.0}, {'t': 2.0}, {'t': 3.0}]
    agg = aggregate(records, ['t'])['t']
    assert agg['mean'] == pytest.approx(2.0)
    assert agg['min'] == pytest.approx(1.0)
    assert agg['max'] == pytest.approx(3.0)
    assert agg['std'] == pytest.approx(math.sqrt(2.0 / 3.0))


def test_aggregate_skips_none_values():
    records = [{'t': None}, {'t': 4.0}]
    agg = aggregate(records, ['t'])['t']
    assert agg['mean'] == pytest.approx(4.0)


def test_aggregate_missing_key_gives_none():
    agg = aggregate([{'x': 1.0}], ['t'])['t']
    assert agg['mean'] is None


def test_success_rate():
    assert success_rate([{'success': True}, {'success': False}]) == pytest.approx(0.5)
    assert success_rate([]) == 0.0
