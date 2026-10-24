#!/usr/bin/env python3
"""基准结果汇总：把 results/ 下的单次 JSON 聚合成 Markdown 对比报告。

用法：
    python3 tools/benchmark/report.py
    python3 tools/benchmark/report.py --out tools/benchmark/results --chart

输出：
- <out>/report.md   场景 × 臂 的指标表 + 自动结论
- <out>/chart.png   成功率/到达时间分组柱状图（需要 matplotlib，缺失则跳过）
"""
import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from metrics import aggregate, success_rate  # noqa: E402

ARM_LABELS = {
    'vision': '纯 OpenCV',
    'lidar': '纯雷达',
    'fused': '视觉+雷达',
    'nav2': 'Nav2',
    'hybrid': 'Nav2+视觉护栏',
}
ARM_ORDER = ['vision', 'lidar', 'fused', 'nav2', 'hybrid']

# 报告列：键 → 表头（与 metrics.aggregate 对齐）
COLUMNS = [
    ('reach_time_sec', '到达时间(s)'),
    ('traveled_m', '路程(m)'),
    ('min_clearance_m', '最小间距(m)'),
    ('collision_count', '碰撞'),
    ('stopped_fraction', '卡死占比'),
    ('angular_jerk', '角速度jerk'),
    ('cpu_time_ratio', 'CPU占比'),
]
AGG_KEYS = [k for k, _ in COLUMNS]


def load_results(out_dir):
    records = []
    for path in sorted(glob.glob(os.path.join(out_dir, '*', '*', 'run_*.json'))):
        with open(path, 'r', encoding='utf-8') as f:
            records.append(json.load(f))
    return records


def group_records(records):
    grouped = {}
    for r in records:
        grouped.setdefault((r['scenario'], r['arm']), []).append(r)
    for rows in grouped.values():
        rows.sort(key=lambda r: r['run'])
    return grouped


def _fmt(value, digits=2):
    if value is None:
        return '—'
    return f'{value:.{digits}f}'


def build_rows(grouped):
    """(scenario, arm) → 聚合后的行数据。"""
    rows = []
    for (scenario, arm), records in sorted(grouped.items()):
        agg = aggregate(records, AGG_KEYS)
        done = [r for r in records if r['success']]
        failures = {}
        for r in records:
            if r['failure']:
                failures[r['failure']] = failures.get(r['failure'], 0) + 1
        rows.append({
            'scenario': scenario,
            'arm': arm,
            'runs': len(records),
            'success_rate': success_rate(records),
            'reach_mean': (sum(r['reach_time_sec'] for r in done) / len(done)
                           if done else None),
            'agg': agg,
            'failures': failures,
        })
    return rows


def format_table(rows, scenario=None):
    lines = []
    header = ['臂', '成功率', '到达时间(s)', '路程(m)', '最小间距(m)',
              '碰撞', '卡死占比', '角速度jerk', 'CPU占比', '失败归因']
    lines.append('| ' + ' | '.join(header) + ' |')
    lines.append('|' + '|'.join(['---'] * len(header)) + '|')
    for row in rows:
        if scenario is not None and row['scenario'] != scenario:
            continue
        agg = row['agg']
        failures = ', '.join(f'{k}×{v}' for k, v in sorted(row['failures'].items()))
        lines.append('| {} | {:.0%} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
            ARM_LABELS.get(row['arm'], row['arm']),
            row['success_rate'],
            _fmt(row['reach_mean'], 1),
            _fmt(agg['traveled_m']['mean'], 2),
            _fmt(agg['min_clearance_m']['min'], 2),
            _fmt(agg['collision_count']['mean'], 1),
            _fmt(agg['stopped_fraction']['mean'], 2),
            _fmt(agg['angular_jerk']['mean'], 2),
            _fmt(agg['cpu_time_ratio']['mean'], 2),
            failures or '—',
        ))
    return '\n'.join(lines)


def arm_totals(rows):
    """跨场景汇总每个臂：加权成功率 + 平均到达时间。"""
    totals = {}
    for row in rows:
        item = totals.setdefault(row['arm'], {'runs': 0, 'success': 0,
                                              'times': []})
        item['runs'] += row['runs']
        item['success'] += round(row['success_rate'] * row['runs'])
        if row['reach_mean'] is not None:
            item['times'].append(row['reach_mean'])
    result = {}
    for arm, item in totals.items():
        result[arm] = {
            'runs': item['runs'],
            'success_rate': (item['success'] / item['runs']) if item['runs'] else 0.0,
            'reach_mean': (sum(item['times']) / len(item['times'])
                           if item['times'] else None),
        }
    return result


def common_time_compare(rows, arm_a, arm_b):
    """只在两臂都跑过的场景上比较平均到达时间。

    各臂覆盖的场景不同（例如 hybrid 只在 detour/obstacle/maze 上跑），
    直接用跨场景均值会把迷宫这类长路线场景算进去，得出误导性结论。
    """
    means = {}
    for r in rows:
        if r['reach_mean'] is None:
            continue
        means.setdefault(r['arm'], {})[r['scenario']] = r['reach_mean']
    a = means.get(arm_a, {})
    b = means.get(arm_b, {})
    common = sorted(s for s in a if s in b)
    if not common:
        return None
    return (sum(a[s] for s in common) / len(common),
            sum(b[s] for s in common) / len(common),
            common)


def build_conclusion(rows):
    totals = arm_totals(rows)
    ordered = [a for a in ARM_ORDER if a in totals]
    if not ordered:
        return ['（没有结果数据）']

    lines = ['### 总体', '']
    lines.append('| 臂 | 运行次数 | 成功率 | 场景平均到达时间(s) |')
    lines.append('|---|---|---|---|')
    for arm in ordered:
        t = totals[arm]
        lines.append('| {} | {} | {:.0%} | {} |'.format(
            ARM_LABELS.get(arm, arm), t['runs'], t['success_rate'],
            _fmt(t['reach_mean'], 1)))
    lines.append('')

    lines.append('### 自动结论')
    lines.append('')
    if 'vision' in totals and 'nav2' in totals:
        v, n = totals['vision'], totals['nav2']
        lines.append(
            f'- 成功率（全部场景）：纯 OpenCV {v["success_rate"]:.0%} vs Nav2 '
            f'{n["success_rate"]:.0%}。')
        cmp = common_time_compare(rows, 'vision', 'nav2')
        if cmp:
            v_t, n_t, common = cmp
            faster = '纯 OpenCV' if v_t < n_t else 'Nav2'
            lines.append(
                f'- 到达时间（仅共同场景 {"、".join(common)}）：纯 OpenCV '
                f'{v_t:.1f}s vs Nav2 {n_t:.1f}s，{faster} 更快。')
        lines.append(
            '- 路程差异来自路线策略：反应式臂边避障边绕行（无地图），'
            'Nav2 在已知地图上走接近最短路。')
    if 'fused' in totals:
        f = totals['fused']
        lines.append(f'- 视觉+雷达融合成功率 {f["success_rate"]:.0%}，'
                     '用于判断视觉与雷达是否互补。')
    if 'hybrid' in totals and 'nav2' in totals:
        h, n = totals['hybrid'], totals['nav2']
        lines.append(
            f'- 视觉护栏叠加后成功率 {n["success_rate"]:.0%} → '
            f'{h["success_rate"]:.0%}，可直接看出 OpenCV 兜底是否有效。')
        cmp = common_time_compare(rows, 'hybrid', 'nav2')
        if cmp:
            h_t, n_t, common = cmp
            lines.append(
                f'- 视觉护栏开销（仅共同场景 {"、".join(common)}）：Nav2 '
                f'{n_t:.1f}s → 加护栏 {h_t:.1f}s。')

    weak = [(r['scenario'], r['arm']) for r in rows
            if r['arm'] in ('vision', 'fused') and r['success_rate'] < 1.0
            and r['failures'].get('collision')]
    if weak:
        lines.append('- 视觉臂出现碰撞失败的场景：'
                     + '、'.join(f'{s}/{a}' for s, a in weak) + '。')
    else:
        lines.append('- 视觉臂没有出现碰撞类失败。')
    return lines


def build_report(rows):
    lines = ['# OpenCV 反应式避障 vs Nav2 导航基准报告', '']
    lines.append('指标定义见 `tools/benchmark/metrics.py`：')
    lines.append('')
    lines.append('- 成功率：进入终点容差圆，或越过终点线且横向偏差 ≤ lane_tolerance'
                 '（反应式臂不收敛到点，用后者）；')
    lines.append('- 最小间距：所有雷达回波的最小值（越小越危险）；')
    lines.append('- 碰撞：连续 2 帧出现 < 0.13m 回波记一次；')
    lines.append('- 卡死占比：线速度 < 0.01m/s 的采样比例；')
    lines.append('- 角速度 jerk：角速度二阶差分 RMS，衡量控制平滑度；')
    lines.append('- CPU 占比：本次运行全部子进程（含 Gazebo 与启动阶段）累计 '
                 'CPU 时间 / 墙钟时间（多核可 >1），用于比较整套栈的开销；')
    lines.append('')

    scenarios = []
    for row in rows:
        if row['scenario'] not in scenarios:
            scenarios.append(row['scenario'])
    for scenario in scenarios:
        lines.append(f'## 场景 `{scenario}`')
        lines.append('')
        lines.append(format_table(rows, scenario))
        lines.append('')

    lines.append('## 结论')
    lines.append('')
    lines.extend(build_conclusion(rows))
    lines.append('')
    return '\n'.join(lines)


def plot(rows, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('未安装 matplotlib，跳过图表', file=sys.stderr)
        return None

    totals = arm_totals(rows)
    arms = [a for a in ARM_ORDER if a in totals]
    labels = [a for a in arms]
    rates = [totals[a]['success_rate'] * 100 for a in arms]
    times = [totals[a]['reach_mean'] or 0.0 for a in arms]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(labels, rates, color='#4C78A8')
    axes[0].set_title('Success rate (%)')
    axes[0].set_ylim(0, 105)
    axes[1].bar(labels, times, color='#F58518')
    axes[1].set_title('Mean time to goal (s)')
    for ax in axes:
        ax.tick_params(axis='x', rotation=20)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    return path


def main():
    parser = argparse.ArgumentParser(description='基准结果汇总')
    parser.add_argument('--out', default=os.path.join(HERE, 'results'))
    parser.add_argument('--report', default=None, help='报告输出路径')
    parser.add_argument('--chart', action='store_true', help='同时生成柱状图')
    args = parser.parse_args()

    records = load_results(args.out)
    if not records:
        sys.exit(f'{args.out} 下没有 run_*.json，请先运行 run_benchmark.py')

    rows = build_rows(group_records(records))
    report_path = args.report or os.path.join(args.out, 'report.md')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(build_report(rows))
    print(build_report(rows))
    print(f'\n报告已写入 {report_path}')

    if args.chart:
        chart_path = plot(rows, os.path.join(args.out, 'chart.png'))
        if chart_path:
            print(f'图表已写入 {chart_path}')


if __name__ == '__main__':
    main()
