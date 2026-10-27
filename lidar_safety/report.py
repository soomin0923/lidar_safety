"""평가 결과(trials.csv)를 표(summary.md)와 그래프(clearance.png)로 정리한다.

  ros2 run lidar_safety eval_report ~/ros2_ws/eval_results/<시각>/trials.csv
"""
import csv
import pathlib
import statistics
import sys

FAULT_ORDER = ['none', 'delay', 'zero', 'freeze', 'noise']
MONITOR_ORDER = ['off', 'on', 'graded']
MONITOR_LABELS = {'off': 'monitor off', 'on': 'monitor on: stop', 'graded': 'monitor graded: slow on noise'}

# 차트 색: 조건마다 고정된 범주색(조건이 빠져도 색이 바뀌지 않음), 충돌은 상태색(critical)으로 따로 둔다
COLORS = {'off': '#2a78d6', 'on': '#eb6834', 'graded': '#1baf7a'}
CRITICAL = '#d03b3b'
INK, INK_2, MUTED, GRID, AXIS, SURFACE = '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7', '#fcfcfb'


def load(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r['speed'] = float(r['speed'])
        r['final_clearance'] = float(r['final_clearance'])
    return rows


def mean_of(rows, key):
    vals = [float(r[key]) for r in rows if r[key] != '']
    return statistics.mean(vals) if vals else None


def groups(rows):
    faults = [f for f in FAULT_ORDER if any(r['fault'] == f for r in rows)]
    faults += sorted({r['fault'] for r in rows} - set(faults))
    speeds = sorted({r['speed'] for r in rows})
    monitors = [m for m in MONITOR_ORDER if any(r['monitor'] == m for r in rows)]
    return faults, speeds, monitors


def summary_table(rows):
    faults, speeds, monitors = groups(rows)
    fmt = lambda x, unit: '-' if x is None else f'{x:.2f}{unit}'
    lines = ['| 결함 | 속도 | 모니터 | 충돌 | 최종 여유거리 평균 (최소) | 감지 지연 평균 | 결함 후 정지 판단까지 평균 |',
             '|---|---|---|---|---|---|---|']
    for speed in speeds:
        for fault in faults:
            for mon in monitors:
                g = [r for r in rows if r['fault'] == fault and r['speed'] == speed and r['monitor'] == mon]
                if not g:
                    continue
                hits = sum(r['outcome'] == 'collision' for r in g)
                timeouts = sum(r['outcome'] == 'timeout' for r in g)
                clear = [r['final_clearance'] for r in g]
                note = f' (시간 초과 {timeouts})' if timeouts else ''
                lines.append(
                    f'| {fault} | {speed:.2f} m/s | {mon} | {"**" if hits else ""}{hits}/{len(g)}{"**" if hits else ""}{note} '
                    f'| {statistics.mean(clear):.2f} m ({min(clear):.2f}) '
                    f'| {fmt(mean_of(g, "detect_latency_s"), " s")} | {fmt(mean_of(g, "stop_latency_s"), " s")} |')
    return '\n'.join(lines)


def plot(rows, png_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    faults, speeds, monitors = groups(rows)
    fig, axes = plt.subplots(1, len(speeds), figsize=(max(5.2 * len(speeds), 8.0), 4.2), sharey=True, squeeze=False)
    fig.patch.set_facecolor(SURFACE)
    gap = 0.04  # 막대 사이 간격
    bar_w = (0.76 - gap * (len(monitors) - 1)) / len(monitors)
    inject = mean_of(rows, 'inject_clearance')
    ymax = max([r['final_clearance'] for r in rows] + [inject or 0]) * 1.15 + 0.05

    for ax, speed in zip(axes[0], speeds):
        ax.set_facecolor(SURFACE)
        for mi, mon in enumerate(monitors):
            offset = (mi - (len(monitors) - 1) / 2) * (bar_w + gap)
            for fi, fault in enumerate(faults):
                g = [r for r in rows if r['fault'] == fault and r['speed'] == speed and r['monitor'] == mon]
                if not g:
                    continue
                x = fi + offset
                ax.bar(x, statistics.mean(r['final_clearance'] for r in g), width=bar_w,
                       color=COLORS[mon], zorder=2)
                for r in g:
                    hit = r['outcome'] == 'collision'
                    ax.plot(x, r['final_clearance'], marker='X' if hit else 'o', markersize=9 if hit else 5,
                            color=CRITICAL if hit else INK, markeredgecolor=SURFACE, markeredgewidth=1.2,
                            zorder=3, linestyle='none', clip_on=False)
                hits = sum(r['outcome'] == 'collision' for r in g)
                if hits:
                    ax.annotate(f'collision {hits}/{len(g)}', (x, 0), xytext=(0, 12), textcoords='offset points',
                                ha='center', fontsize=8, color=INK_2)
        if inject is not None:
            ax.axhline(inject, color=MUTED, linewidth=0.8, linestyle=(0, (4, 3)), zorder=1)
            ax.text(1.0, inject + ymax * 0.01, 'fault injected', transform=ax.get_yaxis_transform(),
                    ha='right', va='bottom', fontsize=8, color=MUTED)
        ax.set_title(f'{speed:.2f} m/s', color=INK, fontsize=11, loc='left')
        ax.set_xticks(range(len(faults)))
        ax.set_xticklabels(faults, color=INK_2)
        ax.set_ylim(0, ymax)
        ax.tick_params(colors=MUTED, length=0)
        ax.grid(axis='y', color=GRID, linewidth=0.8, zorder=0)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(AXIS)

    axes[0][0].set_ylabel('Final clearance to wall (m)', color=INK_2)
    handles = [Patch(color=COLORS[m], label=f'{MONITOR_LABELS.get(m, m)} (mean)') for m in monitors]
    handles += [Line2D([], [], marker='o', color=INK, linestyle='none', markersize=5, label='each trial'),
                Line2D([], [], marker='X', color=CRITICAL, linestyle='none', markersize=8, label='collision')]
    fig.legend(handles=handles, loc='upper center', ncol=min(len(handles), 3), frameon=False,
               labelcolor=INK_2, bbox_to_anchor=(0.5, 1.0))
    angle = rows[0].get('wall_angle_deg') or '0'
    wall = 'the wall' if float(angle) == 0 else f'a wall tilted {float(angle):.0f}°'
    fig.suptitle(f'Lidar fault injected at {inject or 0:.1f} m clearance while driving at {wall}',
                 color=INK, fontsize=12, x=0.01, ha='left', y=1.08 if len(handles) <= 3 else 1.14)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150, bbox_inches='tight', facecolor=SURFACE)
    plt.close(fig)


def write_report(csv_path):
    csv_path = pathlib.Path(csv_path)
    rows = load(csv_path)
    if not rows:
        print('no trials in', csv_path)
        return
    png = csv_path.with_name('clearance.png')
    plot(rows, png)
    md = csv_path.with_name('summary.md')
    md.write_text(
        '# 라이다 결함 주입 평가 결과\n\n'
        f'시도 {len(rows)}회. 벽까지 여유거리 약 {mean_of(rows, "inject_clearance") or 0:.1f}m 지점에서 '
        '주행 중 결함을 주입했다. 여유거리는 로봇 몸체 앞모서리에서 벽면까지의 실제 거리(시뮬레이터 정답)다. '
        '결함 none은 주입 시점부터 장애물 정지까지의 시간을 정지 판단 시간으로 적었다.\n\n'
        f'![final clearance]({png.name})\n\n{summary_table(rows)}\n')
    print(summary_table(rows))
    print(f'\n{md}\n{png}')


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    write_report(sys.argv[1])


if __name__ == '__main__':
    main()
