#!/usr/bin/env python3
"""
汇总 run_lio_online 记录的结果（lio_online/record_dir 下的 status.csv、stops.csv、traj_*.csv）。

    python3 scripts/lio_record_summary.py <记录目录> [--cluster 1.0]

输出：
    - 启动次数（每次启动在 status.csv 里有一行 "# start"，多次说明重启过）和本次运行时长
    - 最新状态，以及运行以来 CPU、内存、每帧耗时、局部地图的最大值
    - 停靠点：把位置相近（--cluster 米内）的停稳记录归为同一个点，列出次数、首末时间、首末位置之差。
      往复运动时同一停靠点的位置随时间的变化，就是里程计的漂移
      每次重启后里程计从当时的位置重新开始，坐标系不同，所以停靠点按每次运行分开统计

也可以直接在板子上运行，只把文字结果传回来：
    ssh -p <端口> <用户>@<板子> python3 lightning_lio/lio_record_summary.py lightning_lio/records
"""

import argparse
import csv
import glob
import os
from datetime import datetime

import numpy as np


def read_status(path):
    starts, rows = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('# start'):
                starts.append(line[len('# start '):])
            elif line and not line.startswith('time'):
                rows.append(line.split(','))
    return starts, rows


def main():
    parser = argparse.ArgumentParser(description='汇总 run_lio_online 的记录')
    parser.add_argument('dir')
    parser.add_argument('--cluster', type=float, default=1.0, help='停靠点归类的距离（米）；漂移超过它时同一停靠点会被拆成相邻的两个')
    args = parser.parse_args()

    starts, rows = read_status(os.path.join(args.dir, 'status.csv'))
    print(f'启动 {len(starts)} 次: ' + ', '.join(starts[-5:]) + (' ...' if len(starts) > 5 else ''))
    if rows:
        last = rows[-1]
        t_start = datetime.strptime(starts[-1].split(' pid')[0], '%Y-%m-%d %H:%M:%S') if starts else None
        t_last = datetime.strptime(last[0], '%Y-%m-%d %H:%M:%S')
        if t_start:
            print(f'本次已运行 {(t_last - t_start).total_seconds() / 3600:.2f} 小时（到 {last[0]}）')
        print(f'最新: 处理 {last[1]}/{last[2]} 帧，丢 {last[3]} 帧，每帧 {last[4]} ms（最大 {last[5]}），'
              f'局部地图 {last[6]} 格，内存 {last[7]} MB，CPU {last[8]}%，位置 ({last[9]}, {last[10]}, {last[11]})')
        v = np.array([[float(x) for x in r[4:9]] for r in rows])
        print(f'运行以来最大: 每帧 {v[:, 1].max():.1f} ms，局部地图 {v[:, 2].max():.0f} 格，'
              f'内存 {v[:, 3].max():.1f} MB，CPU {v[:, 4].max():.0f}%')

    stops = []
    with open(os.path.join(args.dir, 'stops.csv')) as f:
        for r in csv.DictReader(f):
            stops.append((r['time'], np.array([float(r['x']), float(r['y']), float(r['z'])]), float(r['path_m'])))
    print(f'\n停稳 {len(stops)} 次，共走 {sum(s[2] for s in stops) / 1000:.2f} km')

    # 按启动时间把停稳记录分到各次运行
    start_times = [s.split(' pid')[0] for s in starts]
    sessions = {}
    for s in stops:
        k = max([i for i, t in enumerate(start_times) if t <= s[0]], default=0)
        sessions.setdefault(k, []).append(s)

    for k in sorted(sessions):
        print(f'  第 {k + 1} 次运行（{start_times[k] if start_times else "-"} 启动）:')
        clusters = []  # 每个停靠点：记录列表
        for s in sessions[k]:
            for c in clusters:
                if np.linalg.norm(c[0][1][:2] - s[1][:2]) < args.cluster:
                    c.append(s)
                    break
            else:
                clusters.append([s])
        for i, c in enumerate(sorted(clusters, key=len, reverse=True)):
            p = np.array([s[1] for s in c])
            d = p[-1] - p[0]
            print(f'    停靠点 {i}: {len(c)} 次，{c[0][0]} ~ {c[-1][0]}，位置约 ({p[:, 0].mean():.2f}, '
                  f'{p[:, 1].mean():.2f}, {p[:, 2].mean():.2f})，末次减首次 ({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}) m，'
                  f'范围 xy {np.ptp(p[:, 0]):.3f}/{np.ptp(p[:, 1]):.3f} z {np.ptp(p[:, 2]):.3f} m')

    traj = sorted(glob.glob(os.path.join(args.dir, 'traj_*.csv')))
    n = sum(max(0, sum(1 for _ in open(t)) - 1) for t in traj)
    size = sum(os.path.getsize(t) for t in traj) / 1024
    print(f'\n轨迹文件 {len(traj)} 个（{os.path.basename(traj[0]) if traj else "-"} ~ '
          f'{os.path.basename(traj[-1]) if traj else "-"}），共 {n} 行，{size:.0f} KB')


if __name__ == '__main__':
    main()
