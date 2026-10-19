#!/usr/bin/env python3
"""
汇总 run_lio_online 记录的结果（lio_online/record_dir 下的 csv），文字输出。和 lio_record_plot.py 用同一套分析。

    python3 scripts/lio_record_summary.py <记录目录>

输出：
    - 启动次数（每次启动在 status.csv 里有一行 "# start"，多次说明重启过）和本次运行时长
    - 最新状态，以及运行以来 CPU、内存、每帧耗时、局部地图的最大值
    - 每次运行的漂移指标（每次启动里程计从当时的位置重新开始，坐标系不同，所以分开统计）：
        静止时的漂移：IMU 判断真正静止期间，里程计位置变了多少（still.csv）
        来回一致性：每一趟经过同一沿轨道位置时的横向偏移和高度，相对最早经过那里的一趟的偏差

也可以直接在板子上运行，只把文字结果传回来：
    ssh -p <端口> <用户>@<板子> python3 lightning_lio/lio_record_summary.py lightning_lio/records
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np

sys.dont_write_bytecode = True  # 不在脚本目录下留 __pycache__
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lio_record_plot import analyze  # noqa: E402


def fmt(t):
    return datetime.fromtimestamp(t).strftime('%m-%d %H:%M')


def main():
    parser = argparse.ArgumentParser(description='汇总 run_lio_online 的记录')
    parser.add_argument('dir')
    args = parser.parse_args()

    starts, sessions = analyze(args.dir)
    print(f'启动 {len(starts)} 次: ' + ', '.join(fmt(t) for t in starts[-5:]) + (' ...' if len(starts) > 5 else ''))
    if not sessions:
        print('没有数据')
        return

    st = sessions[-1]['status']
    if len(st):
        last = st[-1]
        print(f'本次已运行 {(last[0] - starts[-1]) / 3600:.2f} 小时（到 {datetime.fromtimestamp(last[0]):%Y-%m-%d %H:%M}）')
        print(f'最新: 处理 {last[1]:.0f}/{last[2]:.0f} 帧，丢 {last[3]:.0f} 帧，每帧 {last[4]:.1f} ms（最大 {last[5]:.1f}），'
              f'局部地图 {last[6]:.0f} 格，内存 {last[7]:.1f} MB，CPU {last[8]:.0f}%，位置 ({last[9]:.2f}, {last[10]:.2f}, {last[11]:.2f})')
        if last[1] == 0:
            print('  注意：这次运行一帧数据都没收到，检查雷达驱动和 DDS（见 doc/lio_online.md 开机顺序）')
        print(f'运行以来最大: 每帧 {st[:, 5].max():.1f} ms，局部地图 {st[:, 6].max():.0f} 格，'
              f'内存 {st[:, 7].max():.1f} MB，CPU {st[:, 8].max():.0f}%')

    for S in sessions:
        print(f'\n第 {S["index"] + 1} 次运行（{fmt(S["start"])} 启动）: 停车 {len(S["stops"])} 次，'
              f'走过 {S["stops"][:, 4].sum() / 1000 if len(S["stops"]) else 0:.2f} km')

        sl, final = S['still'], S['still_final']
        if final.any():
            dur = sl[final, 1]
            norm = S['still_norm'][final]
            rates = S['still_rates']
            k = int(np.argmax(norm))
            print(f'  静止漂移: {final.sum()} 段，共 {dur.sum() / 3600:.2f} 小时；'
                  + (f'速率中位 {np.median(rates) * 100:.1f} cm/小时（静止 1 分钟以上的 {len(rates)} 段），' if len(rates) else '')
                  + f'单段最大 {norm[k] * 100:.1f} cm（静止 {dur[k] / 60:.0f} 分钟）')
        else:
            print('  静止漂移: 还没有记录（需要新版本程序，静止 30 秒以上才记）')

        pr = S['pass_rows']
        if pr:
            dl = np.array([r['dl'] for r in pr]) * 100
            dz = np.array([r['dz'] for r in pr]) * 100
            print(f'  来回一致性: {len(S["passes"])} 趟；最近一趟（{fmt(pr[-1]["t"])}）相对最早经过处 横向 {dl[-1]:+.1f} cm、'
                  f'高度 {dz[-1]:+.1f} cm；所有趟里最大 横向 {np.abs(dl).max():.1f} cm、高度 {np.abs(dz).max():.1f} cm')
        else:
            print(f'  来回一致性: {len(S["passes"])} 趟，还没有两趟以上重合的轨迹')


if __name__ == '__main__':
    main()
