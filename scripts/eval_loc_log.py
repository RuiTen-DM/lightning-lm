#!/usr/bin/env python3
"""
根据 lightning 定位 log 评估定位效果。

只给定位 log 时，评估定位稳不稳（无需真值）：
    - 初始化：方式与耗时
    - 匹配得分（NDT confidence）分布、低分次数
    - 匹配失败次数
    - 与定位内部 LIO 的一致性：相邻两次匹配之间，激光定位的位移与 LIO 的位移之差（跳变）
    - 匹配间隔
    - 警告/错误汇总
再给出建图 log（--ref）时，额外以建图的 LIO 轨迹为参考评估位置误差，
适用于“用建图同一个包做定位测试”，且 log 来自生成该地图的那次建图。

用法：
    ros2 run lightning run_loc_offline --config <cfg> --input_bag <bag> --map_path ./data/new_map/ 2>&1 | tee loc.log
    python3 scripts/eval_loc_log.py loc.log
    python3 scripts/eval_loc_log.py loc.log --ref slam.log
"""

import argparse
import re
from collections import OrderedDict

import numpy as np

# 偶发时正常的警告：按消息内容匹配，超过 LIO 帧数的这个比例才视为异常
BENIGN_WARNINGS = {
    'sync package failed': (0.01, 'IMU 尚未覆盖该帧结束时间，该帧留到下次处理；频繁出现说明 IMU 断流或乱序'),
}


def parse_lio(path):
    """每帧 'LIO get cloud ... end: T' 与随后的 'LIO state: x y z'，建图与定位 log 中都有"""
    lio, end = [], None
    for line in open(path, errors='ignore'):
        m = re.search(r'LIO get cloud at beg: [\d.]+, end: ([\d.]+)', line)
        if m:
            end = float(m.group(1))
            continue

        m = re.search(r'LIO state:\s+(\S+)\s+(\S+)\s+(\S+),', line)
        if m and end is not None:
            lio.append([end, *map(float, m.groups())])
            end = None
    return np.array(lio)


def parse_loc(path):
    """解析定位结果、计数类信息和警告"""
    info = {'matches': [], 'min_init_conf': None, 'init_method': None, 'n_fail': 0, 'n_not_success': 0,
            'n_low_score': 0, 'n_lo_check': 0, 'warnings': OrderedDict()}
    conf, match_lines = np.nan, []

    for idx, line in enumerate(open(path, errors='ignore')):
        m = re.search(r'confidence: ([\d.eE+-]+),', line)
        if m:
            conf = float(m.group(1))
            continue

        m = re.search(r'PGO received LidarLoc \[\s*(\S+)\s+(\S+)\s+(\S+)\], t=([\d.]+)', line)
        if m:
            info['matches'].append([float(m.group(4)), *map(float, m.groups()[:3]), conf])
            match_lines.append(idx)
            continue

        m = re.search(r'min init confidence: ([\d.]+)', line)
        if m:
            info['min_init_conf'] = float(m.group(1))

        # 成功时为 "init with <来源>"，如 fp: start、config pose；尝试和失败的行另有前后缀
        m = re.search(r'\] (init with (?!.* failed, score).+?)\s*$', line)
        if m and info['init_method'] is None:
            info['init_method'] = m.group(1)

        info['n_fail'] += 'localization failed!' in line
        info['n_not_success'] += 'loc success is false' in line
        info['n_low_score'] += '当前lidar_loc分值较低' in line
        info['n_lo_check'] += 'delta_rel_abs_pose is:' in line

        m = re.match(r'([WE])\d{4} [\d:.]+\s+\d+ (\S+:\d+)\] (.*)', line)
        if m:
            key = f'{m.group(1)} {m.group(2)}'
            w = info['warnings'].setdefault(key, {'count': 0, 'sample': m.group(3)[:80], 'lines': []})
            w['count'] += 1
            w['lines'].append(idx)

    info['matches'] = np.array(info['matches'])
    # 第二次匹配之前视为启动阶段（第一帧 PGO 处理时 DR 尚未就绪，会有一些警告）
    info['startup_end_line'] = match_lines[1] if len(match_lines) > 1 else None
    return info


def stats_str(e, unit=100, suffix='cm'):
    a = np.abs(e)
    return (f'mean {a.mean() * unit:6.1f} {suffix}  rmse {np.sqrt((e ** 2).mean()) * unit:6.1f} {suffix}  '
            f'median {np.median(a) * unit:6.1f} {suffix}  p95 {np.percentile(a, 95) * unit:6.1f} {suffix}  '
            f'max {a.max() * unit:6.1f} {suffix}')


def interp_pos(traj, t):
    return np.stack([np.interp(t, traj[:, 0], traj[:, k]) for k in (1, 2, 3)], 1)


def kabsch_rotation(a, b):
    """求旋转 R 使 R @ a_i 与 b_i 最接近"""
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1, 1, d]) @ u.T


def eval_stability(info, lio, args):
    loc = info['matches']
    flags = []

    print('=' * 20, '定位稳定性', '=' * 20)
    if len(loc) == 0:
        print('log 中没有定位结果（PGO received LidarLoc），定位未初始化成功')
        return

    t0 = min(lio[0, 0], loc[0, 0]) if len(lio) else loc[0, 0]
    t_end = max(lio[-1, 0], loc[-1, 0]) if len(lio) else loc[-1, 0]
    print(f'数据时长 {t_end - t0:.1f} s, 激光匹配 {len(loc)} 次')
    print(f'初始化: {info["init_method"] or "未知"}, 第一次匹配在数据开始后 {loc[0, 0] - t0:.1f} s')

    # 匹配得分
    c = loc[:, 4]
    th = info['min_init_conf'] if info['min_init_conf'] is not None else 1.8
    n_low = int(np.sum(c < th))
    print(f'\n[匹配得分] min {np.nanmin(c):.2f}  p10 {np.nanpercentile(c, 10):.2f}  median {np.nanmedian(c):.2f}  '
          f'max {np.nanmax(c):.2f}')
    print(f'  低于初始化阈值 {th} 的次数: {n_low}，PGO 报告低分次数: {info["n_low_score"]}')
    if n_low or info['n_low_score']:
        flags.append('存在低分匹配')

    # 匹配失败
    print(f'\n[匹配失败] localization failed: {info["n_fail"]}，loc success is false: {info["n_not_success"]}')
    if info['n_fail'] or info['n_not_success']:
        flags.append('存在匹配失败')

    # 与 LIO 的一致性：相邻两次匹配之间的位移差
    print(f'\n[与 LIO 的一致性] 相邻两次匹配间，激光定位位移与 LIO 位移之差（跳变）')
    if len(lio) >= 2 and len(loc) >= 4:
        sel = (loc[:, 0] >= lio[0, 0]) & (loc[:, 0] <= lio[-1, 0])
        loc_in = loc[sel]
        d_loc = np.diff(loc_in[:, 1:4], axis=0)
        d_lio = np.diff(interp_pos(lio, loc_in[:, 0]), axis=0)
        # LIO 在自身里程计坐标系，先估计它到地图坐标系的旋转
        rot = kabsch_rotation(d_lio, d_loc)
        jump = d_loc - d_lio @ rot.T
        jump_h = np.linalg.norm(jump[:, :2], axis=1)
        step = np.linalg.norm(d_lio, axis=1)
        print(f'  水平: {stats_str(jump_h)}')
        print(f'  高度: {stats_str(jump[:, 2])}')
        print(f'  相对步长: median {np.median(jump_h / np.maximum(step, 0.1)) * 100:.1f}%  '
              f'（相邻匹配平均间距 {step.mean():.2f} m）')
        n_jump = int(np.sum(jump_h > args.jump_th))
        print(f'  水平跳变超过 {args.jump_th} m 的次数: {n_jump}，lightning 自身判定 LO 不一致的次数: {info["n_lo_check"]}')
        for i in np.argsort(-jump_h)[:3]:
            print(f'  跳变最大: t={loc_in[i + 1, 0] - t0:6.1f}s  水平 {jump_h[i] * 100:5.1f} cm  '
                  f'高度 {jump[i, 2] * 100:5.1f} cm  confidence={loc_in[i + 1, 4]:.2f}')
        if n_jump or info['n_lo_check']:
            flags.append('存在与 LIO 不一致的跳变')
    else:
        print('  log 中 LIO 状态或匹配次数不足，跳过')

    # 匹配间隔
    gaps = np.diff(loc[:, 0])
    n_gap = int(np.sum(gaps > args.gap_th))
    print(f'\n[匹配间隔] median {np.median(gaps):.2f} s  max {gaps.max():.2f} s，超过 {args.gap_th} s 的次数: {n_gap}')
    if len(lio):
        tail = t_end - loc[-1, 0]
        print(f'  最后一次匹配到数据结束: {tail:.1f} s')
    if n_gap:
        flags.append('存在长时间没有匹配')

    # 警告/错误
    print('\n[警告/错误]')
    startup_end = info['startup_end_line']
    if not info['warnings']:
        print('  无')
    for key, w in info['warnings'].items():
        n_run = sum(1 for i in w['lines'] if startup_end is None or i > startup_end)
        stage = '（均在启动阶段）' if n_run == 0 else f'（启动后 {n_run} 次）'
        print(f'  {key:<28} x{w["count"]:<4}{stage} {w["sample"]}')

        benign = next((v for k, v in BENIGN_WARNINGS.items() if k in w['sample']), None)
        if benign:
            print(f'      说明: {benign[1]}')
            if n_run > benign[0] * max(len(lio), 1):
                flags.append(f'{key} 出现过于频繁')
        elif n_run:
            flags.append(f'启动后出现 {key}')

    print('\n结论:', '未发现异常' if not flags else '需关注 -> ' + '；'.join(flags))


def eval_accuracy(loc, ref):
    print('\n' + '=' * 20, '相对建图轨迹的位置误差', '=' * 20)
    loc = loc[(loc[:, 0] >= ref[0, 0]) & (loc[:, 0] <= ref[-1, 0])]
    if len(ref) < 2 or len(loc) == 0:
        print('建图 log 与定位结果在时间上没有重叠，跳过')
        return

    p_ref = interp_pos(ref, loc[:, 0])
    err = loc[:, 1:4] - p_ref

    # 水平误差分解为沿轨迹方向与横向
    v = np.stack([np.gradient(p_ref[:, k], loc[:, 0]) for k in (0, 1)], 1)
    v /= np.linalg.norm(v, axis=1, keepdims=True) + 1e-9
    along = np.sum(err[:, :2] * v, 1)
    cross = err[:, 0] * -v[:, 1] + err[:, 1] * v[:, 0]

    path_len = np.sum(np.linalg.norm(np.diff(ref[:, 1:4], axis=0), axis=1))
    print(f'参考帧 {len(ref)}, 定位结果 {len(loc)}, 路径长度 {path_len:.1f} m')
    print(f'3D      {stats_str(np.linalg.norm(err, axis=1))}')
    print(f'水平    {stats_str(np.linalg.norm(err[:, :2], axis=1))}')
    print(f'沿轨迹  {stats_str(along)}')
    print(f'横向    {stats_str(cross)}')
    print(f'高度    {stats_str(err[:, 2])}')
    for i in np.argsort(-np.linalg.norm(err, axis=1))[:3]:
        print(f'  误差最大: t={loc[i, 0] - loc[0, 0]:6.1f}s  err={np.linalg.norm(err[i]) * 100:5.1f} cm  '
              f'confidence={loc[i, 4]:.2f}')


def main():
    parser = argparse.ArgumentParser(description='根据 lightning 定位 log 评估定位效果')
    parser.add_argument('loc_log', help='run_loc_offline / run_loc_online 的 log')
    parser.add_argument('--ref', help='生成该地图的那次建图 log，给出时额外评估位置误差')
    parser.add_argument('--jump-th', type=float, default=0.5, help='水平跳变阈值(m)，默认同 lidar_loc_odom_th')
    parser.add_argument('--gap-th', type=float, default=3.0, help='匹配间隔阈值(s)')
    args = parser.parse_args()

    info = parse_loc(args.loc_log)
    eval_stability(info, parse_lio(args.loc_log), args)

    if args.ref and len(info['matches']):
        eval_accuracy(info['matches'], parse_lio(args.ref))


if __name__ == '__main__':
    main()
