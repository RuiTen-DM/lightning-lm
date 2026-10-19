#!/usr/bin/env python3
"""
把 run_lio_online 的记录目录（status.csv、stops.csv、traj_*.csv、可选的 local_map.pcd）画成一个 html 页面。
页面是单个文件，不依赖网络，浏览器直接打开。

    python3 scripts/lio_record_plot.py <记录目录> [-o lio_record.html]

页面内容（按每次运行分开，默认最后一次；每次启动里程计从当时的位置重新开始，各次运行的坐标系不同）：
    - 俯视图：局部地图（灰色，按高度深浅）、轨迹、停车位置
    - 沿轨道位置、高度随时间的变化
    - 静止时的漂移：IMU 判断真正静止期间，里程计位置变了多少（still.csv）
    - 来回一致性：轨道是固定的直线，每一趟经过同一沿轨道位置时，横向偏移和高度应该相同；
      各趟相对最早经过该处那一趟的偏差随时间变大，就是漂移（航向漂移表现为离起点越远横向偏得越多）
    - CPU、内存、每帧耗时、局部地图格数、丢帧随时间的变化
图上可以滚轮缩放、拖动平移、双击复原，鼠标悬停看数值。地址后加 #序号（从 0 开始）可以直接打开某次运行。
"""

import argparse
import csv
import glob
import html
import json
import os
import struct
import time
from datetime import datetime

import numpy as np


def parse_local(s):
    return time.mktime(datetime.strptime(s, '%Y-%m-%d %H:%M:%S').timetuple())


def read_status(path):
    starts, rows = [], []
    if not os.path.exists(path):
        return starts, rows
    for line in open(path):
        line = line.strip()
        if line.startswith('# start'):
            starts.append(parse_local(line[len('# start '):].split(' pid')[0]))
        elif line and not line.startswith('time'):
            f = line.split(',')
            rows.append([parse_local(f[0])] + [float(x) for x in f[1:]])
    return starts, np.array(rows) if rows else np.zeros((0, 13))


def lzf_decompress(data, out_len):
    out = bytearray(out_len)
    i = o = 0
    n = len(data)
    while i < n:
        c = data[i]
        i += 1
        if c < 32:  # 字面量
            c += 1
            out[o:o + c] = data[i:i + c]
            i += c
            o += c
        else:  # 回溯引用
            length = c >> 5
            ref = o - ((c & 0x1f) << 8) - 1
            if length == 7:
                length += data[i]
                i += 1
            ref -= data[i]
            i += 1
            length += 2
            if ref + length <= o:
                out[o:o + length] = out[ref:ref + length]
                o += length
            else:  # 有重叠，逐字节
                for _ in range(length):
                    out[o] = out[ref]
                    o += 1
                    ref += 1
    return bytes(out)


def read_pcd_xyz(path):
    """读 pcd 的 xyz，支持 ascii / binary / binary_compressed"""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if not line or line.startswith('#'):
                continue
            k, v = line.split(' ', 1)
            header[k] = v.split()
            if k == 'DATA':
                break
        body = f.read()
    fields, sizes, types = header['FIELDS'], [int(x) for x in header['SIZE']], header['TYPE']
    counts = [int(x) for x in header.get('COUNT', ['1'] * len(fields))]
    n = int(header['POINTS'][0])
    dt = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('I', 4): '<i4'}
    fmt = header['DATA'][0]
    if fmt == 'ascii':
        a = np.loadtxt(body.decode().splitlines(), ndmin=2)
        return a[:, :3].astype(np.float32)
    if fmt == 'binary':
        dtype = np.dtype([(f, dt[(t, s)], (c,)) for f, s, t, c in zip(fields, sizes, types, counts)])
        a = np.frombuffer(body[:n * dtype.itemsize], dtype=dtype)
        return np.stack([a['x'][:, 0], a['y'][:, 0], a['z'][:, 0]], 1)
    comp_size, raw_size = struct.unpack('<II', body[:8])
    raw = lzf_decompress(body[8:8 + comp_size], raw_size)
    cols, off = {}, 0
    for f, s, t, c in zip(fields, sizes, types, counts):  # 压缩格式按字段分块存放
        cols[f] = np.frombuffer(raw[off:off + n * s * c], dtype=dt[(t, s)])[::c]
        off += n * s * c
    return np.stack([cols['x'], cols['y'], cols['z']], 1).astype(np.float32)


def decimate(a, max_n):
    if len(a) <= max_n:
        return a
    return a[np.linspace(0, len(a) - 1, max_n).astype(int)]


def r2(a, d=2):
    return np.round(np.asarray(a, dtype=float), d).tolist()


def read_still(path):
    """still.csv：每段静止的起点、时长、起点位置、位移、航向变化、是否为该段最后一行"""
    rows = []
    if os.path.exists(path):
        with open(path) as fh:
            for r in csv.DictReader(fh):
                rows.append([parse_local(r['start']), float(r['duration_s']), float(r['x0']), float(r['y0']),
                             float(r['z0']), float(r['dx']), float(r['dy']), float(r['dz']), float(r['dyaw_deg']),
                             float(r['final'])])
    return np.array(rows) if rows else np.zeros((0, 10))


def split_passes(t, along, min_len=10.0):
    """把轨迹切成一趟一趟：相邻采样（约 1 秒）沿轨道走了 0.1 m 以上且方向相同，算同一趟；走过不到 min_len 米的不要"""
    if len(t) < 3:
        return []
    ds, dt = np.diff(along), np.diff(t)
    dirn = np.where((np.abs(ds) > 0.1) & (dt < 5), np.sign(ds), 0)
    passes, i, n = [], 0, len(ds)
    while i < n:
        if dirn[i] == 0:
            i += 1
            continue
        j = i
        while j + 1 < n and dirn[j + 1] == dirn[i]:
            j += 1
        idx = np.arange(i, j + 2)
        if abs(along[idx[-1]] - along[idx[0]]) >= min_len:
            passes.append(idx)
        i = j + 1
    return passes


def pass_consistency(t, along, lat, z, bin_size=2.0):
    """每一趟在各个 2 m 格子里的横向、高度，减去最早经过这个格子的那一趟，取中位数"""
    passes = split_passes(t, along)
    ref, rows = {}, []
    for idx in passes:
        b = np.floor(along[idx] / bin_size).astype(int)
        dl, dz, new = [], [], {}
        for bi in np.unique(b):
            m = b == bi
            lv, zv = lat[idx][m].mean(), z[idx][m].mean()
            if bi in ref:
                dl.append(lv - ref[bi][0])
                dz.append(zv - ref[bi][1])
            else:
                new[bi] = (lv, zv)
        ref.update(new)
        if dl:
            rows.append({'t': float(t[idx].mean()), 'dl': float(np.median(dl)), 'dz': float(np.median(dz)),
                         'n': len(dl), 'len': float(abs(along[idx[-1]] - along[idx[0]])),
                         'dir': int(np.sign(along[idx[-1]] - along[idx[0]]))})
    return passes, rows


def analyze(d):
    """读一个记录目录，按每次运行分开整理，供画图和文字汇总共用"""
    starts, status = read_status(os.path.join(d, 'status.csv'))
    if not starts:
        starts = [status[0, 0] if len(status) else 0.0]

    traj = []
    for f in sorted(glob.glob(os.path.join(d, 'traj_*.csv'))):
        with open(f) as fh:
            for r in csv.DictReader(fh):
                traj.append([float(r['time']), float(r['x']), float(r['y']), float(r['z'])])
    traj = np.array(traj) if traj else np.zeros((0, 4))

    stops = []
    if os.path.exists(os.path.join(d, 'stops.csv')):
        with open(os.path.join(d, 'stops.csv')) as fh:
            for r in csv.DictReader(fh):
                stops.append([parse_local(r['time']), float(r['x']), float(r['y']), float(r['z']), float(r['path_m'])])
    stops = np.array(stops) if stops else np.zeros((0, 5))
    still = read_still(os.path.join(d, 'still.csv'))

    # 地图属于哪次运行，按 local_map_pose*.txt 里记的存图时间判断（没收到数据的运行不会存图，不能按先后顺序猜）
    maps = {}
    for name, pose_name in (('local_map.pcd', 'local_map_pose.txt'), ('local_map_prev.pcd', 'local_map_pose_prev.txt')):
        p, pp = os.path.join(d, name), os.path.join(d, pose_name)
        if not (os.path.exists(p) and os.path.exists(pp)):
            continue
        lines = [ln for ln in open(pp) if ln.strip() and not ln.startswith('#')]
        if not lines:
            continue
        saved = parse_local(' '.join(lines[-1].split()[:2]))
        sess = max([i for i, t0 in enumerate(starts) if t0 <= saved], default=-1)
        if sess >= 0 and sess not in maps:
            pts = read_pcd_xyz(p)
            # 俯视图只需要稀疏一些的点：0.5 m 格子里留最高的点
            key = np.floor(pts[:, :2] / 0.5).astype(np.int64)
            k = key[:, 0] * 1000003 + key[:, 1]
            order = np.lexsort((-pts[:, 2], k))
            k, pts = k[order], pts[order]
            first = np.r_[True, k[1:] != k[:-1]]
            maps[sess] = (pts[first], len(order))

    sessions = []
    bounds = list(starts) + [float('inf')]
    for i in range(len(starts)):
        lo, hi = bounds[i], bounds[i + 1]
        pick = lambda a: a[(a[:, 0] >= lo) & (a[:, 0] < hi)] if len(a) else a
        t, s, st, sl = pick(traj), pick(stops), pick(status), pick(still)
        if len(t) == 0 and len(st) == 0:
            continue

        # 沿轨道方向：轨迹水平位置的主方向
        axis = np.array([0.0, 1.0])
        if len(t) > 2:
            xy = t[:, 1:3] - t[:, 1:3].mean(0)
            w, v = np.linalg.eigh(xy.T @ xy)
            if w[1] > 1.0:  # 至少走过一段，主方向才可靠
                axis = v[:, 1]
                if axis @ (t[-1, 1:3] - t[0, 1:3]) < 0:
                    axis = -axis
        lat = np.array([-axis[1], axis[0]])

        along_t, lat_t = (t[:, 1:3] @ axis, t[:, 1:3] @ lat) if len(t) else (np.zeros(0), np.zeros(0))
        passes, pass_rows = pass_consistency(t[:, 0], along_t, lat_t, t[:, 3]) if len(t) else ([], [])

        # 静止漂移：位移投到沿轨道 / 横向
        if len(sl):
            d_along, d_lat = sl[:, 5:7] @ axis, sl[:, 5:7] @ lat
            d_norm = np.linalg.norm(sl[:, 5:8], axis=1)
        else:
            d_along = d_lat = d_norm = np.zeros(0)
        final = sl[:, 9] > 0 if len(sl) else np.zeros(0, bool)
        long_final = final & (sl[:, 1] >= 60) if len(sl) else final
        rates = d_norm[long_final] / sl[long_final, 1] * 3600 if long_final.any() else np.zeros(0)

        sessions.append({
            'index': i, 'start': starts[i], 'axis': axis, 'lat': lat, 'traj': t, 'stops': s, 'status': st,
            'still': sl, 'still_along': d_along, 'still_lat': d_lat, 'still_norm': d_norm, 'still_final': final,
            'still_rates': rates, 'passes': passes, 'pass_rows': pass_rows, 'map': maps.get(i),
            'along_t': along_t, 'lat_t': lat_t,
        })
    return starts, sessions


def main():
    parser = argparse.ArgumentParser(description='把 run_lio_online 的记录画成 html')
    parser.add_argument('dir')
    parser.add_argument('-o', '--output', help='输出 html，默认 <记录目录>/lio_record.html')
    args = parser.parse_args()
    d = args.dir
    out = args.output or os.path.join(d, 'lio_record.html')

    starts, sessions = analyze(d)
    pages = []
    for S in sessions:
        t, s, st, sl, axis, lat = S['traj'], S['stops'], S['status'], S['still'], S['axis'], S['lat']
        td_idx = np.linspace(0, len(t) - 1, min(len(t), 40000)).astype(int) if len(t) else []
        td = t[td_idx] if len(t) else t
        m = S['map']

        # 剖面：最早和最近一趟（各自抽稀到 2000 点）
        prof = []
        if S['passes']:
            chosen = [(0, '最早一趟')] + ([(len(S['passes']) - 1, '最近一趟')] if len(S['passes']) > 1 else [])
            for k, name in chosen:
                idx = S['passes'][k]
                idx = idx[np.linspace(0, len(idx) - 1, min(len(idx), 2000)).astype(int)]
                prof.append({'name': f'{name}（{datetime.fromtimestamp(t[idx[0], 0]):%m-%d %H:%M}）',
                             'along': r2(S['along_t'][idx]), 'lat': r2(S['lat_t'][idx], 3), 'z': r2(t[idx, 3], 3)})

        pr = S['pass_rows']
        rates = S['still_rates']
        pages.append({
            'label': f'第 {S["index"] + 1} 次运行（{datetime.fromtimestamp(S["start"]):%m-%d %H:%M} 启动）',
            'traj': {'t': r2(td[:, 0], 1), 'x': r2(td[:, 1]), 'y': r2(td[:, 2]), 'z': r2(td[:, 3]),
                     'along': r2(td[:, 1:3] @ axis), 'lat': r2(td[:, 1:3] @ lat)} if len(td) else None,
            'stops': {'t': r2(s[:, 0], 0), 'x': r2(s[:, 1]), 'y': r2(s[:, 2]), 'z': r2(s[:, 3]),
                      'along': r2(s[:, 1:3] @ axis), 'lat': r2(s[:, 1:3] @ lat), 'path': r2(s[:, 4], 1)}
            if len(s) else None,
            'still': {'t': r2(sl[:, 0] + sl[:, 1], 0), 'dur': r2(sl[:, 1], 0), 'along': r2(S['still_along'] * 100),
                      'lat': r2(S['still_lat'] * 100), 'z': r2(sl[:, 7] * 100), 'yaw': r2(sl[:, 8], 3),
                      'final': r2(sl[:, 9], 0)} if len(sl) else None,
            'passdev': {'t': [r['t'] for r in pr], 'dl': [round(r['dl'] * 100, 1) for r in pr],
                        'dz': [round(r['dz'] * 100, 1) for r in pr], 'n': [r['n'] for r in pr],
                        'len': [round(r['len'], 1) for r in pr], 'dir': [r['dir'] for r in pr]} if pr else None,
            'profiles': prof,
            'map': {'along': r2(m[0][:, :2] @ axis), 'lat': r2(m[0][:, :2] @ lat), 'z': r2(m[0][:, 2], 1),
                    'n': int(m[1])} if m else None,
            'status': {'t': r2(st[:, 0], 0), 'frames': r2(st[:, 1], 0), 'scans': r2(st[:, 2], 0),
                       'dropped': r2(st[:, 3], 0), 'lio_avg': r2(st[:, 4]), 'lio_max': r2(st[:, 5]),
                       'grids': r2(st[:, 6], 0), 'rss': r2(st[:, 7], 1), 'cpu': r2(st[:, 8], 0)} if len(st) else None,
            'stats': {
                'hours': (st[-1, 0] - S['start']) / 3600 if len(st) else 0,
                'frames': int(st[-1, 1]) if len(st) else 0, 'scans': int(st[-1, 2]) if len(st) else 0,
                'dropped': int(st[-1, 3]) if len(st) else 0,
                'cpu_max': float(st[:, 8].max()) if len(st) else 0, 'rss_max': float(st[:, 7].max()) if len(st) else 0,
                'lio_max': float(st[:, 5].max()) if len(st) else 0, 'grids': int(st[-1, 6]) if len(st) else 0,
                'stops': int(len(s)), 'km': float(s[:, 4].sum() / 1000) if len(s) else 0,
                'still_h': float(sl[S['still_final'], 1].sum() / 3600) if len(sl) else 0,
                'still_rate': float(np.median(rates) * 100) if len(rates) else None,
                'passes': len(S['passes']),
                'last_dl': pr[-1]['dl'] * 100 if pr else None, 'last_dz': pr[-1]['dz'] * 100 if pr else None,
            },
        })

    meta = {'dir': os.path.abspath(d), 'generated': datetime.now().strftime('%Y-%m-%d %H:%M'),
            'sessions': len(starts)}
    page = TEMPLATE.replace('__DATA__', json.dumps({'meta': meta, 'sessions': pages}, ensure_ascii=False,
                                                   separators=(',', ':')))
    page = page.replace('__TITLE__', html.escape(os.path.basename(os.path.abspath(d)) or '里程计记录'))
    with open(out, 'w') as f:
        f.write(page)
    print(f'{out}  （{os.path.getsize(out) / 1024:.0f} KB，{len(pages)} 次运行）')


TEMPLATE = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>里程计记录 · __TITLE__</title>
<style>
:root {
  --bg: #f7f7f5; --panel: #ffffff; --text: #1f2328; --muted: #6b7078; --grid: #e6e6e3; --axis: #9a9ea6;
  --map: #9aa0a6; --c0: #2f6fdd; --c1: #d4572a; --c2: #2c9a5b; --c3: #8a55c9; --c4: #b58b00; --c5: #0f8b99;
  --stop: #d4572a; --tip-bg: #1f2328; --tip-fg: #ffffff;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #15171a; --panel: #1d2024; --text: #e6e7e9; --muted: #9aa0a8; --grid: #2b2f35; --axis: #6b717a;
    --map: #5b6169; --c0: #6ea2ff; --c1: #ff8a5c; --c2: #5fcf8e; --c3: #b993ff; --c4: #e6c04d; --c5: #4fd1de;
    --stop: #ff8a5c; --tip-bg: #e6e7e9; --tip-fg: #15171a;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text);
  font: 14px/1.5 -apple-system, "PingFang SC", "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; }
main { max-width: 1200px; margin: 0 auto; padding: 20px 16px 40px; }
h1 { font-size: 20px; margin: 0 0 4px; }
.sub { color: var(--muted); font-size: 12px; margin-bottom: 16px; word-break: break-all; }
.bar { display: flex; gap: 12px; align-items: center; flex-wrap: wrap; margin-bottom: 12px; }
select { font: inherit; padding: 4px 8px; background: var(--panel); color: var(--text);
  border: 1px solid var(--grid); border-radius: 6px; }
.cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 8px; margin-bottom: 16px; }
.card { background: var(--panel); border: 1px solid var(--grid); border-radius: 8px; padding: 8px 10px; }
.card .k { color: var(--muted); font-size: 12px; }
.card .v { font-size: 17px; font-variant-numeric: tabular-nums; }
.panel { background: var(--panel); border: 1px solid var(--grid); border-radius: 8px; padding: 10px 12px 6px;
  margin-bottom: 12px; }
.panel h2 { font-size: 14px; margin: 0 0 2px; }
.panel .note { color: var(--muted); font-size: 12px; margin-bottom: 6px; }
.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 760px) { .grid2 { grid-template-columns: 1fr; } }
canvas { width: 100%; display: block; touch-action: none; }
.legend { display: flex; flex-wrap: wrap; gap: 4px 14px; font-size: 12px; color: var(--muted); margin: 2px 0 4px; }
.legend i { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 5px;
  vertical-align: -1px; }
#tip { position: fixed; pointer-events: none; background: var(--tip-bg); color: var(--tip-fg); font-size: 12px;
  padding: 4px 7px; border-radius: 5px; white-space: pre; display: none; z-index: 10; }
.empty { color: var(--muted); padding: 20px 0; text-align: center; }
</style>
</head>
<body>
<main>
  <h1>里程计记录</h1>
  <div class="sub" id="sub"></div>
  <div class="bar"><label>运行：<select id="sess"></select></label>
    <span class="sub" style="margin:0">滚轮缩放，拖动平移，双击复原</span></div>
  <div class="cards" id="cards"></div>
  <div class="panel"><h2>俯视图</h2><div class="note" id="mapnote"></div>
    <div class="legend" id="lg-top"></div><canvas id="top" height="360"></canvas></div>
  <div class="grid2">
    <div class="panel"><h2>沿轨道位置</h2><div class="note">轨迹水平位置投到主方向上</div>
      <canvas id="along" height="240"></canvas></div>
    <div class="panel"><h2>高度</h2><div class="note">odom 系 z</div><canvas id="height" height="240"></canvas></div>
  </div>
  <div class="panel"><h2>静止时的漂移</h2>
    <div class="note">IMU 判断真正静止（加计 1 秒内的波动小于 4 mg）的每一段里，里程计位置变了多少，单位 cm。横轴是该段结束的时间；
      大点是整段的结果，小点是长时间静止时每 10 分钟记一次的累计值。不依赖停车位置</div>
    <div class="legend" id="lg-still"></div><canvas id="still" height="220"></canvas></div>
  <div class="panel"><h2>来回一致性</h2>
    <div class="note">轨道是固定的直线：每一趟经过同一沿轨道位置（2 m 一格）时，横向偏移和高度应该相同。
      这里画每一趟相对最早经过那里的一趟的偏差（取中位数，单位 cm），随时间变大就是漂移。沿轨道方向本身的漂移从这里看不出来</div>
    <div class="legend" id="lg-pass"></div><canvas id="passdev" height="220"></canvas></div>
  <div class="grid2">
    <div class="panel"><h2>横向剖面</h2><div class="note">横轴沿轨道位置，纵轴横向偏移，单位 m</div>
      <div class="legend" id="lg-prof"></div><canvas id="prof_lat" height="200"></canvas></div>
    <div class="panel"><h2>高度剖面</h2><div class="note">横轴沿轨道位置，纵轴高度，单位 m</div>
      <div class="legend" id="lg-prof2"></div><canvas id="prof_z" height="200"></canvas></div>
  </div>
  <div class="grid2">
    <div class="panel"><h2>CPU（占单核 %）</h2><canvas id="cpu" height="180"></canvas></div>
    <div class="panel"><h2>常驻内存（MB）</h2><canvas id="rss" height="180"></canvas></div>
    <div class="panel"><h2>每帧耗时（ms）</h2><div class="legend" id="lg-lio"></div><canvas id="lio" height="180"></canvas></div>
    <div class="panel"><h2>局部地图格数 / 累计丢帧</h2><div class="legend" id="lg-grids"></div>
      <canvas id="grids" height="180"></canvas></div>
  </div>
</main>
<div id="tip"></div>
<script>
const DATA = __DATA__;
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const PAL = ['--c0', '--c1', '--c2', '--c3', '--c4', '--c5'];
const tip = document.getElementById('tip');
const pad2 = n => String(n).padStart(2, '0');
function fmtTime(t, span) {
  const d = new Date(t * 1000);
  const hm = pad2(d.getHours()) + ':' + pad2(d.getMinutes());
  return span > 86400 * 1.5 ? (d.getMonth() + 1) + '-' + d.getDate() + ' ' + hm : hm;
}
function fmtFull(t) {
  const d = new Date(t * 1000);
  return (d.getMonth() + 1) + '-' + d.getDate() + ' ' + pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
}
function niceTicks(lo, hi, n) {
  const span = hi - lo || 1, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 5, 10].map(m => m * mag).find(s => s >= step0);
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}
function timeTicks(lo, hi, n) {
  const steps = [60, 300, 600, 1800, 3600, 7200, 14400, 21600, 43200, 86400, 172800, 604800];
  const step = steps.find(s => (hi - lo) / s <= n) || steps[steps.length - 1];
  const off = new Date().getTimezoneOffset() * 60, out = [];
  for (let v = Math.ceil((lo - off) / step) * step + off; v <= hi; v += step) out.push(v);
  return out;
}

// 简单的画布图：series = [{name, color, x, y, mode: 'line'|'dot', size, text(i)}]
class Chart {
  constructor(id, opt) {
    this.c = document.getElementById(id); this.opt = opt; this.series = [];
    this.c.addEventListener('wheel', e => this.zoom(e), {passive: false});
    this.c.addEventListener('pointerdown', e => this.down(e));
    window.addEventListener('pointermove', e => this.move(e));
    window.addEventListener('pointerup', () => { this.drag = null; });
    this.c.addEventListener('dblclick', () => { this.view = null; this.draw(); });
    this.c.addEventListener('pointerleave', () => { tip.style.display = 'none'; this.hover = null; this.draw(); });
  }
  set(series) { this.series = series; this.view = null; this.draw(); }
  extent() {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const s of this.series) {
      if (s.noExtent) continue;
      for (let i = 0; i < s.x.length; i++) {
        const x = s.x[i], y = s.y[i];
        if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
      }
    }
    if (!isFinite(x0)) return null;
    if (x1 - x0 < 1e-6) { x0 -= 1; x1 += 1; }
    if (y1 - y0 < 1e-6) { y0 -= 0.5; y1 += 0.5; }
    const py = (y1 - y0) * 0.06, px = this.opt.time ? 0 : (x1 - x0) * 0.03;
    return [x0 - px, x1 + px, y0 - py, y1 + py];
  }
  layout() {
    const dpr = window.devicePixelRatio || 1, w = this.c.clientWidth, h = +this.c.getAttribute('height');
    if (this.c.width !== Math.round(w * dpr)) { this.c.width = Math.round(w * dpr); this.c.height = Math.round(h * dpr);
      this.c.style.height = h + 'px'; }
    const m = {l: 56, r: 12, t: 8, b: 26};
    let [x0, x1, y0, y1] = this.view || this.ext || [0, 1, 0, 1];
    const pw = w - m.l - m.r, ph = h - m.t - m.b;
    if (this.opt.equal) {  // 俯视图等比例
      const sx = pw / (x1 - x0), sy = ph / (y1 - y0), s = Math.min(sx, sy);
      const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
      x0 = cx - pw / s / 2; x1 = cx + pw / s / 2; y0 = cy - ph / s / 2; y1 = cy + ph / s / 2;
    }
    return {dpr, w, h, m, pw, ph, x0, x1, y0, y1,
      X: x => m.l + (x - x0) / (x1 - x0) * pw, Y: y => m.t + (1 - (y - y0) / (y1 - y0)) * ph};
  }
  draw() {
    const g = this.c.getContext('2d');
    this.ext = this.extent();
    const L = this.L = this.layout(), {dpr, w, h, m} = L;
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
    if (!this.ext) { g.fillStyle = css('--muted'); g.font = '13px sans-serif'; g.textAlign = 'center';
      g.fillText('没有数据', w / 2, h / 2); return; }
    g.font = '11px sans-serif'; g.strokeStyle = css('--grid'); g.fillStyle = css('--muted'); g.lineWidth = 1;
    const xt = this.opt.time ? timeTicks(L.x0, L.x1, 7) : niceTicks(L.x0, L.x1, 8), yt = niceTicks(L.y0, L.y1, 5);
    g.textAlign = 'center';
    for (const v of xt) { const x = L.X(v); g.beginPath(); g.moveTo(x, m.t); g.lineTo(x, h - m.b); g.stroke();
      g.fillText(this.opt.time ? fmtTime(v, L.x1 - L.x0) : +v.toFixed(3), x, h - 8); }
    g.textAlign = 'right';
    for (const v of yt) { const y = L.Y(v); g.beginPath(); g.moveTo(m.l, y); g.lineTo(w - m.r, y); g.stroke();
      g.fillText(+v.toFixed(3), m.l - 6, y + 4); }
    g.save(); g.beginPath(); g.rect(m.l, m.t, L.pw, L.ph); g.clip();
    for (const s of this.series) {
      g.strokeStyle = g.fillStyle = s.color;
      if (s.mode === 'line') {
        g.lineWidth = s.width || 1.5; g.beginPath(); let pen = false;
        for (let i = 0; i < s.x.length; i++) {
          const x = L.X(s.x[i]), y = L.Y(s.y[i]);
          const gap = s.gap && i > 0 && s.x[i] - s.x[i - 1] > s.gap;
          if (!pen || gap) { g.moveTo(x, y); pen = true; } else g.lineTo(x, y);
        }
        g.stroke();
      } else {
        const r = s.size || 2; g.globalAlpha = s.alpha || 1;
        for (let i = 0; i < s.x.length; i++) {
          if (s.colors) g.fillStyle = s.colors[i];
          g.fillRect(L.X(s.x[i]) - r / 2, L.Y(s.y[i]) - r / 2, r, r);
        }
        g.globalAlpha = 1;
      }
    }
    if (this.hover) { const {s, i} = this.hover; g.fillStyle = s.color; g.strokeStyle = css('--panel'); g.lineWidth = 2;
      g.beginPath(); g.arc(L.X(s.x[i]), L.Y(s.y[i]), 4.5, 0, 7); g.fill(); g.stroke(); }
    g.restore();
    g.fillStyle = css('--muted'); g.textAlign = 'left';
    if (this.opt.ylabel) g.fillText(this.opt.ylabel, m.l + 4, m.t + 12);
  }
  pos(e) { const r = this.c.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
  zoom(e) {
    if (!this.L || !this.ext) return; e.preventDefault();
    const [px, py] = this.pos(e), L = this.L, k = e.deltaY > 0 ? 1.25 : 0.8;
    const fx = (px - L.m.l) / L.pw, fy = 1 - (py - L.m.t) / L.ph;
    const cx = L.x0 + fx * (L.x1 - L.x0), cy = L.y0 + fy * (L.y1 - L.y0);
    let [x0, x1, y0, y1] = [L.x0, L.x1, L.y0, L.y1];
    x0 = cx - (cx - x0) * k; x1 = cx + (x1 - cx) * k;
    if (!this.opt.time || this.opt.equal) { y0 = cy - (cy - y0) * k; y1 = cy + (y1 - cy) * k; }
    else { [, , y0, y1] = this.ext; }
    this.view = [x0, x1, y0, y1]; this.draw();
  }
  down(e) { if (!this.L) return; this.drag = {p: this.pos(e), v: [this.L.x0, this.L.x1, this.L.y0, this.L.y1]}; }
  move(e) {
    if (!this.L) return;
    const [px, py] = this.pos(e), L = this.L;
    if (this.drag) {
      const [qx, qy] = this.drag.p, [x0, x1, y0, y1] = this.drag.v;
      const dx = (px - qx) / L.pw * (x1 - x0), dy = (py - qy) / L.ph * (y1 - y0);
      this.view = this.opt.time && !this.opt.equal ? [x0 - dx, x1 - dx, ...this.ext.slice(2)]
                                                     : [x0 - dx, x1 - dx, y0 + dy, y1 + dy];
      this.draw(); return;
    }
    if (px < 0 || py < 0 || px > L.w || py > L.h) return;
    let best = null, bd = 144;
    for (const s of this.series) {
      if (!s.text) continue;
      for (let i = 0; i < s.x.length; i++) {
        const d = (L.X(s.x[i]) - px) ** 2 + (L.Y(s.y[i]) - py) ** 2;
        if (d < bd) { bd = d; best = {s, i}; }
      }
    }
    this.hover = best; this.draw();
    if (best) { tip.textContent = best.s.text(best.i); tip.style.display = 'block';
      tip.style.left = (e.clientX + 12) + 'px'; tip.style.top = (e.clientY + 12) + 'px'; }
    else tip.style.display = 'none';
  }
}

const charts = {};
for (const [id, opt] of Object.entries({top: {equal: true}, along: {time: true, ylabel: 'm'}, height: {time: true, ylabel: 'm'},
  still: {time: true, ylabel: 'cm'}, passdev: {time: true, ylabel: 'cm'}, prof_lat: {ylabel: 'm'}, prof_z: {ylabel: 'm'},
  cpu: {time: true}, rss: {time: true}, lio: {time: true}, grids: {time: true}}))
  charts[id] = new Chart(id, opt);

function legend(id, items) {
  document.getElementById(id).innerHTML = items.map(([n, c]) => `<span><i style="background:${c}"></i>${n}</span>`).join('');
}
function card(k, v) { return `<div class="card"><div class="k">${k}</div><div class="v">${v}</div></div>`; }

function show(idx) {
  const S = DATA.sessions[idx], st = S.stats, T = S.traj, P = S.stops, M = S.map, Q = S.status;
  document.getElementById('cards').innerHTML = [
    card('运行时长', st.hours.toFixed(st.hours < 10 ? 2 : 1) + ' 小时'),
    card('处理 / 收到帧', st.frames + ' / ' + st.scans), card('丢帧', st.dropped),
    card('走过路程', st.km.toFixed(2) + ' km'), card('来回趟数', st.passes),
    card('最近一趟偏差（横向 / 高度）', st.last_dl === null ? '-' : st.last_dl.toFixed(1) + ' / ' + st.last_dz.toFixed(1) + ' cm'),
    card('静止总时长', st.still_h.toFixed(st.still_h < 10 ? 2 : 1) + ' 小时'),
    card('静止漂移（中位）', st.still_rate === null ? '-' : st.still_rate.toFixed(1) + ' cm/小时'),
    card('局部地图', st.grids + ' 格'), card('最大 CPU', st.cpu_max.toFixed(0) + '%'),
    card('最大内存', st.rss_max.toFixed(0) + ' MB'), card('最大每帧耗时', st.lio_max.toFixed(0) + ' ms')].join('');

  const c0 = css('--c0'), cs = css('--stop'), cm = css('--map');
  // 俯视图：地图按高度从浅到深
  const top = [];
  if (M) {
    let z0 = Infinity, z1 = -Infinity; for (const z of M.z) { if (z < z0) z0 = z; if (z > z1) z1 = z; }
    const dark = matchMedia('(prefers-color-scheme: dark)').matches;
    const colors = M.z.map(z => { const f = (z - z0) / (z1 - z0 || 1);
      const v = dark ? Math.round(60 + 120 * f) : Math.round(215 - 120 * f); return `rgb(${v},${v},${v + 4})`; });
    top.push({mode: 'dot', x: M.along, y: M.lat, colors, size: 1.6});
  }
  if (T) top.push({name: '轨迹', mode: 'line', x: T.along, y: T.lat, color: c0, width: 1.6, gap: 300,
    text: i => `${fmtFull(T.t[i])}\nx ${T.x[i]}  y ${T.y[i]}  z ${T.z[i]}`});
  if (P) top.push({name: '停车位置', mode: 'dot', x: P.along, y: P.lat, color: cs, size: 7,
    text: i => `停稳 ${fmtFull(P.t[i])}\nx ${P.x[i]}  y ${P.y[i]}  z ${P.z[i]}\n这一段 ${P.path[i]} m`});
  charts.top.set(top);
  legend('lg-top', [['轨迹', c0], ['停车位置', cs]].concat(M ? [['局部地图（按高度深浅）', cm]] : []));
  const frame = '横轴沿轨道、纵轴横向（轨迹的主方向，等比例），单位 m；悬停显示 odom 系 x y z';
  document.getElementById('mapnote').textContent = M ? `${frame}。局部地图 ${M.n} 点（显示 ${M.along.length} 个，每 0.5 m 取最高点）`
    : `${frame}。没有这次运行的局部地图（local_map.pcd）`;

  charts.along.set(T ? [{mode: 'line', x: T.t, y: T.along, color: c0, gap: 300,
    text: i => `${fmtFull(T.t[i])}\n沿轨道 ${T.along[i]} m`}].concat(P ? [{mode: 'dot', x: P.t, y: P.along, color: cs, size: 6,
    text: i => `停稳 ${fmtFull(P.t[i])}\n沿轨道 ${P.along[i]} m`}] : []) : []);
  charts.height.set(T ? [{mode: 'line', x: T.t, y: T.z, color: c0, gap: 300,
    text: i => `${fmtFull(T.t[i])}\nz ${T.z[i]} m`}] : []);

  // 静止漂移
  const L = S.still, c1 = css('--c1'), c2 = css('--c2');
  const still = [];
  if (L) {
    const pick = f => L.t.map((_, i) => i).filter(i => (L.final[i] > 0) === f);
    for (const [f, size] of [[true, 7], [false, 3]]) {
      const ix = pick(f);
      [['沿轨道', 'along', c0], ['横向', 'lat', c1], ['高度', 'z', c2]].forEach(([name, key, col]) => still.push({
        mode: 'dot', size, color: col, x: ix.map(i => L.t[i]), y: ix.map(i => L[key][i]),
        text: k => { const i = ix[k]; return `静止 ${(L.dur[i] / 60).toFixed(1)} 分钟，到 ${fmtFull(L.t[i])}${f ? '' : '（累计中）'}\n` +
          `沿轨道 ${L.along[i]}  横向 ${L.lat[i]}  高度 ${L.z[i]} cm\n航向 ${L.yaw[i]}°  ` +
          `合计 ${(Math.hypot(L.along[i], L.lat[i], L.z[i]) / L.dur[i] * 3600).toFixed(1)} cm/小时`; }}));
    }
  }
  charts.still.set(still);
  legend('lg-still', L ? [['沿轨道', c0], ['横向', c1], ['高度', c2]] : [['还没有记录（still.csv，需要新版本程序；静止 30 秒以上才记）', css('--muted')]]);

  // 来回一致性
  const D = S.passdev;
  charts.passdev.set(D ? [['横向', 'dl', c1], ['高度', 'dz', c2]].map(([name, key, col]) => ({mode: 'dot', size: 6, color: col,
    x: D.t, y: D[key], text: i => `${fmtFull(D.t[i])} 这一趟（${D.dir[i] > 0 ? '正向' : '反向'}，${D.len[i]} m，` +
      `和更早的趟重合 ${D.n[i] * 2} m）\n横向偏差 ${D.dl[i]} cm  高度偏差 ${D.dz[i]} cm`})) : []);
  legend('lg-pass', D ? [['横向', c1], ['高度', c2]] : [['还没有两趟以上重合的轨迹（每趟至少 10 m）', css('--muted')]]);

  // 剖面：最早和最近一趟
  const PR = S.profiles, pc = [c0, c1];
  charts.prof_lat.set(PR.map((p, k) => ({mode: 'line', x: p.along, y: p.lat, color: pc[k], width: 1.4,
    text: i => `${p.name}\n沿轨道 ${p.along[i]} m  横向 ${p.lat[i]} m`})));
  charts.prof_z.set(PR.map((p, k) => ({mode: 'line', x: p.along, y: p.z, color: pc[k], width: 1.4,
    text: i => `${p.name}\n沿轨道 ${p.along[i]} m  高度 ${p.z[i]} m`})));
  legend('lg-prof', PR.length ? PR.map((p, k) => [p.name, pc[k]]) : [['还没有完整的一趟（至少 10 m）', css('--muted')]]);
  legend('lg-prof2', PR.map((p, k) => [p.name, pc[k]]));

  const ts = (key, col, name) => Q ? {mode: 'line', x: Q.t, y: Q[key], color: col, gap: 600,
    text: i => `${fmtFull(Q.t[i])}\n${name} ${Q[key][i]}`} : null;
  charts.cpu.set(Q ? [ts('cpu', c0, 'CPU %')] : []);
  charts.rss.set(Q ? [ts('rss', c0, '内存 MB')] : []);
  charts.lio.set(Q ? [ts('lio_max', css('--c1'), '最大 ms'), ts('lio_avg', c0, '平均 ms')] : []);
  legend('lg-lio', [['平均', c0], ['区间内最大', css('--c1')]]);
  if (Q) {
    const gmax = Math.max(...Q.grids, 1), dmax = Math.max(...Q.dropped, 1);
    charts.grids.set([ts('grids', c0, '格数'), {mode: 'line', x: Q.t, y: Q.dropped.map(v => v / dmax * gmax), color: css('--c1'),
      text: i => `${fmtFull(Q.t[i])}\n累计丢帧 ${Q.dropped[i]}`}]);
  } else charts.grids.set([]);
  legend('lg-grids', [['局部地图格数', c0], ['累计丢帧（按比例缩放到同一高度）', css('--c1')]]);
}

const sel = document.getElementById('sess');
DATA.sessions.forEach((s, i) => sel.add(new Option(s.label, i)));
const want = parseInt(location.hash.slice(1));  // 地址后加 #序号 可以直接打开某次运行
sel.value = want >= 0 && want < DATA.sessions.length ? want : DATA.sessions.length - 1;
sel.onchange = () => show(+sel.value);
document.getElementById('sub').textContent = `${DATA.meta.dir} · 生成于 ${DATA.meta.generated} · 共启动 ${DATA.meta.sessions} 次`;
if (DATA.sessions.length) show(+sel.value);
else document.querySelector('main').insertAdjacentHTML('beforeend', '<div class="empty">记录里没有数据</div>');
window.addEventListener('resize', () => Object.values(charts).forEach(c => c.draw()));
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => show(+sel.value));
</script>
</body>
</html>
'''

if __name__ == '__main__':
    main()
