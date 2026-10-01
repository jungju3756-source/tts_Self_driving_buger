#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
replay.py  —  녹화한 프레임으로 판정 로직을 검증한다 (센서 없이)

MQTT frame 토픽을 그대로 받아 적은 JSONL 을 넣으면, 현재 설정으로 몇 프레임이
낭떠러지로 판정되는지 알려준다. 문턱값을 바꾼 뒤 '평지는 0%, 절벽은 100%' 가
되는지 확인하는 용도다.

    # 녹화
    mosquitto_sub -h 192.168.0.37 -t wheelchair/tof/frame -C 60 > flat.jsonl

    # 검증
    python3 tools/replay.py flat.jsonl                 # 판정 비율
    python3 tools/replay.py flat.jsonl --baseline out.yaml   # 기준면 생성
    python3 tools/replay.py flat.jsonl cliff.jsonl     # 두 상태를 비교
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rapa2'))
from tof_cliff_detector import CliffDetector, ZoneGeometry   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_CONFIG = os.path.join(ROOT, 'config', 'cliff_config.yaml')


def load_frames(path, n):
    """JSONL → [프레임, n, n] (mm, 무응답은 nan)"""
    out = []
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            g = d.get('frame', d).get('grid_mm')
            if g is None:
                continue
            out.append([np.nan if v is None else float(v) for v in g])
    if not out:
        raise SystemExit('{}: 프레임이 없습니다'.format(path))
    return np.array(out, dtype=float).reshape(-1, n, n)


def make_baseline(stack, out_path, height_m):
    """평지 녹화본에서 셀별 기준거리 / 노이즈 / 응답률을 뽑아 저장한다."""
    n = stack.shape[1]
    m = stack / 1000.0
    valid_rate = np.mean(np.isfinite(m) & (m > 0), axis=0)
    base = np.full((n, n), np.nan)
    sig = np.full((n, n), np.nan)
    need = max(3, len(m) // 2)
    for r in range(n):
        for c in range(n):
            col = m[:, r, c]
            col = col[np.isfinite(col) & (col > 0)]
            if len(col) >= need:
                base[r, c] = float(np.median(col))
                sig[r, c] = float(np.std(col))

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({
            'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'frames': int(len(m)),
            'mount_height_m': float(height_m),
            'baseline_m': [[None if not np.isfinite(v) else round(float(v), 4)
                            for v in row] for row in base],
            'sigma_m': [[None if not np.isfinite(v) else round(float(v), 4)
                         for v in row] for row in sig],
            'valid_rate': [[round(float(v), 3) for v in row] for row in valid_rate],
        }, f, allow_unicode=True, sort_keys=False)
    ok = int(np.sum(np.isfinite(base)))
    print('[base] 저장: {}  (기준값 확보 {}/{}셀, {}프레임)'
          .format(out_path, ok, n * n, len(m)))
    return base, valid_rate


def run(stack, cfg, baseline_path, label):
    scfg, dcfg = cfg['sensor'], cfg['detect']
    geo = ZoneGeometry(scfg, dcfg['max_valid_range_mm'])
    det = CliffDetector(geo, dcfg)
    if det.mode in ('geometric', 'baseline'):
        if not det.load_baseline(baseline_path):
            raise SystemExit('기준면을 읽지 못했습니다: {}'.format(baseline_path))

    hits = 0
    zones = {}
    for g in stack:
        ev = det.evaluate(g)
        if ev['cliff']:
            hits += 1
            zones[ev['zone']] = zones.get(ev['zone'], 0) + 1
    pct = 100.0 * hits / len(stack)
    print('  {:<10} {:>3}/{:<3} 프레임 낭떠러지 판정 ({:5.1f}%)  {}'
          .format(label, hits, len(stack), pct,
                  ' '.join('%s=%d' % kv for kv in sorted(zones.items())) or ''))
    return pct, det


def cell_report(stack, det):
    """어느 셀이 얼마나 자주 낙차로 찍히는지 — 오탐 원인 추적용."""
    n = stack.shape[1]
    cnt = np.zeros((n, n))
    for g in stack:
        det.evaluate(g)
        cnt += det.last_drop | det.last_blind
    cnt = 100.0 * cnt / len(stack)
    print('\n  셀별 낙차판정 비율 [%]  (R7=먼쪽 위, . = 0%)')
    for r in range(n - 1, -1, -1):
        print('   R%d ' % r + ' '.join(
            '  . ' if cnt[r, c] == 0 else '%3.0f ' % cnt[r, c] for c in range(n)))


def main():
    ap = argparse.ArgumentParser(description='녹화 프레임으로 판정 로직 검증')
    ap.add_argument('flat', help='평지 녹화 JSONL')
    ap.add_argument('cliff', nargs='?', help='낭떠러지 녹화 JSONL (있으면 비교)')
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--baseline', help='이 평지 녹화로 기준면을 만들어 저장')
    ap.add_argument('--cells', action='store_true', help='셀별 판정 비율 출력')
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    n = int(cfg['sensor']['grid'])

    bpath = args.baseline or cfg['sensor'].get('baseline_file')
    if bpath and not os.path.isabs(bpath):
        bpath = os.path.join(ROOT, bpath)

    flat = load_frames(args.flat, n)
    if args.baseline:
        make_baseline(flat, bpath, float(cfg['sensor']['mount_height_m']))

    print('\n판정 방식: {}\n'.format(cfg['detect']['mode']))
    _, det = run(flat, cfg, bpath, '평지')
    if args.cells:
        cell_report(flat, det)
    if args.cliff:
        cliff = load_frames(args.cliff, n)
        run(cliff, cfg, bpath, '낭떠러지')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
