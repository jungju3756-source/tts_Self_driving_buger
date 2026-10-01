#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag.py  —  셀 단위 진단

지금 센서가 보고 있는 장면에 대해, 셀마다
  측정거리 / 기준거리 / 꺼진 깊이 / 판정 결과
를 한 화면에 펼쳐 보여준다. "왜 감지가 됐는지 / 왜 안 됐는지"를 눈으로 확인할 때 쓴다.

실행:  python3 tools/diag.py [--frames 10]
"""

import argparse
import os
import sys

import numpy as np
import yaml

_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_R, 'rapa2'))
from tof_cliff_detector import ZoneGeometry, CliffDetector, SerialFrameSource  # noqa: E402


def grid_print(title, arr, fmt='{:6.0f}', mask=None, na='   .  '):
    print('\n' + title)
    print('        ' + ' '.join('{:>6}'.format('C' + str(c)) for c in range(arr.shape[1])))
    for r in range(arr.shape[0]):
        cells = []
        for c in range(arr.shape[1]):
            v = arr[r, c]
            if (mask is not None and not mask[r, c]) or not np.isfinite(v):
                cells.append(na)
            else:
                cells.append(fmt.format(v))
        print('   R{}  '.format(r) + ' '.join(cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(_R, 'config', 'cliff_config.yaml'))
    ap.add_argument('--frames', type=int, default=10)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding='utf-8'))
    scfg, dcfg = cfg['sensor'], cfg['detect']

    geo = ZoneGeometry(scfg, dcfg['max_valid_range_mm'])
    det = CliffDetector(geo, dcfg)
    bpath = scfg.get('baseline_file')
    if bpath and not os.path.isabs(bpath):
        bpath = os.path.join(_R, bpath)
    has_base = det.load_baseline(bpath)

    src = SerialFrameSource(scfg['port'], int(scfg['baud']), geo.n)
    frames = []
    while len(frames) < args.frames:
        f = src.read_frame()
        if f is None:
            print('프레임 수신 실패')
            return 1
        frames.append(f)
    src.close()

    stack = np.array(frames, dtype=float)
    with np.errstate(all='ignore'):
        med = np.nanmedian(np.where(stack > 0, stack, np.nan), axis=0)

    print('기준면: {}'.format(bpath if has_base else '(없음 — 기하 모델 사용)'))
    print('판정 대상 셀: {}/{}   깊이 문턱 {:.0f}mm   군집 {}셀'.format(
        int(geo.active.sum()), geo.n * geo.n,
        det.min_depth_m * 1000, det.min_cluster))

    ref = det.baseline if det.baseline is not None else geo.d_exp
    drop, blind, depth = det._cell_flags(med)

    grid_print('① 측정거리 [mm]', med)
    grid_print('② 기준 지면거리 [mm]  (판정 대상 셀만)', ref * 1000, mask=geo.active)
    grid_print('③ 꺼진 깊이 [mm]  (양수 = 지면보다 아래)', depth * 1000, mask=geo.active)

    print('\n④ 판정  (D=낙차, ?=무응답, .=정상, 공백=판정 제외)')
    print('        ' + ' '.join('{:>2}'.format('C' + str(c)) for c in range(geo.n)))
    for r in range(geo.n):
        row = []
        for c in range(geo.n):
            if not geo.active[r, c]:
                row.append('  ')
            elif drop[r, c]:
                row.append(' D')
            elif blind[r, c]:
                row.append(' ?')
            else:
                row.append(' .')
        print('   R{}  '.format(r) + ' '.join(row))

    ev = det.evaluate(med)
    print('\n⑤ 이 프레임 판정: cliff={} zone={} cells={} range={} depth={} free_side={}'
          .format(ev['cliff'], ev['zone'], ev['n_cells'], ev.get('range_m'),
                  ev.get('depth_m'), ev.get('free_side')))

    n_drop, n_blind = int(drop.sum()), int(blind.sum())
    print('\n낙차 셀 {}개, 무응답 셀 {}개'.format(n_drop, n_blind))
    if n_drop == 0 and n_blind == 0:
        print('→ 지면이 기준면과 일치합니다. 낭떠러지가 감지 구간(전방 {:.2f}~{:.2f} m)'
              ' 밖에 있을 수 있습니다.'
              .format(float(geo.x_ahead[geo.active].min()),
                      float(geo.x_ahead[geo.active].max())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
