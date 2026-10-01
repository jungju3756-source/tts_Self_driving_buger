#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fit_mount.py  —  장착 형상(높이·각도·행 방향) 역산 도구

평평한 바닥 위에 세워둔 상태로 실행하면, 측정 데이터가 가장 잘 설명되는
mount_height_m / tilt_deg / row0_is_top 조합을 찾아준다.

원리: 평지에서는 모든 셀에 대해  d * (-v_z) = h  가 성립한다.
      따라서 (tilt, row 방향)을 바꿔가며 이 값의 흩어짐이 최소가 되는 조합을 고르면
      그것이 실제 장착 형상이고, 그때의 중앙값이 곧 센서 높이다.

주의: 시야 안에 낭떠러지나 장애물이 없어야 한다. 평평한 바닥만 보여줄 것.

실행:  python3 tools/fit_mount.py            (라파2에서)
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rapa2'))
from tof_cliff_detector import ZoneGeometry, SerialFrameSource   # noqa: E402


def collect(port, baud, n_frames, grid=8):
    src = SerialFrameSource(port, baud, grid)
    frames = []
    while len(frames) < n_frames:
        f = src.read_frame()
        if f is None:
            print('프레임 수신 실패 — 센서 연결 확인')
            src.close()
            sys.exit(1)
        frames.append(f)
    src.close()
    stack = np.array(frames, dtype=float)
    # 셀별 중앙값 (튀는 프레임 제거)
    with np.errstate(all='ignore'):
        med = np.nanmedian(np.where(stack > 0, stack, np.nan), axis=0)
    return med / 1000.0, stack


def fit(med_m, scfg, row0_top, tilt_deg):
    s = dict(scfg)
    s['row0_is_top'] = row0_top
    s['tilt_deg'] = tilt_deg
    s['mount_height_m'] = 1.0          # 높이는 스케일만 바꾸므로 아무 값
    geo = ZoneGeometry(s, 100000)      # 유효범위 제한 없이
    ok = np.isfinite(med_m) & (med_m > 0) & (geo.nz > 1e-6)
    if ok.sum() < 20:
        return None
    hs = med_m[ok] * geo.nz[ok]        # 평지라면 전부 같은 값(=h)이어야 함
    h = float(np.median(hs))
    resid = float(np.median(np.abs(hs - h)))   # 중앙절대편차
    return h, resid, int(ok.sum())


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(root, 'config', 'cliff_config.yaml'))
    ap.add_argument('--frames', type=int, default=20)
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    scfg = cfg['sensor']

    print('평지 데이터 {} 프레임 수집 중...'.format(args.frames))
    med_m, stack = collect(scfg['port'], int(scfg['baud']), args.frames,
                           int(scfg['grid']))

    valid_ratio = float(np.mean(np.isfinite(med_m)))
    print('유효 셀 비율 {:.0%}\n'.format(valid_ratio))

    print('측정 거리 [mm] (셀별 중앙값)')
    for r in range(med_m.shape[0]):
        print('  R{} '.format(r) + ' '.join(
            '----' if not np.isfinite(v) else '{:4.0f}'.format(v * 1000)
            for v in med_m[r]))

    TILT_LO, TILT_HI = 3.0, 80.0
    best = None
    for row0_top in (True, False):
        for tilt in np.arange(TILT_LO, TILT_HI + 0.01, 0.25):
            r = fit(med_m, scfg, row0_top, float(tilt))
            if r is None:
                continue
            h, resid, n = r
            if 0.05 < h < 1.5 and (best is None or resid < best[2]):
                best = (row0_top, float(tilt), resid, h, n)

    if best is None:
        print('\n적합 실패 — 평지가 아니거나 유효 셀이 너무 적습니다')
        return 1

    row0_top, tilt, resid, h, n = best
    print('\n' + '=' * 56)
    print(' 역산 결과')
    print('=' * 56)
    print('  tilt_deg        : {:.1f}'.format(tilt))
    print('  mount_height_m  : {:.3f}   ({:.1f} cm)'.format(h, h * 100))
    print('  row0_is_top     : {}'.format(str(row0_top).lower()))
    print('  잔차(중앙절대편차): {:.1f} mm   (사용 셀 {}개)'.format(resid * 1000, n))

    if tilt <= TILT_LO + 0.3 or tilt >= TILT_HI - 0.3:
        print('\n  ※ 탐색 범위 끝({:.0f}~{:.0f}°)에 걸렸습니다 — 결과를 믿지 마세요.'
              .format(TILT_LO, TILT_HI))
        print('     평면이 아닌 것을 보고 있을 가능성이 큽니다.')
    elif resid > 0.03:
        print('\n  ※ 잔차가 큽니다. 시야에 장애물/낭떠러지가 있거나')
        print('     바닥이 평평하지 않을 수 있습니다. 다시 측정해 보세요.')
    else:
        print('\n  잔차가 작아 평면과 잘 맞습니다. 위 값을 config 에 넣으세요.')

    # 반대 행 방향과 비교해 판정이 확실한지 보여준다
    other = None
    for tilt2 in np.arange(TILT_LO, TILT_HI + 0.01, 0.25):
        r = fit(med_m, scfg, not row0_top, float(tilt2))
        if r and 0.05 < r[0] < 1.5 and (other is None or r[1] < other[1]):
            other = r
    if other:
        print('  참고: row0_is_top={} 로 두면 잔차 {:.1f} mm (더 나쁨)'
              .format(str(not row0_top).lower(), other[1] * 1000))

    print('\n좌우(col0_is_left)는 평지만으로는 알 수 없습니다.')
    print('센서 앞 왼쪽 바닥에만 물체를 놓고 감지기를 돌려 zone 을 확인하세요.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
