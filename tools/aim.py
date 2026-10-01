#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aim.py  —  센서 조준 도우미 (실시간)

브래킷을 손으로 잡고 이 화면을 보면서 각도를 맞춘다.
평평한 바닥 위, 앞이 1m 이상 트인 곳에서 실행할 것.

화면이 알려주는 것
  · 지금 바닥이 보이는지, 벽만 보고 있는지
  · 광축이 몇 도 숙어져 있는지 (목표 25~35°)
  · 센서가 옆으로 비틀렸는지 (롤, 목표 0°)
  · 센서 높이

  전부 초록(OK)이 되면 Ctrl+C 로 끝내고 --calibrate 를 돌린다.

실행:  python3 tools/aim.py
"""

import math
import os
import sys
import time

import numpy as np
import yaml

_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_R, 'rapa2'))
from tof_cliff_detector import ZoneGeometry, SerialFrameSource   # noqa: E402

GREEN, YELLOW, RED, DIM, RST = '\033[92m', '\033[93m', '\033[91m', '\033[2m', '\033[0m'

TILT_MIN, TILT_MAX = 25.0, 35.0     # 권장 광축 범위
ROLL_TOL = 5.0                      # 허용 비틀림


def estimate(med_m, scfg):
    """측정 격자에서 (롤, 광축각, 높이, 잔차) 를 추정한다."""
    n = med_m.shape[0]
    ok = np.isfinite(med_m) & (med_m > 0)
    if ok.sum() < 12:
        return None

    # --- 롤 ---
    # 평지를 보는 정렬된 센서라면 거리는 '행'을 따라서만 변해야 한다.
    # 거리를 d ≈ a*row + b*col + c 로 근사했을 때 b 가 크면 옆으로 비틀린 것.
    rr, cc = np.mgrid[0:n, 0:n]
    A = np.stack([rr[ok], cc[ok], np.ones(int(ok.sum()))], axis=1)
    coef, *_ = np.linalg.lstsq(A, med_m[ok], rcond=None)
    a, b = float(coef[0]), float(coef[1])
    roll = math.degrees(math.atan2(b, abs(a))) if abs(a) > 1e-9 else 90.0

    # --- 광축각 / 높이 ---
    # 시야에 낭떠러지나 먼 벽이 섞여 있으면 그것까지 한 평면으로 끼워 맞추다가
    # 엉뚱한 각도가 나온다. 가까운 셀(=지면 후보)만 쓰고, 한 번 더 이상치를 걸러낸다.
    # 1m 이내 셀만 지면 후보로 본다. 낭떠러지 너머 바닥이나 먼 벽이 섞이면
    # 그것까지 한 평면으로 맞추려다 엉뚱한 각도가 나온다.
    ground = ok & (med_m < 1.0)
    if ground.sum() < 10:
        return roll, None, None, None, ok

    best = None
    for row0_top in (True, False):
        for tilt in np.arange(3.0, 80.01, 0.5):
            s = dict(scfg, row0_is_top=row0_top, tilt_deg=float(tilt),
                     mount_height_m=1.0)
            geo = ZoneGeometry(s, 10 ** 6)
            m = ground & (geo.nz > 1e-6)
            if m.sum() < 10:
                continue
            # 평지라면 d * (-v_z) 가 모든 셀에서 같은 값(=높이)이어야 한다.
            # 그 흩어짐이 가장 작은 (행방향, 각도) 조합이 실제 형상이다.
            # 이상치를 걸러내면 한 행만 남기고 잔차가 0이 되는 가짜 해가 이기므로
            # 후보 셀 전체에 대해 그대로 평가한다.
            hs = med_m[m] * geo.nz[m]
            h = float(np.median(hs))
            resid = float(np.median(np.abs(hs - h)))
            if 0.05 < h < 1.5 and (best is None or resid < best[3]):
                best = (row0_top, float(tilt), h, resid)
    if best is None:
        return roll, None, None, None, ok
    return roll, best[1], best[2], best[3], ok


def main():
    cfg = yaml.safe_load(open(os.path.join(_R, 'config', 'cliff_config.yaml'),
                              encoding='utf-8'))
    scfg = cfg['sensor']
    src = SerialFrameSource(scfg['port'], int(scfg['baud']), int(scfg['grid']))

    buf = []
    try:
        while True:
            f = src.read_frame()
            if f is None:
                print('프레임 수신 실패'); break
            buf.append(f)
            if len(buf) > 5:
                buf.pop(0)
            if len(buf) < 3:
                continue

            stack = np.array(buf, dtype=float)
            with np.errstate(all='ignore'):
                med = np.nanmedian(np.where(stack > 0, stack, np.nan), axis=0) / 1000.0

            out = ['\033[2J\033[H', '센서 조준 도우미   (Ctrl+C 로 종료)', '=' * 52, '']
            n = med.shape[0]
            for r in range(n):
                cells = []
                for c in range(n):
                    v = med[r, c]
                    if not np.isfinite(v):
                        cells.append(DIM + ' ----' + RST)
                    elif v < 1.0:
                        cells.append(GREEN + '{:5.0f}'.format(v * 1000) + RST)
                    else:
                        cells.append(DIM + '{:5.0f}'.format(v * 1000) + RST)
                out.append('  R{} '.format(r) + ''.join(cells))
            out.append('')
            out.append(DIM + '  초록 = 1m 이내(바닥일 가능성), 회색 = 먼 곳(벽)' + RST)
            out.append('')

            res = estimate(med, scfg)
            if res is None:
                out.append(RED + '  유효한 측정값이 너무 적습니다' + RST)
            else:
                roll, tilt, h, resid, ok = res
                near = int(np.sum(np.isfinite(med) & (med < 1.0)))

                def mark(good):
                    return GREEN + ' OK  ' + RST if good else RED + ' 조정 ' + RST

                if tilt is None:
                    out.append(RED + '  평면으로 설명이 안 됩니다 — 바닥이 안 보입니다' + RST)
                else:
                    ok_tilt = TILT_MIN <= tilt <= TILT_MAX
                    ok_roll = abs(roll) <= ROLL_TOL
                    ok_near = near >= 32
                    ok_res = resid is not None and resid < 0.020

                    out.append('  {} 바닥 보이는 셀   {:2d}/64      (목표 32 이상)'
                               .format(mark(ok_near), near))
                    # 바닥이 거의 안 보이면 각도 추정은 의미가 없다.
                    # 엉뚱한 숫자를 보여주면 그 숫자를 쫓아가게 되므로 감춘다.
                    if near < 8 or resid is None or resid > 0.030:
                        ok_tilt = False
                        out.append('  {} 광축 숙인 각도   측정불가    '
                                   '(바닥이 안 보여 계산 못 함)'.format(mark(False)))
                    else:
                        out.append('  {} 광축 숙인 각도   {:5.1f}°     (목표 {:.0f}~{:.0f}°)'
                                   .format(mark(ok_tilt), tilt, TILT_MIN, TILT_MAX))
                    out.append('  {} 옆으로 비틀림    {:+5.1f}°     (목표 0°, ±{:.0f} 이내)'
                               .format(mark(ok_roll), roll, ROLL_TOL))
                    out.append('  {} 평면 잔차        {:5.1f}mm    (목표 20 미만)'
                               .format(mark(ok_res), resid * 1000))
                    out.append('       센서 높이        {:5.1f}cm'.format(h * 100))
                    out.append('')
                    if ok_tilt and ok_roll and ok_near and ok_res:
                        out.append(GREEN + '  ▶ 준비 완료 — 이 상태로 고정하고 '
                                           '--calibrate 를 돌리세요' + RST)
                    else:
                        if not ok_near or (tilt is not None and tilt < TILT_MIN):
                            out.append(YELLOW + '  → 더 숙이세요 (앞이 아니라 발밑을 보게)' + RST)
                        elif tilt > TILT_MAX:
                            out.append(YELLOW + '  → 너무 숙였습니다, 조금 세우세요' + RST)
                        if not ok_roll:
                            side = '왼쪽' if roll > 0 else '오른쪽'
                            out.append(YELLOW + '  → 옆으로 비틀렸습니다 — {}이 내려가 있습니다. '
                                                '기판 가로줄을 수평으로'.format(side) + RST)

            sys.stdout.write('\n'.join(out) + '\n')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        print('\n종료')


if __name__ == '__main__':
    main()
