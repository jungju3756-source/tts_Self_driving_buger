#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tof_cliff_detector.py  —  라파2 / 산출물 ①

VL53L8CX 8x8 ToF 어레이를 읽어 '지면이 꺼진 곳(단차·낭떠러지)'을 판정하고
MQTT 로 이벤트를 발행한다. ROS2 를 사용하지 않는다.

판정 원리
---------
센서는 지면에서 h 만큼 높은 곳에, 수평 기준 tilt 만큼 아래로 기울어 달려 있다.
각 셀(row, col)은 고유한 시선 벡터 v 를 가지므로, 지면이 평평하다면 그 셀이
측정해야 할 거리는 기하학적으로 정해진다.

    d_expected = h / (-v_z)

실제 측정거리 d 가 이보다 멀다면, 그 시선이 지면 아래를 뚫고 지나간 것이다.
지면 아래로 꺼진 깊이는 수직 성분만 뽑아내면 된다.

    depth = d * (-v_z) - h          [m]   (양수 = 지면보다 아래)

즉 '거리가 멀어졌다' 가 아니라 '실제로 몇 cm 꺼졌다' 로 판정하므로,
행마다 기대거리가 다른 문제를 자동으로 흡수한다.

오탐 억제
---------
  공간 : 4-이웃으로 연결된 낙차 셀이 min_cluster 개 이상일 때만 인정
  시간 : 최근 window 프레임 중 set_votes 회 이상 검출돼야 위험 확정
  해제 : 연속 clear_frames 프레임 깨끗해야 해제 (히스테리시스)
  무응답: 낭떠러지는 '무응답'으로도 나타나지만 검은 바닥도 그렇다.
          → 무응답만으로 판정할 때는 더 큰 군집(invalid_min_cluster)을 요구

사용법
------
  python3 tof_cliff_detector.py                      # 정상 구동
  python3 tof_cliff_detector.py --calibrate 60       # 평지에서 기준면 캘리브레이션
  python3 tof_cliff_detector.py --source sim -v      # 센서 없이 알고리즘만 시험
  python3 tof_cliff_detector.py --no-mqtt -v         # 터미널 출력만

의존성:  pip3 install pyserial paho-mqtt pyyaml numpy
"""

import argparse
import json
import math
import os
import re
import signal
import sys
import time
from collections import deque

import numpy as np
import yaml

try:
    import serial
except ImportError:
    serial = None

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'cliff_config.yaml')


# =============================================================================
#  기하 모델
# =============================================================================
class ZoneGeometry:
    """8x8 각 셀의 시선 벡터와, 평지 가정에서의 기대 측정거리를 계산한다."""

    def __init__(self, scfg, max_valid_range_mm):
        n = int(scfg['grid'])
        pitch = math.radians(float(scfg['fov_deg']) / n)   # 셀 하나가 차지하는 각
        h = float(scfg['mount_height_m'])
        tilt = math.radians(float(scfg['tilt_deg']))
        yaw = math.radians(float(scfg['yaw_deg']))
        row0_top = bool(scfg.get('row0_is_top', True))
        col0_left = bool(scfg.get('col0_is_left', True))

        # 월드 좌표계 X=전진 Y=좌측 Z=위 에서 본 센서 축
        axis = np.array([math.cos(tilt), 0.0, -math.sin(tilt)])   # 광축(전방하단)
        up = np.array([math.sin(tilt), 0.0, math.cos(tilt)])      # 센서 이미지의 위쪽
        right = np.array([0.0, -1.0, 0.0])                        # 센서 이미지의 오른쪽
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

        self.n = n
        self.h = h
        self.col0_left = col0_left      # jump 모드에서 좌우 판단에 쓴다
        self.v = np.zeros((n, n, 3))
        self.nz = np.zeros((n, n))          # -v_z : 시선의 하강 성분
        self.d_exp = np.full((n, n), np.inf)  # 평지 기대 측정거리 [m]
        self.x_ahead = np.zeros((n, n))     # 그 셀이 보는 지면의 전방 거리 [m]
        self.y_lat = np.zeros((n, n))       # 그 셀이 보는 지면의 좌우 오프셋 [m] (좌 +)
        self.active = np.zeros((n, n), dtype=bool)  # 판정에 쓸 수 있는 셀

        max_range_m = float(max_valid_range_mm) / 1000.0

        for r in range(n):
            for c in range(n):
                # 광축 기준 각 오프셋
                beta = (3.5 - r) * pitch if row0_top else (r - 3.5) * pitch    # 위 +
                alpha = (c - 3.5) * pitch if col0_left else (3.5 - c) * pitch  # 우 +
                vec = axis + math.tan(alpha) * right + math.tan(beta) * up
                vec = Rz @ (vec / np.linalg.norm(vec))
                self.v[r, c] = vec

                if vec[2] < -1e-6:          # 아래를 향해야 지면과 만난다
                    d = h / (-vec[2])
                    self.nz[r, c] = -vec[2]
                    self.d_exp[r, c] = d
                    self.x_ahead[r, c] = d * vec[0]
                    self.y_lat[r, c] = d * vec[1]
                    self.active[r, c] = (d <= max_range_m)

    def summary(self):
        a = self.active
        if not a.any():
            return '유효 셀 없음 — mount_height_m / tilt_deg 를 확인하세요'
        return ('유효셀 {}/{} | 기대거리 {:.2f}~{:.2f} m | 전방 감지범위 {:.2f}~{:.2f} m'
                .format(int(a.sum()), self.n * self.n,
                        float(self.d_exp[a].min()), float(self.d_exp[a].max()),
                        float(self.x_ahead[a].min()), float(self.x_ahead[a].max())))


# =============================================================================
#  단차 판정기
# =============================================================================
class CliffDetector:

    def __init__(self, geo, dcfg):
        self.geo = geo
        # geometric : 장착 형상 + 기준면으로 '몇 cm 꺼졌나'를 계산 (정밀, 캘리브레이션 필요)
        # jump      : 한 프레임 안에서 거리가 갑자기 몇 배로 뛰는 지점을 찾음
        #             (장착 각도/높이를 몰라도 되고 캘리브레이션도 필요 없음)
        self.mode = str(dcfg.get('mode', 'geometric')).lower()
        self.jump_ratio = float(dcfg.get('jump_ratio', 2.5))
        self.base_ratio = float(dcfg.get('base_ratio', 1.5))
        self.max_ground_mm = float(dcfg.get('max_ground_mm', 800.0))
        self.min_depth_m = float(dcfg['min_cliff_depth_mm']) / 1000.0
        self.min_excess_m = float(dcfg['min_range_excess_mm']) / 1000.0
        self.sigma_k = float(dcfg.get('sigma_k', 3.0))
        self.use_invalid = bool(dcfg.get('treat_invalid_as_cliff', True))
        self.invalid_min_cluster = int(dcfg.get('invalid_min_cluster', 5))
        self.blind_min_rate = float(dcfg.get('blind_min_valid_rate', 0.9))
        self.blind_ok = None           # 무응답을 낭떠러지 신호로 믿어도 되는 셀
        self.min_cluster = int(dcfg['min_cluster'])
        self.center_band = float(dcfg['center_band_m'])

        self.window = int(dcfg['window'])
        self.set_votes = int(dcfg['set_votes'])
        self.clear_frames = int(dcfg['clear_frames'])

        self.hist = deque(maxlen=self.window)
        self.clear_run = 0
        self.latched = False
        self.last_hazard = None        # 래치 유지 중 참조할 마지막 유효 관측

        self.baseline = None           # (n,n) [m] — 평지 캘리브레이션 결과
        self.sigma = None              # (n,n) [m]

        n = self.geo.n
        self.last_drop = np.zeros((n, n), dtype=bool)   # 시각화용 셀 단위 판정
        self.last_blind = np.zeros((n, n), dtype=bool)

    # ---------------------------------------------------------------- 기준면
    def load_baseline(self, path):
        if not path or not os.path.exists(path):
            return False
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        n = self.geo.n
        try:
            base = np.array(data['baseline_m'], dtype=float).reshape(n, n)
            sig = np.array(data['sigma_m'], dtype=float).reshape(n, n)
        except Exception:
            return False
        # 평지에서조차 기준값을 못 뽑은 셀은 판정에서 아예 뺀다.
        # 그런 셀은 무응답이 낭떠러지 때문인지 원래 안 잡히는 것인지 구분할 수 없다.
        if self.mode == 'baseline':
            # 이 모드는 장착 각도·높이를 쓰지 않는다. 실측 기준면이 있는 셀이
            # 곧 판정 범위다 (max_valid_range_mm 로 행을 자르지 않는다).
            self.geo.active = np.isfinite(base)
        else:
            self.geo.active = self.geo.active & np.isfinite(base)

        # 무응답을 낭떠러지 신호로 써도 되는 셀만 골라 둔다.
        # 스침각이 얕은 먼 행처럼 평지에서도 응답이 자주 끊기는 셀은,
        # 무응답만으로 판정하면 평지에서 오탐이 난다.
        # (그런 셀도 값이 돌아올 때의 깊이 판정에는 계속 쓴다.)
        rate = data.get('valid_rate')
        if rate is not None:
            rate = np.array(rate, dtype=float).reshape(n, n)
            self.blind_ok = np.isfinite(rate) & (rate >= self.blind_min_rate)
        else:
            self.blind_ok = np.ones((n, n), dtype=bool)

        self.baseline = np.where(np.isfinite(base), base, self.geo.d_exp)
        self.sigma = np.where(np.isfinite(sig), sig, 0.0)
        return True

    # ---------------------------------------------------------------- 셀 판정
    def _cell_flags(self, grid_mm):
        """(낙차 셀, 무응답 셀, 셀별 깊이[m]) 를 돌려준다."""
        if self.mode == 'jump':
            return self._cell_flags_jump(grid_mm)
        if self.mode == 'baseline':
            return self._cell_flags_baseline(grid_mm)
        valid = np.isfinite(grid_mm) & (grid_mm > 0)
        d_m = np.where(valid, grid_mm / 1000.0, 0.0)

        ref = self.baseline if self.baseline is not None else self.geo.d_exp
        ref = np.where(np.isfinite(ref), ref, 0.0)

        excess = np.where(valid, d_m - ref, 0.0)      # 기준면보다 더 멀리 나간 양
        depth = excess * self.geo.nz                  # 그 양의 수직 성분 = 꺼진 깊이

        thr = np.full_like(excess, self.min_excess_m)
        if self.sigma is not None:
            thr = np.maximum(thr, self.sigma_k * self.sigma)

        drop = valid & self.geo.active & (depth > self.min_depth_m) & (excess > thr)
        blind = (~valid) & self.geo.active
        if self.blind_ok is not None:
            blind = blind & self.blind_ok
        return drop, blind, depth

    # ------------------------------------------------------- 셀별 기준면 판정
    def _cell_flags_baseline(self, grid_mm):
        """평지에서 학습한 '셀별 기준거리'와 비교한다.

        jump 모드가 오탐을 내는 이유는, 배열 위쪽 행이 지면을 스치듯 보기 때문에
        평지에서도 아래 행의 4~5배가 정상으로 나오기 때문이다. 센서가 조금이라도
        비틀려(롤) 있으면 같은 행 안에서도 좌우 열이 두 배씩 차이난다.
        하나의 문턱값으로는 이 정상 기하와 진짜 낙차를 가를 수 없다.

        그래서 셀마다 자기 기준값을 갖는다. 어떤 셀이 평지에서 늘 1.6m 를 찍는다면
        그 셀에게 1.6m 는 정상이고, 2.4m 이상이어야 낙차다. 롤도 스침각도 셀별
        기준값에 이미 녹아 있으므로 따로 보정할 필요가 없다.

        낙차 판정 조건은 둘 중 하나:
          * 그 셀 기준거리의 base_ratio 배보다 멀다 (지면이 꺼졌다)
          * 평소엔 잘 잡히는 셀인데 지금 응답이 없다 (허공이라 반사가 안 온다)
        """
        valid = np.isfinite(grid_mm) & (grid_mm > 0)
        d_m = np.where(valid, grid_mm / 1000.0, 0.0)
        ref = np.where(np.isfinite(self.baseline), self.baseline, 0.0)
        ok = self.geo.active & (ref > 0)

        # 셋 중 가장 너그러운 문턱을 쓴다 — 비율 / 절대여유 / 그 셀의 실측 노이즈.
        # 값이 프레임마다 크게 흔들리는 셀은 sigma 가 커져 스스로 판정에서 빠진다.
        thr = np.maximum(ref * self.base_ratio, ref + self.min_excess_m)
        if self.sigma is not None:
            thr = np.maximum(thr, ref + self.sigma_k * self.sigma)

        drop = valid & ok & (d_m > thr)
        blind = (~valid) & ok
        if self.blind_ok is not None:
            blind = blind & self.blind_ok
        depth = np.where(ok & valid, d_m - ref, 0.0)
        return drop, blind, depth

    # ------------------------------------------------------- 캘리브레이션 없는 판정
    def _cell_flags_jump(self, grid_mm):
        """한 프레임만 보고 지면이 끊기는 지점을 찾는다.

        각 열을 '가까운 행 → 먼 행' 순으로 훑는다. 평평한 바닥이라면 거리는
        완만하게 늘어난다. 어느 지점에서 갑자기 jump_ratio 배 넘게 뛰거나
        응답이 사라지면, 거기서 지면이 끊긴 것이고 그 너머는 전부 허공이다.

        장착 각도·높이를 몰라도 되고 기준면도 필요 없다. 대신 '몇 cm 꺼졌는지'
        같은 물리량은 알 수 없고, '여기서 끊겼다'만 알 수 있다.

        주의: 배열 위쪽 행은 수평선에 가까워서, 평평한 바닥에서도 거리가 몇 배씩
        뛴다. 그래서 직전 유효값이 max_ground_mm 안쪽일 때만 급변을 낙차로 친다.
        멀리서 일어나는 급변은 원래 그런 것이므로 무시한다.
        """
        n = grid_mm.shape[0]
        valid = np.isfinite(grid_mm) & (grid_mm > 0)
        cliff = np.zeros((n, n), dtype=bool)
        blind = np.zeros((n, n), dtype=bool)
        ratio = np.zeros((n, n))

        if valid.sum() < 8:
            return cliff, blind, ratio

        # 어느 쪽 끝이 '가까운 행'인지 데이터로 정한다 (row0_is_top 설정에 안 기댄다)
        first = np.median(grid_mm[0][valid[0]]) if valid[0].any() else np.inf
        last = np.median(grid_mm[n - 1][valid[n - 1]]) if valid[n - 1].any() else np.inf
        order = list(range(n)) if first <= last else list(range(n - 1, -1, -1))

        for c in range(n):
            prev = None
            broken = False
            for r in order:
                if broken:
                    # 끊긴 지점 너머는 전부 허공으로 본다
                    (cliff if valid[r, c] else blind)[r, c] = True
                    continue
                # 직전 지면이 이미 멀면 그 너머는 신뢰 구간 밖 — 이 열은 여기까지만 본다
                if prev is not None and prev > self.max_ground_mm:
                    break
                if not valid[r, c]:
                    # 지면을 보다가 응답이 사라진 것만 낙차로 친다.
                    # 처음부터 안 잡히는 셀은 그냥 사각지대일 뿐이다.
                    if prev is not None:
                        blind[r, c] = True
                        broken = True
                    continue
                d = float(grid_mm[r, c])
                if prev is not None:
                    ratio[r, c] = d / prev
                    if ratio[r, c] > self.jump_ratio:
                        cliff[r, c] = True
                        broken = True
                prev = d

        if not self.use_invalid:
            blind[:] = False
        return cliff, blind, ratio

    # ---------------------------------------------------------------- 군집화
    @staticmethod
    def _components(mask):
        """4-이웃 연결 성분 목록. 각 성분은 (row, col) 좌표 리스트."""
        n = mask.shape[0]
        seen = np.zeros_like(mask, dtype=bool)
        out = []
        for r0 in range(n):
            for c0 in range(n):
                if not mask[r0, c0] or seen[r0, c0]:
                    continue
                stack = [(r0, c0)]
                seen[r0, c0] = True
                comp = []
                while stack:
                    r, c = stack.pop()
                    comp.append((r, c))
                    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < n and 0 <= cc < n and mask[rr, cc] and not seen[rr, cc]:
                            seen[rr, cc] = True
                            stack.append((rr, cc))
                out.append(comp)
        return out

    # ---------------------------------------------------------------- 1프레임
    def evaluate(self, grid_mm):
        """한 프레임을 평가해 이벤트 dict 를 만든다. 시간 필터까지 적용."""
        drop, blind, depth = self._cell_flags(grid_mm)
        # 시각화(tools/tof_web.py)용 — 판정 결과를 셀 단위로 보관한다.
        self.last_drop, self.last_blind = drop, blind

        candidate = drop | blind if self.use_invalid else drop
        cells = []
        for comp in self._components(candidate):
            n_hard = sum(1 for (r, c) in comp if drop[r, c])
            # 실측으로 꺼짐이 확인된 셀이 충분하거나, 무응답 덩어리가 충분히 크거나
            if n_hard >= self.min_cluster or len(comp) >= self.invalid_min_cluster:
                cells.extend(comp)

        present = len(cells) > 0

        # ---- 시간 필터 (SET: M/N 투표, CLEAR: 연속 K프레임) ----
        self.hist.append(present)
        votes = sum(self.hist)
        if not self.latched:
            if votes >= self.set_votes:
                self.latched = True
                self.clear_run = 0
        else:
            self.clear_run = 0 if present else self.clear_run + 1
            if self.clear_run >= self.clear_frames:
                self.latched = False
                self.last_hazard = None

        obs = self._describe(cells, drop, blind, depth, grid_mm) if present else None
        if obs is not None:
            self.last_hazard = obs

        # 래치 중 순간적으로 놓쳐도 직전 관측을 유지한다 (지시가 깜빡이지 않도록)
        hazard = obs if obs is not None else (self.last_hazard if self.latched else None)

        ev = {
            'cliff': bool(self.latched and hazard is not None),
            'raw_present': present,
            'votes': int(votes),
            'window': int(self.window),
        }
        if ev['cliff']:
            ev.update(hazard)
        else:
            ev.update({'zone': 'NONE', 'n_cells': 0, 'range_m': None,
                       'depth_m': None, 'free_side': self._free_side(drop, blind),
                       'cells': []})
        return ev

    # ---------------------------------------------------------------- 서술
    def _describe(self, cells, drop, blind, depth, grid_mm):
        if self.mode in ('jump', 'baseline'):
            return self._describe_jump(cells, drop, blind, depth, grid_mm)

        y = np.array([self.geo.y_lat[r, c] for r, c in cells])
        x = np.array([self.geo.x_ahead[r, c] for r, c in cells])
        d = np.array([depth[r, c] if drop[r, c] else np.nan for r, c in cells])

        # 진행 통로(center_band) 안에 낙차가 걸리면 전방 단차 — 정지 대상
        if np.any(np.abs(y) <= self.center_band):
            zone = 'FRONT'
        else:
            zone = 'LEFT' if float(np.mean(y)) > 0.0 else 'RIGHT'

        n_active = max(int(self.geo.active.sum()), 1)
        return {
            'zone': zone,
            'n_cells': len(cells),
            'confidence': round(min(1.0, len(cells) / max(self.min_cluster * 2.0, 1.0)), 3),
            'coverage': round(len(cells) / n_active, 3),
            'range_m': round(float(np.min(x)), 3),            # 가장 가까운 낙차까지 전방거리
            'lateral_m': round(float(np.mean(y)), 3),
            'depth_m': (round(float(np.nanmax(d)), 3) if np.any(np.isfinite(d)) else None),
            'blind_cells': int(sum(1 for r, c in cells if blind[r, c])),
            'free_side': self._free_side(drop, blind),
            'cells': [[int(r), int(c)] for r, c in cells],
        }

    # ------------------------------------------------- 캘리브레이션 없는 서술
    def _describe_jump(self, cells, drop, blind, ratio, grid_mm):
        """jump 모드에서는 물리적 깊이를 모른다. 열 위치로 방향만 판단한다."""
        n = self.geo.n
        cols = np.array([c for _, c in cells], dtype=float)
        # C0 이 로봇 좌측이면 열 번호가 작을수록 왼쪽
        side = cols - (n - 1) / 2.0
        if not self.geo.col0_left:
            side = -side

        center = (n - 1) / 2.0
        if np.any(np.abs(cols - center) <= 1.0):     # 가운데 두 열에 걸리면 전방
            zone = 'FRONT'
        else:
            zone = 'LEFT' if float(np.mean(side)) < 0.0 else 'RIGHT'

        dists = [grid_mm[r, c] for r, c in cells if np.isfinite(grid_mm[r, c])]
        rr = [ratio[r, c] for r, c in cells if ratio[r, c] > 0]

        return {
            'zone': zone,
            'n_cells': len(cells),
            'confidence': round(min(1.0, len(cells) / max(self.min_cluster * 2.0, 1.0)), 3),
            'coverage': round(len(cells) / float(n * n), 3),
            # jump 모드의 range_m 은 '낙차가 시작된 지점까지의 측정거리'다.
            # 기하 모델을 안 쓰므로 전방 수평거리가 아니라 시선 방향 거리다.
            'range_m': (round(float(min(dists)) / 1000.0, 3) if dists else None),
            'lateral_m': None,
            'depth_m': None,                      # 깊이는 알 수 없음
            'jump': (round(float(max(rr)), 2) if rr else None),
            'blind_cells': int(sum(1 for r, c in cells if blind[r, c])),
            'free_side': self._free_side(drop, blind),
            'cells': [[int(r), int(c)] for r, c in cells],
        }

    def _free_side(self, drop, blind):
        """좌/우 중 '멀쩡한 지면'이 더 많은 쪽 — 전방 단차일 때 우회 방향 힌트."""
        safe = (~drop) & (~blind)
        if self.mode in ('jump', 'baseline'):
            n = self.geo.n
            half = n // 2
            left = int(np.sum(safe[:, :half]))
            right = int(np.sum(safe[:, half:]))
            if not self.geo.col0_left:
                left, right = right, left
        else:
            safe = safe & self.geo.active
            left = int(np.sum(safe & (self.geo.y_lat > 0)))
            right = int(np.sum(safe & (self.geo.y_lat < 0)))
        if left == right:
            return 'NONE'
        return 'LEFT' if left > right else 'RIGHT'


# =============================================================================
#  프레임 소스
# =============================================================================
class SerialFrameSource:
    """Pico 펌웨어 출력 파싱.

        Frame #123
        R0  412  418  ----  405 ...      (8개, mm, '----' = 무응답)
        ...
        R7  ...
    """

    def __init__(self, port, baud, n=8):
        if serial is None:
            raise RuntimeError('pyserial 이 없습니다 — pip3 install pyserial')
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.n = n
        self._grid = np.full((n, n), np.nan)
        self._in_frame = False
        self._row_re = re.compile(r'^R(\d)\s+(.*)')

    def read_frame(self):
        """완성된 프레임 1장을 돌려준다. 타임아웃이면 None."""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            if line.startswith('Frame #'):
                self._in_frame = True
                self._grid = np.full((self.n, self.n), np.nan)
                continue
            if not self._in_frame:
                continue
            m = self._row_re.match(line)
            if not m:
                continue
            r = int(m.group(1))
            parts = m.group(2).split()
            if len(parts) != self.n or r >= self.n:
                continue
            for c, p in enumerate(parts):
                if p == '----':
                    self._grid[r, c] = np.nan
                else:
                    try:
                        self._grid[r, c] = float(int(p))
                    except ValueError:
                        self._grid[r, c] = np.nan
            if r == self.n - 1:
                self._in_frame = False
                return self._grid.copy()
        return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SimFrameSource:
    """하드웨어 없이 알고리즘을 검증하기 위한 합성 프레임.

    처음 3초는 평지, 이후 오른쪽 절반이 사라지는(낭떠러지) 상황을 반복한다.
    """

    def __init__(self, geo, period_s=6.0, noise_mm=8.0):
        self.geo = geo
        self.t0 = time.time()
        self.period = period_s
        self.noise = noise_mm

    def read_frame(self):
        time.sleep(1.0 / 15.0)
        n = self.geo.n
        g = self.geo.d_exp * 1000.0
        g = np.where(np.isfinite(g), g, np.nan)
        g = g + np.random.normal(0.0, self.noise, size=(n, n))

        phase = (time.time() - self.t0) % self.period
        if phase > self.period * 0.5:          # 낭떠러지 구간
            drop_cols = slice(n // 2, n)
            far_rows = slice(0, n // 2)        # 먼 쪽 행부터 지면이 사라진다
            sub = g[far_rows, drop_cols]
            # 절반은 아주 먼 값, 절반은 무응답 — 실제 낭떠러지의 전형적 모습
            g[far_rows, drop_cols] = np.where(
                np.random.rand(*sub.shape) < 0.5, np.nan, sub * 2.2)
        g[~self.geo.active] = np.nan
        return g

    def close(self):
        pass


# =============================================================================
#  MQTT
# =============================================================================
class MqttPublisher:

    def __init__(self, mcfg, client_id, lwt_topic, lwt_payload):
        if mqtt is None:
            raise RuntimeError('paho-mqtt 가 없습니다 — pip3 install paho-mqtt')
        self.client = _make_client(client_id)
        user = mcfg.get('username') or ''
        if user:
            self.client.username_pw_set(user, mcfg.get('password') or '')
        self.client.will_set(lwt_topic, json.dumps(lwt_payload), qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.connected = False
        self._host = mcfg['host']
        self._port = int(mcfg['port'])
        self._keepalive = int(mcfg.get('keepalive', 15))
        self.client.connect_async(self._host, self._port, self._keepalive)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, *a):
        self.connected = (rc == 0)
        print('[mqtt] {} rc={}'.format('연결됨' if rc == 0 else '연결 실패', rc), flush=True)

    def _on_disconnect(self, client, userdata, rc, *a):
        self.connected = False
        print('[mqtt] 연결 끊김 rc={} — 자동 재연결 시도'.format(rc), flush=True)

    def publish(self, topic, payload, qos=0, retain=False):
        self.client.publish(topic, json.dumps(payload, allow_nan=False),
                            qos=qos, retain=retain)

    def close(self, final_status_topic=None, final_status=None):
        try:
            if final_status_topic:
                self.client.publish(final_status_topic, json.dumps(final_status),
                                    qos=1, retain=True)
                time.sleep(0.2)
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass


def _make_client(client_id):
    """paho-mqtt 1.x / 2.x 양쪽에서 동작하는 클라이언트 생성."""
    try:
        from paho.mqtt.client import CallbackAPIVersion
        return mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        return mqtt.Client(client_id=client_id)


# =============================================================================
#  캘리브레이션
# =============================================================================
def calibrate(source, geo, n_frames, out_path):
    """평평한 바닥 위에서 기준 지면거리와 셀별 노이즈를 측정해 저장한다."""
    print('[cal] 평지 위에서 {} 프레임 수집 — 로봇을 움직이지 마세요'.format(n_frames))
    n = geo.n
    buf = []
    got = 0
    while got < n_frames:
        f = source.read_frame()
        if f is None:
            print('[cal] 프레임 수신 실패 — 센서 연결 확인')
            return False
        buf.append(f)
        got += 1
        if got % 10 == 0:
            print('[cal] {}/{}'.format(got, n_frames))

    stack = np.array(buf, dtype=float) / 1000.0        # [프레임, n, n] (m)
    # 셀별 응답률 — 평지에서 얼마나 안정적으로 값이 돌아오는가.
    # 이 값이 낮은 셀은 '무응답 = 낭떠러지' 규칙에서 제외된다.
    valid_rate = np.mean(np.isfinite(stack) & (stack > 0), axis=0)
    base = np.full((n, n), np.nan)
    sig = np.full((n, n), np.nan)
    for r in range(n):
        for c in range(n):
            col = stack[:, r, c]
            col = col[np.isfinite(col) & (col > 0)]
            # 절반 이상 유효해야 신뢰
            if len(col) >= max(3, n_frames // 2):
                base[r, c] = float(np.median(col))
                sig[r, c] = float(np.std(col))

    ok = int(np.sum(np.isfinite(base)))
    ok_active = int(np.sum(np.isfinite(base) & geo.active))
    # 판정에 실제로 쓰는 셀(active)만 비교한다. 수평선 근처 행은 기대거리가
    # 수십 m 라, 평균에 섞이면 숫자가 무의미해진다.
    model_err = np.abs(base - geo.d_exp)[geo.active]
    model_err = model_err[np.isfinite(model_err)]

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({
            'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'frames': int(n_frames),
            'mount_height_m': float(geo.h),
            'baseline_m': [[None if not np.isfinite(v) else round(float(v), 4)
                            for v in row] for row in base],
            'sigma_m': [[None if not np.isfinite(v) else round(float(v), 4)
                         for v in row] for row in sig],
            'valid_rate': [[round(float(v), 3) for v in row] for row in valid_rate],
        }, f, allow_unicode=True, sort_keys=False)

    print('[cal] 저장: {}'.format(out_path))
    print('[cal] 유효 셀 {}/{}  (판정 대상 {}/{})'
          .format(ok, n * n, ok_active, int(geo.active.sum())))
    flaky = int(np.sum(geo.active & (valid_rate < 0.9)))
    if flaky:
        print('[cal] 응답률 90% 미만 셀 {}개 — 무응답 판정에서 제외됩니다'.format(flaky))
    if len(model_err):
        print('[cal] 기하모델과의 평균 오차 {:.1f} mm (최대 {:.1f} mm) — 판정 대상 셀 기준'
              .format(float(model_err.mean()) * 1000, float(model_err.max()) * 1000))
        if float(model_err.mean()) > 0.05:
            print('[cal] ※ 오차가 큽니다 — mount_height_m / tilt_deg 실측값을 다시 확인하세요')
    return True


# =============================================================================
#  메인
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description='ToF 단차(낭떠러지) 감지기 — 라파2')
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--source', choices=['serial', 'sim'], default='serial')
    ap.add_argument('--calibrate', type=int, metavar='N',
                    help='평지에서 N 프레임을 모아 기준면을 만들고 종료')
    ap.add_argument('--no-mqtt', action='store_true', help='MQTT 없이 콘솔 출력만')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    root = os.path.dirname(os.path.abspath(args.config))
    root = os.path.dirname(root)          # config/ 의 상위 = 프로젝트 루트
    scfg, dcfg, mcfg = cfg['sensor'], cfg['detect'], cfg['mqtt']
    topics = mcfg['topics']

    mode = str(dcfg.get('mode', 'geometric')).lower()
    geo = ZoneGeometry(scfg, dcfg['max_valid_range_mm'])
    if mode == 'jump':
        print('[mode] jump — 프레임 내 거리 급변으로 판정 (캘리브레이션·장착값 불필요)')
        print('[mode] 급변 배율 {:.1f}배, 최소 군집 {}셀'
              .format(float(dcfg.get('jump_ratio', 2.5)), int(dcfg['min_cluster'])))
    elif mode == 'baseline':
        print('[mode] baseline — 셀별 평지 기준거리와 비교 (장착 각도·높이 불필요)')
        print('[mode] 기준거리의 {:.1f}배 이상이면 낙차, 최소 군집 {}셀'
              .format(float(dcfg.get('base_ratio', 1.5)), int(dcfg['min_cluster'])))
    else:
        print('[geo] ' + geo.summary())
        if not geo.active.any():
            return 2

    # ---- 프레임 소스 ----
    if args.source == 'sim':
        source = SimFrameSource(geo)
        print('[src] 시뮬레이션 모드')
    else:
        source = SerialFrameSource(scfg['port'], int(scfg['baud']), geo.n)
        print('[src] 시리얼 {} @ {}'.format(scfg['port'], scfg['baud']))

    # ---- 캘리브레이션만 하고 종료 ----
    if args.calibrate:
        bpath = scfg.get('baseline_file') or 'config/ground_baseline.yaml'
        if not os.path.isabs(bpath):
            bpath = os.path.join(root, bpath)
        ok = calibrate(source, geo, args.calibrate, bpath)
        source.close()
        return 0 if ok else 1

    det = CliffDetector(geo, dcfg)
    bpath = scfg.get('baseline_file')
    if bpath and not os.path.isabs(bpath):
        bpath = os.path.join(root, bpath)
    if mode == 'jump':
        pass                       # jump 모드는 기준면을 쓰지 않는다
    elif mode == 'baseline':
        if not det.load_baseline(bpath):
            print('[cal] 기준면이 없습니다: {}'.format(bpath))
            print('[cal] 평지에 두고 먼저 실행하세요:  --calibrate 60')
            return 2
        print('[cal] 기준면 적용: {} (판정 대상 {}셀)'
              .format(bpath, int(geo.active.sum())))
    elif det.load_baseline(bpath):
        print('[cal] 기준면 적용: {}'.format(bpath))
    else:
        print('[cal] 기준면 없음 — 기하 모델로 판정 (--calibrate 권장)')

    # ---- MQTT ----
    pub = None
    sensor_id = scfg.get('id', 'tof_front')
    if not args.no_mqtt:
        pub = MqttPublisher(
            mcfg, 'cliff-detector-' + sensor_id,
            topics['status'], {'sensor': sensor_id, 'state': 'offline'})

    running = {'go': True}

    def _stop(*_a):
        running['go'] = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    event_period = 1.0 / float(dcfg['event_hz'])
    frame_hz = float(dcfg.get('frame_hz', 0))
    frame_period = (1.0 / frame_hz) if frame_hz > 0 else None

    seq = 0
    last_event_t = 0.0
    last_frame_t = 0.0
    last_status_t = 0.0
    last_printed = None
    miss = 0

    print('[run] 시작 — Ctrl+C 로 종료')
    while running['go']:
        grid = source.read_frame()
        now = time.time()

        if grid is None:
            miss += 1
            if miss % 5 == 1:
                print('[warn] 프레임 수신 안 됨 ({}회)'.format(miss))
            if pub and now - last_status_t > 1.0:
                pub.publish(topics['status'],
                            {'sensor': sensor_id, 'state': 'no_data', 'ts': now},
                            qos=1, retain=True)
                last_status_t = now
            continue
        miss = 0

        ev = det.evaluate(grid)

        if now - last_event_t >= event_period:
            last_event_t = now
            seq += 1
            payload = {'seq': seq, 'ts': round(now, 3), 'sensor': sensor_id}
            payload.update(ev)
            if pub:
                pub.publish(topics['cliff'], payload, qos=1)
            if args.verbose:
                key = (ev['cliff'], ev['zone'])
                if key != last_printed or seq % 30 == 0:
                    print('[{}] cliff={} zone={} cells={} range={} depth={} free={} votes={}/{}'
                          .format(seq, ev['cliff'], ev['zone'], ev['n_cells'],
                                  ev.get('range_m'), ev.get('depth_m'),
                                  ev.get('free_side'), ev['votes'], ev['window']))
                    last_printed = key

        if pub and frame_period and now - last_frame_t >= frame_period:
            last_frame_t = now
            flat = [None if not np.isfinite(v) else int(v) for v in grid.reshape(-1)]
            # 시각화가 '왜 그렇게 판정했는지'까지 보여줄 수 있도록 셀 단위 결과를 같이 싣는다.
            roi = geo.active if geo.active.any() else np.ones_like(geo.active, dtype=bool)
            inroi = [v for v, m in zip(flat, roi.reshape(-1)) if m and v is not None]
            pub.publish(topics['frame'],
                        {'ts': round(now, 3), 'sensor': sensor_id,
                         'grid_mm': flat, 'rows': geo.n, 'cols': geo.n,
                         'roi': [bool(m) for m in roi.reshape(-1)],
                         'roi_min_mm': min(inroi) if inroi else None,
                         'drop': [bool(x) for x in det.last_drop.reshape(-1)],
                         'blind': [bool(x) for x in det.last_blind.reshape(-1)],
                         'cliff': bool(ev['cliff']), 'zone': ev['zone'],
                         'row0_is_top': bool(scfg.get('row0_is_top', True)),
                         'col0_is_left': bool(scfg.get('col0_is_left', True))}, qos=0)

        if pub and now - last_status_t > 1.0:
            pub.publish(topics['status'],
                        {'sensor': sensor_id, 'state': 'online', 'ts': round(now, 3)},
                        qos=1, retain=True)
            last_status_t = now

    print('\n[run] 종료 중...')
    if pub:
        pub.close(topics['status'], {'sensor': sensor_id, 'state': 'offline',
                                     'ts': round(time.time(), 3)})
    source.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
