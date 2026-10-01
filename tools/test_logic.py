#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""브로커 없이 bridge 상태머신 + guard 오버라이드 로직만 검증."""
import os, sys, time, types, yaml
_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_R, 'rapa2'))
sys.path.insert(0, os.path.join(_R, 'rapa1'))

from cliff_bridge import CliffBridge

cfg = yaml.safe_load(open(os.path.join(_R, 'config', 'cliff_config.yaml'), encoding='utf-8'))

FAIL = []
def check(label, got, want):
    ok = got == want
    print('  {} {:<38} got={:<16} want={}'.format('OK ' if ok else 'FAIL', label, str(got), str(want)))
    if not ok:
        FAIL.append(label)

def feed(b, t, ev, state='online'):
    b._event = ev
    b._event_t = t
    b._detector_state = state
    return b.decide(t)['action']

print('\n[1] 전방 단차 → 정지 유지 후 우회')
b = CliffBridge(cfg); t = 1000.0
check('위험 전', feed(b, t, {'cliff': False, 'zone': 'NONE'}), 'CLEAR')
ev_front = {'cliff': True, 'zone': 'FRONT', 'free_side': 'LEFT', 'range_m': 0.3}
check('단차 감지 직후', feed(b, t + 0.1, ev_front), 'STOP')
check('0.5s 경과 (min_stop 1.0s 이내)', feed(b, t + 0.5, ev_front), 'STOP')
check('1.2s 경과 → 넓은 쪽으로 우회', feed(b, t + 1.2, ev_front), 'DETOUR_LEFT')

print('\n[1-b] 우회 방향은 한 번 정하면 고정 (free_side 가 흔들려도)')
b = CliffBridge(cfg); t = 1500.0
feed(b, t, {'cliff': True, 'zone': 'FRONT', 'free_side': 'RIGHT'})
check('1.2s 후 우회 진입', feed(b, t + 1.2, {'cliff': True, 'zone': 'FRONT', 'free_side': 'RIGHT'}), 'DETOUR_RIGHT')
check('free_side 가 LEFT 로 뒤집혀도', feed(b, t + 1.3, {'cliff': True, 'zone': 'FRONT', 'free_side': 'LEFT'}), 'DETOUR_RIGHT')
check('free_side 가 NONE 이 되어도', feed(b, t + 1.4, {'cliff': True, 'zone': 'FRONT', 'free_side': 'NONE'}), 'DETOUR_RIGHT')
feed(b, t + 3.0, {'cliff': False, 'zone': 'NONE'})
feed(b, t + 4.0, {'cliff': False, 'zone': 'NONE'})   # 해제로 래치 풀림
feed(b, t + 5.0, {'cliff': True, 'zone': 'FRONT', 'free_side': 'LEFT'})
check('해제 후 새 위험은 다시 선택', feed(b, t + 6.2, {'cliff': True, 'zone': 'FRONT', 'free_side': 'LEFT'}), 'DETOUR_LEFT')

print('\n[2] 탈출로가 없으면 계속 정지')
b2 = CliffBridge(cfg); t = 2000.0
ev_noexit = {'cliff': True, 'zone': 'FRONT', 'free_side': 'NONE'}
feed(b2, t, ev_noexit)
check('양쪽 다 위험', feed(b2, t + 2.0, ev_noexit), 'STOP')

print('\n[3] 측면 단차 → 반대편 보정 (후진 아님)')
b3 = CliffBridge(cfg); t = 3000.0
check('좌측 단차', feed(b3, t, {'cliff': True, 'zone': 'LEFT', 'free_side': 'RIGHT'}), 'CORRECT_RIGHT')
b4 = CliffBridge(cfg)
check('우측 단차', feed(b4, t, {'cliff': True, 'zone': 'RIGHT', 'free_side': 'LEFT'}), 'CORRECT_LEFT')

print('\n[4] 해제 시 release_hold 후 CLEAR')
b5 = CliffBridge(cfg); t = 4000.0
feed(b5, t, ev_front)
check('해제 직후 (0.5s 유지구간)', feed(b5, t + 0.2, {'cliff': False, 'zone': 'NONE'}), 'STOP')
check('유지구간 경과', feed(b5, t + 1.0, {'cliff': False, 'zone': 'NONE'}), 'CLEAR')

print('\n[5] 센서/링크 이상')
b6 = CliffBridge(cfg); t = 5000.0
feed(b6, t, {'cliff': False, 'zone': 'NONE'})
check('detector offline (위험 없었음)', feed(b6, t, {'cliff': False}, state='offline'), 'DEGRADED')
b7 = CliffBridge(cfg); t = 6000.0
feed(b7, t, ev_front)
check('위험 중 detector offline', feed(b7, t + 0.1, ev_front, state='offline'), 'FAULT')
b8 = CliffBridge(cfg); t = 7000.0
feed(b8, t, ev_front)
b8._event_t = t - 3.0   # 이벤트가 3초째 안 옴
check('이벤트 stale', b8.decide(t)['action'], 'FAULT')

# ------------------------------------------------------------------ guard
print('\n[6] 라파1 오버라이드 결과 (linear.x, angular.z)')
from cliff_guard_node import CliffGuardNode
from geometry_msgs.msg import Twist

g = cfg['guard']
stub = types.SimpleNamespace(
    detour_lin=g['detour_linear'], detour_ang=g['detour_angular'],
    max_ang=g['max_angular'], degraded_scale=g['degraded_speed_scale'],
    allow_reverse=g['allow_reverse_when_stopped'])

def drive(action, lin, ang, gain=1.0):
    u = Twist(); u.linear.x = lin; u.angular.z = ang
    o = CliffGuardNode._apply(stub, action, gain, u)
    return (round(o.linear.x, 3), round(o.angular.z, 3))

check('CLEAR 전진 0.2 통과', drive('CLEAR', 0.2, 0.0), (0.2, 0.0))
check('STOP 중 전진 입력 → 0', drive('STOP', 0.2, 0.0), (0.0, 0.0))
check('STOP 중 후진 입력 → 허용', drive('STOP', -0.15, 0.0), (-0.15, 0.0))
check('STOP 중 제자리 회전 → 허용', drive('STOP', 0.0, 0.5), (0.0, 0.5))
check('DETOUR_LEFT + 사용자 전진', drive('DETOUR_LEFT', 0.2, 0.0), (0.08, 0.8))
check('DETOUR_LEFT + 사용자 정지', drive('DETOUR_LEFT', 0.0, 0.0), (0.0, 0.0))
check('CORRECT_RIGHT 전진 유지 + 우측 보정', drive('CORRECT_RIGHT', 0.2, 0.0, 0.6), (0.2, -0.48))
check('DEGRADED 감속', drive('DEGRADED', 0.2, 0.5), (0.06, 0.15))
check('각속도 상한 클리핑', drive('CORRECT_LEFT', 0.2, 1.0, 1.0), (0.2, 1.2))

print('\n' + ('=' * 50))
print('실패 {}건'.format(len(FAIL)) if FAIL else '전체 통과')
for f in FAIL:
    print('  - ' + f)
sys.exit(1 if FAIL else 0)
