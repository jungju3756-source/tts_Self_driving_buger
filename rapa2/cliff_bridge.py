#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cliff_bridge.py  —  라파2 / 산출물 ②

tof_cliff_detector.py 가 MQTT 로 올린 단차 이벤트를 받아,
라파1이 그대로 실행할 수 있는 '회피 지시' 로 변환해 다시 MQTT 로 발행한다.
ROS2 를 사용하지 않는다.

역할 분담
---------
  detector : "어디에 몇 cm 꺼진 곳이 있다"   (인식)
  bridge   : "그러면 지금 무엇을 해야 한다"  (판단·중재)   ← 이 파일
  guard    : "그 지시대로 모터 명령을 고친다" (실행, 라파1)

지시 어휘 (action)
------------------
  CLEAR          위험 없음 — 조이스틱 입력을 그대로 통과
  STOP           전방 단차 — 전진 금지, 완전 정지
  DETOUR_LEFT    정지 후, 안전한 좌측으로 우회 주행
  DETOUR_RIGHT   정지 후, 안전한 우측으로 우회 주행
  CORRECT_LEFT   우측 단차 — 전진은 유지하고 좌측으로 방향 보정
  CORRECT_RIGHT  좌측 단차 — 전진은 유지하고 우측으로 방향 보정
  DEGRADED       센서 이상 — 위험은 없었으므로 감속 주행으로 강등
  FAULT          위험 감지 중 센서가 끊김 — 안전측으로 정지 유지

설계 원칙 (PDF 슬라이드 3·4)
---------------------------
  · 전방 단차는 '정지 및 우회', 측면 단차는 '전진 유지 + 반대편 보정'.
    어느 경우에도 후진하지 않는다.
  · 안전 개입은 항상 조이스틱보다 우선한다 (하드 오버라이드).
  · 지시는 이벤트 발생 시점뿐 아니라 publish_hz 주기로 계속 재전송한다.
    라파1은 이 신호가 끊기는 것 자체를 이상으로 감지할 수 있어야 한다.

사용법:  python3 cliff_bridge.py [-v]
의존성:  pip3 install paho-mqtt pyyaml
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import yaml

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'cliff_config.yaml')

# 조이스틱을 덮어쓰는 지시들 — 라파1 로그/판단용
OVERRIDE_ACTIONS = {'STOP', 'DETOUR_LEFT', 'DETOUR_RIGHT',
                    'CORRECT_LEFT', 'CORRECT_RIGHT', 'FAULT'}


def _make_client(client_id):
    """paho-mqtt 1.x / 2.x 양쪽에서 동작하는 클라이언트 생성."""
    try:
        from paho.mqtt.client import CallbackAPIVersion
        return mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        return mqtt.Client(client_id=client_id)


class CliffBridge:

    def __init__(self, cfg, verbose=False):
        self.cfg = cfg
        self.verbose = verbose
        self.mcfg = cfg['mqtt']
        self.ccfg = cfg['command']
        self.topics = self.mcfg['topics']

        self.min_stop_s = float(self.ccfg['min_stop_s'])
        self.release_hold_s = float(self.ccfg['release_hold_s'])
        self.side_gain = float(self.ccfg['side_correct_gain'])
        self.stale_s = float(self.ccfg['event_stale_s'])
        self.period = 1.0 / float(self.ccfg['publish_hz'])

        # ---- 공유 상태 (MQTT 수신 스레드 ↔ 발행 루프) ----
        self._lock = threading.Lock()
        self._event = None            # 최근 단차 이벤트
        self._event_t = 0.0           # 그 수신 시각
        self._detector_state = 'unknown'

        # ---- 상태머신 ----
        self.state = 'IDLE'           # IDLE | STOP | DETOUR | CORRECT | FAULT
        self.t_stop = 0.0             # 전방 단차로 정지에 들어간 시각
        self.t_clear = 0.0            # 위험이 사라진 시각
        self.detour_side = None       # 한 번 정한 우회 방향 (위험 해제까지 고정)
        self.hazard_active = False
        self.last_action = 'CLEAR'
        self.last_reason = 'init'
        self.seq = 0

        self.client = None

    # ------------------------------------------------------------------ MQTT
    def connect(self):
        if mqtt is None:
            raise RuntimeError('paho-mqtt 가 없습니다 — pip3 install paho-mqtt')
        self.client = _make_client('cliff-bridge')
        user = self.mcfg.get('username') or ''
        if user:
            self.client.username_pw_set(user, self.mcfg.get('password') or '')
        # bridge 가 죽으면 라파1이 즉시 알 수 있도록 유언장을 남긴다
        self.client.will_set(
            self.topics['command'],
            json.dumps({'action': 'FAULT', 'reason': 'bridge_offline',
                        'override': True, 'seq': -1}),
            qos=1, retain=False)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = lambda *a: print('[mqtt] 연결 끊김 — 재연결 시도',
                                                     flush=True)
        self.client.connect_async(self.mcfg['host'], int(self.mcfg['port']),
                                  int(self.mcfg.get('keepalive', 15)))
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, *a):
        if rc == 0:
            client.subscribe([(self.topics['cliff'], 1), (self.topics['status'], 1)])
            print('[mqtt] 연결됨 — 구독: {}, {}'.format(
                self.topics['cliff'], self.topics['status']), flush=True)
        else:
            print('[mqtt] 연결 실패 rc={}'.format(rc), flush=True)

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except Exception:
            return
        with self._lock:
            if msg.topic == self.topics['cliff']:
                self._event = data
                self._event_t = time.time()
            elif msg.topic == self.topics['status']:
                self._detector_state = data.get('state', 'unknown')

    # -------------------------------------------------------------- 상태머신
    def decide(self, now):
        """현재 시각 기준으로 내려야 할 지시를 결정한다."""
        with self._lock:
            ev = dict(self._event) if self._event else None
            ev_age = now - self._event_t if self._event else None
            det_state = self._detector_state

        # ---- 센서/링크 이상 ----
        link_ok = (det_state == 'online') and (ev is not None) and (ev_age <= self.stale_s)
        if not link_ok:
            reason = 'detector_{}'.format(det_state) if det_state != 'online' \
                else 'event_stale_{:.2f}s'.format(ev_age if ev_age else -1)
            if self.hazard_active:
                # 위험이 살아 있는 채로 눈을 잃었다 → 절대 통과시키지 않는다
                self.state = 'FAULT'
                return self._cmd('FAULT', reason, ev)
            self.state = 'IDLE'
            self.detour_side = None
            return self._cmd('DEGRADED', reason, ev)

        cliff = bool(ev.get('cliff'))
        zone = ev.get('zone', 'NONE')

        # ---- 위험 해제 ----
        if not cliff:
            if self.hazard_active:
                self.hazard_active = False
                self.t_clear = now
            if now - self.t_clear < self.release_hold_s and self.last_action != 'CLEAR':
                # 지시가 깜빡이지 않도록 짧게 유지한 뒤 놓아준다
                return self._cmd(self.last_action, 'release_hold', ev)
            self.state = 'IDLE'
            self.detour_side = None
            return self._cmd('CLEAR', 'no_cliff', ev)

        # ---- 위험 있음 ----
        if not self.hazard_active:
            self.hazard_active = True

        if zone == 'FRONT':
            # 전방 단차 : 먼저 확실히 세우고, 그 다음 넓은 쪽으로 우회
            if self.state not in ('STOP', 'DETOUR'):
                self.state = 'STOP'
                self.t_stop = now
            if now - self.t_stop < self.min_stop_s:
                return self._cmd('STOP', 'cliff_front', ev)

            # 우회 방향은 한 번 정하면 위험이 해제될 때까지 바꾸지 않는다.
            # free_side 는 프레임마다 흔들릴 수 있는데, 그때마다 방향을 뒤집으면
            # 좌우로 떨기만 하다가 정작 빠져나가지 못한다.
            if self.detour_side is None:
                side = ev.get('free_side', 'NONE')
                if side not in ('LEFT', 'RIGHT'):
                    # 양쪽 다 위험하거나 판단 불가 → 정지 유지가 안전
                    return self._cmd('STOP', 'cliff_front_no_exit', ev)
                self.detour_side = side
            self.state = 'DETOUR'
            return self._cmd('DETOUR_' + self.detour_side, 'cliff_front_detour', ev)

        # 측면 단차 : 전진은 유지하고 반대편으로 방향 보정
        self.state = 'CORRECT'
        self.detour_side = None
        if zone == 'LEFT':
            return self._cmd('CORRECT_RIGHT', 'cliff_left', ev)
        return self._cmd('CORRECT_LEFT', 'cliff_right', ev)

    def _cmd(self, action, reason, ev):
        self.seq += 1
        now = time.time()
        cmd = {
            'seq': self.seq,
            'ts': round(now, 3),
            'action': action,
            'reason': reason,
            'override': action in OVERRIDE_ACTIONS,
            'gain': self.side_gain if action.startswith('CORRECT') else 1.0,
            'detour_side': self.detour_side,
            # 라파1 워치독용 — 이 시각까지는 이 지시가 유효하다
            'valid_until': round(now + self.period * 3.0, 3),
        }
        if ev:
            cmd.update({
                'zone': ev.get('zone', 'NONE'),
                'range_m': ev.get('range_m'),
                'depth_m': ev.get('depth_m'),
                'free_side': ev.get('free_side', 'NONE'),
                'n_cells': ev.get('n_cells', 0),
                'detector_seq': ev.get('seq'),
            })
        self.last_action = action
        self.last_reason = reason
        return cmd

    # ------------------------------------------------------------------ 루프
    def run(self):
        running = {'go': True}

        def _stop(*_a):
            running['go'] = False
        signal.signal(signal.SIGINT, _stop)
        signal.signal(signal.SIGTERM, _stop)

        print('[run] 시작 — 발행: {} @ {:.0f}Hz'.format(
            self.topics['command'], 1.0 / self.period))

        prev_action = None
        next_t = time.time()
        while running['go']:
            now = time.time()
            cmd = self.decide(now)
            self.client.publish(self.topics['command'], json.dumps(cmd), qos=1)

            if cmd['action'] != prev_action:
                print('[{}] {} -> {}  ({}, zone={} range={} depth={})'.format(
                    cmd['seq'], prev_action, cmd['action'], cmd['reason'],
                    cmd.get('zone'), cmd.get('range_m'), cmd.get('depth_m')),
                    flush=True)
                prev_action = cmd['action']
            elif self.verbose and cmd['seq'] % 20 == 0:
                print('[{}] {} ({})'.format(cmd['seq'], cmd['action'], cmd['reason']),
                      flush=True)

            next_t += self.period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()      # 밀렸으면 주기 재동기화

        print('\n[run] 종료 — 마지막으로 FAULT 를 알립니다')
        try:
            self.client.publish(self.topics['command'], json.dumps({
                'seq': self.seq + 1, 'ts': round(time.time(), 3),
                'action': 'FAULT', 'reason': 'bridge_shutdown', 'override': True,
            }), qos=1)
            time.sleep(0.2)
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description='단차 이벤트 → 회피 지시 변환 — 라파2')
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    bridge = CliffBridge(cfg, verbose=args.verbose)
    bridge.connect()
    bridge.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
