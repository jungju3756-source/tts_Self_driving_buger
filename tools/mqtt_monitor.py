#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mqtt_monitor.py  —  디버깅/시연용 터미널 모니터

세 토픽을 한 화면에서 본다.
  · 8x8 원본 거리 격자 (단차로 판정된 셀은 X 로 표시)
  · detector 가 낸 단차 이벤트
  · bridge 가 낸 회피 지시

실행:  python3 tools/mqtt_monitor.py
"""

import argparse
import json
import os
import sys
import time

import yaml

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print('paho-mqtt 가 필요합니다 — pip3 install paho-mqtt')
    sys.exit(1)


DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'cliff_config.yaml')

STATE = {'frame': None, 'cliff': None, 'command': None, 'status': None,
         't': {'frame': 0, 'cliff': 0, 'command': 0}}


def _make_client(cid):
    try:
        from paho.mqtt.client import CallbackAPIVersion
        return mqtt.Client(CallbackAPIVersion.VERSION1, client_id=cid)
    except (ImportError, AttributeError):
        return mqtt.Client(client_id=cid)


def render(topics):
    ev = STATE['cliff']
    cmd = STATE['command']
    fr = STATE['frame']
    now = time.time()

    lines = []
    lines.append('=' * 62)
    lines.append(' 단차 감지 모니터   {}'.format(time.strftime('%H:%M:%S')))
    lines.append('=' * 62)

    st = STATE['status'] or {}
    lines.append(' detector : {:<10} (마지막 이벤트 {:.1f}s 전)'.format(
        st.get('state', '?'),
        now - STATE['t']['cliff'] if STATE['t']['cliff'] else -1))

    # ---- 8x8 격자 ----
    cliff_cells = set()
    if ev and ev.get('cells'):
        cliff_cells = {(int(r), int(c)) for r, c in ev['cells']}
    if fr:
        rows, cols = fr.get('rows', 8), fr.get('cols', 8)
        grid = fr.get('grid_mm', [])
        lines.append('')
        lines.append(' 거리 [mm]   (X = 단차 판정 셀,  ---- = 무응답)')
        for r in range(rows):
            cells = []
            for c in range(cols):
                i = r * cols + c
                v = grid[i] if i < len(grid) else None
                txt = '----' if v is None else '{:4d}'.format(int(v))
                cells.append(('X' if (r, c) in cliff_cells else ' ') + txt)
            lines.append('  R{} {}'.format(r, ' '.join(cells)))
    else:
        lines.append('')
        lines.append(' (frame 토픽 수신 없음 — detect.frame_hz 가 0인지 확인)')

    # ---- 이벤트 ----
    lines.append('')
    if ev:
        lines.append(' 단차   : {}  zone={}  cells={}  range={}m  depth={}m'.format(
            'YES' if ev.get('cliff') else 'no ', ev.get('zone'),
            ev.get('n_cells'), ev.get('range_m'), ev.get('depth_m')))
        lines.append(' 투표   : {}/{}   안전한 쪽={}'.format(
            ev.get('votes'), ev.get('window'), ev.get('free_side')))
    else:
        lines.append(' 단차   : (수신 없음)')

    # ---- 지시 ----
    if cmd:
        age = now - STATE['t']['command']
        lines.append(' 지시   : {:<14} ({})  {:.1f}s 전'.format(
            cmd.get('action'), cmd.get('reason'), age))
    else:
        lines.append(' 지시   : (수신 없음)')
    lines.append('=' * 62)

    sys.stdout.write('\033[2J\033[H' + '\n'.join(lines) + '\n')
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    m = cfg['mqtt']
    topics = m['topics']
    rev = {v: k for k, v in topics.items()}

    def on_connect(client, userdata, flags, rc, *a):
        client.subscribe([(t, 0) for t in topics.values()])

    def on_message(client, userdata, msg):
        key = rev.get(msg.topic)
        if not key:
            return
        try:
            STATE[key] = json.loads(msg.payload.decode('utf-8'))
        except Exception:
            return
        if key in STATE['t']:
            STATE['t'][key] = time.time()

    c = _make_client('cliff-monitor')
    user = m.get('username') or ''
    if user:
        c.username_pw_set(user, m.get('password') or '')
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(m['host'], int(m['port']), int(m.get('keepalive', 15)))
    c.loop_start()

    try:
        while True:
            render(topics)
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        c.loop_stop()
        c.disconnect()


if __name__ == '__main__':
    main()
