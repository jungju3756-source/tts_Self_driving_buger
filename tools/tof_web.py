#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tof_web.py  —  ToF 8x8 실시간 시각화 (웹 브라우저)

MQTT 로 흘러다니는 프레임/이벤트/지시를 받아 브라우저에 그린다.
파이썬 표준 라이브러리 + paho-mqtt 만 쓴다 (Flask 등 불필요).

    라파1(브로커)에서 실행:
        python3 ~/cliff_mqtt/tools/tof_web.py

    윈도우 PC / 폰 브라우저에서 열기:
        http://192.168.0.37:8080

보이는 것
    * 8x8 격자 — 칸마다 측정거리 [mm] 숫자, 거리에 따른 색
      (빨강 = 가까움, 청록 = 멂, 검정 = 무응답)
    * 노란 테두리 = ROI (판정에 실제로 쓰는 영역)
    * 흰 테두리 + 굵은 글씨 = 낭떠러지로 판정된 칸
    * 상단 배너 = 라파1로 나가는 안전 지시 (CLEAR / STOP / DETOUR_*)
    * ROI 최소거리와 그 추이 그래프

옵션
    --port 8080         수신 포트
    --record out.jsonl  프레임을 파일로 남긴다 (나중에 재생/보고용)
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'config', 'cliff_config.yaml')


# =============================================================================
#  공유 상태 — MQTT 스레드가 쓰고, SSE 스레드들이 읽는다
# =============================================================================
class Hub:

    def __init__(self, record_path=None):
        self.lock = threading.Lock()
        self.subs = []                 # 접속한 브라우저마다 큐 하나
        self.frame = None
        self.event = None
        self.command = None
        self.status = None
        self.last_frame_t = 0.0
        self.fps = 0.0
        self._t_hist = []
        self.rec = open(record_path, 'a', encoding='utf-8') if record_path else None

    def subscribe(self):
        q = queue.Queue(maxsize=4)
        with self.lock:
            self.subs.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subs:
                self.subs.remove(q)

    def _fanout(self, msg):
        data = json.dumps(msg, allow_nan=False)
        with self.lock:
            targets = list(self.subs)
        for q in targets:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass               # 느린 브라우저는 프레임을 건너뛴다

    # ---- MQTT 콜백에서 호출 ----
    def on_frame(self, payload):
        now = time.time()
        self._t_hist.append(now)
        self._t_hist = [t for t in self._t_hist if now - t < 3.0]
        self.fps = (len(self._t_hist) - 1) / max(1e-6, now - self._t_hist[0]) \
            if len(self._t_hist) > 1 else 0.0
        self.frame = payload
        self.last_frame_t = now
        if self.rec:
            self.rec.write(json.dumps({'t': round(now, 3), 'frame': payload}) + '\n')
            self.rec.flush()
        self._fanout(self.snapshot())

    def on_event(self, payload):
        self.event = payload

    def on_command(self, payload):
        self.command = payload

    def on_status(self, payload):
        self.status = payload

    def snapshot(self):
        return {
            'frame': self.frame,
            'event': self.event,
            'command': self.command,
            'status': self.status,
            'fps': round(self.fps, 1),
            'age': round(time.time() - self.last_frame_t, 2) if self.last_frame_t else None,
            'server_ts': round(time.time(), 3),
        }


# =============================================================================
#  MQTT
# =============================================================================
def make_client(client_id):
    """paho-mqtt 1.x / 2.x 양쪽에서 동작하는 클라이언트 생성."""
    try:
        from paho.mqtt.client import CallbackAPIVersion
        return mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        return mqtt.Client(client_id=client_id)


def start_mqtt(hub, mcfg):
    topics = mcfg['topics']
    cli = make_client('tof-web-{}'.format(os.getpid()))
    if mcfg.get('username'):
        cli.username_pw_set(mcfg['username'], mcfg.get('password', ''))

    routes = {
        topics['frame']: hub.on_frame,
        topics['cliff']: hub.on_event,
        topics['command']: hub.on_command,
        topics['status']: hub.on_status,
    }

    def on_connect(client, userdata, flags, rc, *a):
        print('[mqtt] {} rc={}'.format('연결됨' if rc == 0 else '연결 실패', rc), flush=True)
        for t in routes:
            client.subscribe(t, qos=0)

    def on_message(client, userdata, msg):
        fn = routes.get(msg.topic)
        if not fn:
            return
        try:
            fn(json.loads(msg.payload.decode('utf-8')))
        except Exception as exc:
            print('[mqtt] {} 파싱 실패: {}'.format(msg.topic, exc), flush=True)

    cli.on_connect = on_connect
    cli.on_message = on_message
    cli.connect_async(mcfg['host'], int(mcfg['port']), int(mcfg.get('keepalive', 15)))
    cli.loop_start()
    return cli


# =============================================================================
#  HTTP
# =============================================================================
class Handler(BaseHTTPRequestHandler):

    hub = None
    page = b''

    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass                       # 접속 로그로 콘솔을 더럽히지 않는다

    def do_GET(self):
        if self.path.startswith('/stream'):
            return self._stream()
        if self.path == '/' or self.path.startswith('/index'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(self.page)))
            self.end_headers()
            self.wfile.write(self.page)
            return
        self.send_error(404)

    def _stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        q = self.hub.subscribe()
        try:
            # 접속하자마자 현재 상태를 한 번 보내 화면이 비어 있지 않게 한다
            self._send(json.dumps(self.hub.snapshot(), allow_nan=False))
            while True:
                try:
                    data = q.get(timeout=2.0)
                except queue.Empty:
                    # 프레임이 끊겨도 연결은 유지하고 '몇 초째 끊김'을 갱신한다
                    data = json.dumps(self.hub.snapshot(), allow_nan=False)
                self._send(data)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.hub.unsubscribe(q)

    def _send(self, data):
        self.wfile.write(b'data: ' + data.encode('utf-8') + b'\n\n')
        self.wfile.flush()


# =============================================================================
#  페이지
# =============================================================================
PAGE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ToF 낭떠러지 감지</title>
<style>
  :root { --bg:#14161a; --panel:#1d2026; --line:#333842; --txt:#e8eaee; --dim:#8a919c; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--txt); font:14px/1.5
         ui-monospace,"DejaVu Sans Mono",Menlo,Consolas,monospace; }
  header { padding:12px 16px; border-bottom:1px solid var(--line);
           display:flex; align-items:center; gap:14px; flex-wrap:wrap; }
  h1 { font-size:15px; margin:0; font-weight:600; letter-spacing:.02em; }
  .pill { padding:3px 10px; border-radius:999px; font-size:12px;
          border:1px solid var(--line); color:var(--dim); }
  .pill.on  { color:#0c1d12; background:#54d98c; border-color:#54d98c; }
  .pill.off { color:#2a0d0d; background:#ff6b6b; border-color:#ff6b6b; }

  #banner { margin:14px 16px 0; padding:12px 16px; border-radius:8px;
            font-size:20px; font-weight:700; letter-spacing:.04em;
            display:flex; justify-content:space-between; align-items:center;
            background:#20242b; border:1px solid var(--line); }
  #banner .why { font-size:13px; font-weight:400; opacity:.85; }
  #banner.clear  { background:#123a22; border-color:#2f7d4f; color:#7ef0a8; }
  #banner.stop   { background:#4a1414; border-color:#c0392b; color:#ff9b93; }
  #banner.detour { background:#4a3a10; border-color:#c9a227; color:#ffdf7e; }
  #banner.bad    { background:#3a2044; border-color:#8e44ad; color:#e0a3ff; }

  main { display:flex; gap:16px; padding:14px 16px 24px; flex-wrap:wrap;
         align-items:flex-start; }
  .card { background:var(--panel); border:1px solid var(--line);
          border-radius:8px; padding:12px; }
  .card h2 { margin:0 0 10px; font-size:12px; color:var(--dim);
             font-weight:600; letter-spacing:.08em; text-transform:uppercase; }

  /* ---- 격자 ---- */
  #gridwrap { position:relative; }
  #grid { display:grid; grid-template-columns:repeat(8, 62px);
          grid-auto-rows:52px; gap:2px; }
  .cell { display:flex; align-items:center; justify-content:center;
          font-size:15px; font-variant-numeric:tabular-nums; color:#000;
          background:#000; border-radius:2px; position:relative; }
  .cell.void { color:#4a4a4a; font-size:13px; }
  .cell.cliff { box-shadow:inset 0 0 0 3px #fff; font-weight:800; }
  .cell.blind { box-shadow:inset 0 0 0 3px #fff; color:#fff;
                background:repeating-linear-gradient(45deg,#000 0 6px,#222 6px 12px); }
  #roibox { position:absolute; border:2px solid #ffd400; border-radius:3px;
            pointer-events:none; display:none; }
  .axis { color:var(--dim); font-size:11px; display:flex; }
  .axis span { width:62px; text-align:center; }
  .rowlab { position:absolute; left:-26px; color:var(--dim); font-size:11px; }

  table.kv { border-collapse:collapse; font-size:13px; min-width:230px; }
  table.kv td { padding:3px 8px 3px 0; vertical-align:top; }
  table.kv td:first-child { color:var(--dim); white-space:nowrap; }
  .big { font-size:26px; font-weight:700; font-variant-numeric:tabular-nums; }

  #spark { display:block; background:#101317; border-radius:4px; }
  .legend { display:flex; align-items:center; gap:8px; font-size:12px;
            color:var(--dim); margin-top:10px; }
  .ramp { height:12px; width:200px; border-radius:3px;
          background:linear-gradient(90deg,
            hsl(0,95%,52%), hsl(48,95%,52%), hsl(120,95%,52%), hsl(195,95%,52%)); }
  label { font-size:12px; color:var(--dim); }
  input[type=number] { width:64px; background:#0e1115; color:var(--txt);
        border:1px solid var(--line); border-radius:4px; padding:2px 6px;
        font:inherit; font-size:12px; }
</style>
</head>
<body>

<header>
  <h1>ToF 낭떠러지 감지</h1>
  <span class="pill" id="p-sensor">센서 —</span>
  <span class="pill" id="p-link">링크 —</span>
  <span class="pill" id="p-fps">— fps</span>
  <span style="flex:1"></span>
  <label>색 범위 <input type="number" id="lo" value="100" step="50"> ~
    <input type="number" id="hi" value="2000" step="50"> mm</label>
</header>

<div id="banner"><span id="b-act">대기 중</span><span class="why" id="b-why"></span></div>

<main>
  <div class="card">
    <h2>8 × 8 측정거리 [mm]</h2>
    <div id="gridwrap" style="margin-left:26px">
      <div id="grid"></div>
      <div id="roibox"></div>
    </div>
    <div class="axis" id="collab" style="margin-left:26px"></div>
    <div class="legend">
      <span>가까움</span><div class="ramp"></div><span>멂</span>
      <span style="margin-left:14px">
        <b style="color:#ffd400">▭</b> ROI
        &nbsp;<b style="color:#fff">▭</b> 낭떠러지 판정
        &nbsp;<b style="color:#666">■</b> 무응답</span>
    </div>
  </div>

  <div class="card">
    <h2>판정</h2>
    <div>ROI 최소거리</div>
    <div class="big" id="roimin">—</div>
    <canvas id="spark" width="260" height="60"></canvas>
    <table class="kv" style="margin-top:10px">
      <tr><td>구역</td><td id="k-zone">—</td></tr>
      <tr><td>낙차 셀</td><td id="k-cells">—</td></tr>
      <tr><td>가까운 거리</td><td id="k-range">—</td></tr>
      <tr><td>비어있는 쪽</td><td id="k-free">—</td></tr>
      <tr><td>투표</td><td id="k-votes">—</td></tr>
      <tr><td>지시</td><td id="k-act">—</td></tr>
      <tr><td>사유</td><td id="k-why">—</td></tr>
    </table>
  </div>
</main>

<script>
const N = 8;
const gridEl = document.getElementById('grid');
const cells = [];
for (let i = 0; i < N * N; i++) {
  const d = document.createElement('div');
  d.className = 'cell';
  gridEl.appendChild(d);
  cells.push(d);
}
const colEl = document.getElementById('collab');
const hist = [];

function colorFor(mm, lo, hi) {
  if (mm === null || mm === undefined) return '#000';
  let t = (mm - lo) / Math.max(1, hi - lo);
  t = Math.max(0, Math.min(1, t));
  return 'hsl(' + (t * 195).toFixed(0) + ', 95%, 52%)';
}

// 센서 장착 방향에 맞춰 '가까운 쪽이 아래' 로 보이게 행/열 순서를 정한다.
function order(f) {
  const rows = [], cols = [];
  const rowTop = f.row0_is_top !== false;   // R0 가 영상 위쪽(먼 쪽)인가
  const colLeft = f.col0_is_left !== false;
  for (let r = 0; r < N; r++) rows.push(rowTop ? r : N - 1 - r);
  for (let c = 0; c < N; c++) cols.push(colLeft ? c : N - 1 - c);
  return { rows, cols };
}

function drawGrid(f) {
  const lo = +document.getElementById('lo').value;
  const hi = +document.getElementById('hi').value;
  const { rows, cols } = order(f);
  const g = f.grid_mm, drop = f.drop || [], blind = f.blind || [], roi = f.roi || [];
  let rmin = null, rmax = null, cmin = null, cmax = null;

  for (let i = 0; i < N; i++) {
    for (let j = 0; j < N; j++) {
      const src = rows[i] * N + cols[j];
      const el = cells[i * N + j];
      const v = g[src];
      el.className = 'cell' + (v === null ? ' void' : '')
        + (drop[src] ? ' cliff' : '') + (blind[src] ? ' blind' : '');
      el.style.background = blind[src] ? '' : colorFor(v, lo, hi);
      el.textContent = (v === null ? '----' : v);
      if (roi[src]) {
        if (rmin === null || i < rmin) rmin = i;
        if (rmax === null || i > rmax) rmax = i;
        if (cmin === null || j < cmin) cmin = j;
        if (cmax === null || j > cmax) cmax = j;
      }
    }
  }

  // ROI 를 사진처럼 하나의 노란 사각형으로 두른다
  const box = document.getElementById('roibox');
  if (rmin === null) { box.style.display = 'none'; }
  else {
    const W = 62, H = 52, GAP = 2;
    box.style.display = 'block';
    box.style.left = (cmin * (W + GAP) - 2) + 'px';
    box.style.top = (rmin * (H + GAP) - 2) + 'px';
    box.style.width = ((cmax - cmin + 1) * (W + GAP) - GAP + 4) + 'px';
    box.style.height = ((rmax - rmin + 1) * (H + GAP) - GAP + 4) + 'px';
  }

  if (!colEl.dataset.done) {
    colEl.innerHTML = cols.map(c => '<span>C' + c + '</span>').join('');
    colEl.dataset.done = '1';
  }
}

function drawSpark() {
  const cv = document.getElementById('spark');
  const x = cv.getContext('2d');
  x.clearRect(0, 0, cv.width, cv.height);
  if (hist.length < 2) return;
  const vals = hist.filter(v => v !== null);
  if (!vals.length) return;
  const lo = Math.min(...vals), hi = Math.max(...vals, lo + 1);
  x.beginPath();
  hist.forEach((v, i) => {
    if (v === null) return;
    const px = i / (hist.length - 1) * cv.width;
    const py = cv.height - 4 - (v - lo) / (hi - lo) * (cv.height - 8);
    i === 0 ? x.moveTo(px, py) : x.lineTo(px, py);
  });
  x.strokeStyle = '#54d98c'; x.lineWidth = 2; x.stroke();
}

const BANNER = {
  CLEAR: ['clear', '정상 주행'],
  STOP: ['stop', '정지 — 전진 차단'],
  DETOUR_LEFT: ['detour', '좌측으로 우회'],
  DETOUR_RIGHT: ['detour', '우측으로 우회'],
  CORRECT_LEFT: ['detour', '좌측 보정'],
  CORRECT_RIGHT: ['detour', '우측 보정'],
  DEGRADED: ['bad', '센서 이상 — 감속 주행'],
  FAULT: ['bad', '센서 두절 — 정지'],
};

function set(id, v) { document.getElementById(id).textContent = v; }

function render(s) {
  const f = s.frame, ev = s.event, cm = s.command;

  const stale = s.age === null || s.age > 1.5;
  const p = document.getElementById('p-sensor');
  const on = s.status && s.status.state === 'online' && !stale;
  p.className = 'pill ' + (on ? 'on' : 'off');
  p.textContent = '센서 ' + (s.status ? s.status.state : '—');
  const pl = document.getElementById('p-link');
  pl.className = 'pill ' + (stale ? 'off' : 'on');
  pl.textContent = stale ? ('프레임 끊김 ' + (s.age === null ? '' : s.age + 's')) : '프레임 수신 중';
  set('p-fps', (s.fps || 0).toFixed(1) + ' fps');

  if (f) {
    drawGrid(f);
    const rm = f.roi_min_mm;
    set('roimin', rm === null || rm === undefined ? '—'
        : rm + ' mm  (' + (rm / 1000).toFixed(2) + ' m)');
    hist.push(rm === undefined ? null : rm);
    while (hist.length > 150) hist.shift();
    drawSpark();
  }

  if (ev) {
    set('k-zone', ev.zone || 'NONE');
    set('k-cells', ev.n_cells != null ? ev.n_cells + ' 칸' : '—');
    set('k-range', ev.range_m != null ? ev.range_m.toFixed(2) + ' m' : '—');
    set('k-free', ev.free_side || '—');
    set('k-votes', (ev.votes != null ? ev.votes : '—') + ' / ' + (ev.window || '—'));
  }

  const act = cm ? cm.action : (f ? (f.cliff ? 'STOP' : 'CLEAR') : null);
  const b = document.getElementById('banner');
  if (act) {
    const [cls, txt] = BANNER[act] || ['', act];
    b.className = cls;
    set('b-act', act);
    set('b-why', txt);
    set('k-act', act);
    set('k-why', cm ? (cm.reason || '—') : '—');
  }
}

const es = new EventSource('/stream');
es.onmessage = e => { try { render(JSON.parse(e.data)); } catch (_) {} };
es.onerror = () => {
  const pl = document.getElementById('p-link');
  pl.className = 'pill off';
  pl.textContent = '서버 연결 끊김';
};
</script>
</body>
</html>
"""


# =============================================================================
def main():
    ap = argparse.ArgumentParser(description='ToF 8x8 웹 시각화')
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--bind', default='0.0.0.0')
    ap.add_argument('--record', metavar='FILE', help='프레임을 JSONL 로 기록')
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    hub = Hub(args.record)
    start_mqtt(hub, cfg['mqtt'])

    Handler.hub = hub
    Handler.page = PAGE.encode('utf-8')
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    srv.daemon_threads = True

    print('[web] http://{}:{}  (브라우저에서 열기)'
          .format(cfg['mqtt']['host'], args.port), flush=True)
    if args.record:
        print('[web] 기록: {}'.format(args.record), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n[web] 종료')
    return 0


if __name__ == '__main__':
    sys.exit(main())
