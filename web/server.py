#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
server.py  —  로봇 조작 웹 페이지 (지도 · 장소로 보내기 · 벽 그리기 · 음성 명령)

    python3 ~/tts_Self_driving_buger/web/server.py
    → 브라우저:  https://192.168.0.57:8443   (자체 서명 인증서 — 처음 한 번 '고급 → 계속')

    브라우저 ──WebSocket /ws──► server.py ──rclpy──► Nav2 / place_manager / wall_manager
       │  마이크 PCM 16k (Gemini 모드)        └──wss──► Gemini Live (API 키는 서버에만)
       └◄─ 지도·로봇 위치·경로·배터리 (실시간)

음성 두 가지:
  - 'browser' : 브라우저(Chrome) 음성 인식 → 문장 → intent.py 규칙 해석.  API 키 없이 바로 됨
  - 'gemini'  : Gemini Live 한 세션으로 듣기·판단·말하기 (캡스톤 A안). 키: ~/.config/robot_web/gemini_api_key
둘 다 '멈춰/정지/그만' 은 LLM 을 거치지 않고 바로 정지한다.

HTTPS 가 필요한 이유: 브라우저는 https(또는 localhost)에서만 마이크를 열어 준다.
"""

import asyncio
import io
import json
import logging
import math
import os
import queue
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
import tornado.ioloop
import tornado.web
import tornado.websocket
import yaml
from PIL import Image
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import intent
from gemini_live import GeminiLive

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)
CONF_DIR = os.path.expanduser('~/.config/robot_web')
KEY_FILE = os.path.join(CONF_DIR, 'gemini_api_key')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(message)s', datefmt='%H:%M:%S')
log = logging.getLogger('web')


def load_conf():
    with open(os.path.join(ROOT, 'config', 'web.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


def read_key():
    """키 파일에서 '#' 주석·빈 줄을 뺀 첫 줄. 공백이 있거나 너무 짧으면 None.
    (키 형식은 AIza…(39자) 만 있는 게 아니다 — 2026-10 발급 키는 AQ…(53자). 실제 확인은 Google 이 한다)"""
    if not os.path.exists(KEY_FILE):
        return None
    with open(KEY_FILE, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                return line if len(line) >= 20 and ' ' not in line else None
    return None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# =============================================================================
# ROS 쪽 (별도 스레드에서 spin) — 웹 쪽과는 emit() / submit() 으로만 주고받는다
# =============================================================================
class RobotNode(Node):

    def __init__(self, emit):
        super().__init__('web_server')
        self.emit = emit                     # dict → 웹 쪽 브로드캐스트 (스레드 안전)
        self.jobs = queue.Queue()            # 웹 → ROS 스레드 작업
        self.places = {}
        self.map_png = None
        self.map_info = None
        self.pose = None
        self.goal_handle = None
        self.nav = {'state': 'idle', 'target': None, 'remaining': None}

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.create_subscription(OccupancyGrid, '/map', self._map, LATCHED)
        self.create_subscription(String, '/places/list', self._places, LATCHED)
        self.create_subscription(String, '/walls/list', lambda m: self.emit(
            {'type': 'walls', **json.loads(m.data)}), LATCHED)
        self.create_subscription(String, '/places/event', lambda m: self.emit(
            {'type': 'place_event', **json.loads(m.data)}), 10)
        self.create_subscription(String, '/walls/event', lambda m: self.emit(
            {'type': 'wall_event', **json.loads(m.data)}), 10)
        self.create_subscription(String, '/voice/event', lambda m: self.emit(       # rapa1/voice_ptt.py (패드 B)
            {'type': 'pad_voice', **json.loads(m.data)}), 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(BatteryState, '/battery_state', self._battery, 10)

        self.pub_places = self.create_publisher(String, '/places/cmd', 10)
        self.pub_walls = self.create_publisher(String, '/walls/cmd', 10)
        self.pub_click = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.2, self._tick_pose)
        self.create_timer(0.05, self._run_jobs)
        self._last_batt = 0.0

    # ---------- 웹 → ROS ----------
    def submit(self, fn, *args):
        self.jobs.put((fn, args))

    def _run_jobs(self):
        while not self.jobs.empty():
            fn, args = self.jobs.get_nowait()
            try:
                fn(*args)
            except Exception as e:
                log.exception('job failed')
                self.emit({'type': 'log', 'level': 'error', 'text': f'{fn.__name__}: {e}'})

    # ---------- 구독 ----------
    def _map(self, m):
        w, h = m.info.width, m.info.height
        a = np.array(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, np.uint8)            # 미탐색 = 회색
        img[a == 0] = 254
        img[a >= 65] = 0
        img[(a > 0) & (a < 65)] = 180
        buf = io.BytesIO()
        Image.fromarray(np.flipud(img)).save(buf, 'PNG')  # ROS 행0=아래 → 이미지 행0=위
        self.map_png = buf.getvalue()
        self.map_info = {'width': w, 'height': h, 'resolution': m.info.resolution,
                         'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y,
                         'version': time.time()}
        self.emit({'type': 'map', **self.map_info})

    def _places(self, m):
        self.places = json.loads(m.data)
        self.emit({'type': 'places', 'places': self.places})

    def _plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        step = max(1, len(pts) // 150)
        self.emit({'type': 'plan', 'points': [[round(x, 3), round(y, 3)] for x, y in pts[::step]]})

    def _battery(self, m):
        if time.time() - self._last_batt < 2.0:
            return
        self._last_batt = time.time()
        pct = m.percentage if m.percentage <= 1.5 else m.percentage / 100.0
        self.emit({'type': 'battery', 'voltage': round(m.voltage, 2),
                   'percent': round(min(max(pct, 0.0), 1.0) * 100)})

    def _tick_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        tr = t.transform
        self.pose = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        self.emit({'type': 'pose', 'x': round(self.pose[0], 3), 'y': round(self.pose[1], 3),
                   'yaw': round(self.pose[2], 3)})

    # ---------- 동작 ----------
    def _set_nav(self, **kw):
        self.nav.update(kw)
        self.emit({'type': 'nav', **self.nav})

    def goto(self, name):
        p = self.places.get(name)
        if p is None:
            self.emit({'type': 'log', 'level': 'error', 'text': f"'{name}' 없음"})
            return
        if not self.nav_ac.server_is_ready():
            self._set_nav(state='failed', target=name, remaining=None, reason='Nav2 가 실행 중이 아님')
            return
        if p.get('yaw') is not None:
            yaw = p['yaw']
        elif self.pose:
            yaw = math.atan2(p['y'] - self.pose[1], p['x'] - self.pose[0])
        else:
            yaw = 0.0
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = p['x'], p['y']
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._set_nav(state='sending', target=name, remaining=None, reason=None, started=time.time())
        fut = self.nav_ac.send_goal_async(g, feedback_callback=self._feedback)
        fut.add_done_callback(lambda f: self._accepted(f, name))

    def _accepted(self, fut, name):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._set_nav(state='failed', reason='목표 거부됨')
            return
        self.goal_handle = gh
        self._set_nav(state='moving', target=name)
        gh.get_result_async().add_done_callback(lambda f: self._done(f, gh, name))

    def _feedback(self, fb):
        r = fb.feedback.distance_remaining
        if self.nav['state'] == 'moving' and r > 0:
            self.nav['remaining'] = round(r, 2)
            self.emit({'type': 'nav', **self.nav})

    def _done(self, fut, gh, name):
        if gh is not self.goal_handle:          # 이미 새 목표로 바뀜 — 옛 결과는 무시
            return
        status = fut.result().status
        err = None
        if self.pose and name in self.places:
            p = self.places[name]
            err = round(math.hypot(self.pose[0] - p['x'], self.pose[1] - p['y']), 3)
        state = {GoalStatus.STATUS_SUCCEEDED: 'arrived',
                 GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, 'failed')
        self.goal_handle = None
        self._set_nav(state=state, target=name, remaining=None, error_m=err,
                      took=round(time.time() - self.nav.get('started', time.time()), 1))

    def stop(self, why='정지'):
        # 1) 목표 취소  2) 바퀴 즉시 0 (Nav2 가 취소를 처리하는 사이에도 서도록)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        for _ in range(3):
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(m)
        self._set_nav(state='canceled' if self.goal_handle else self.nav['state'], reason=why)
        self.emit({'type': 'log', 'level': 'warn', 'text': f'■ {why}'})

    def place_cmd(self, text):
        self.pub_places.publish(String(data=text))

    def wall_cmd(self, obj):
        self.pub_walls.publish(String(data=json.dumps(obj)))

    def place_at(self, x, y, name):
        m = PointStamped()
        m.header.frame_id = 'map'
        m.point.x, m.point.y = float(x), float(y)
        self.pub_click.publish(m)
        # place_manager 가 클릭을 받은 뒤 save 하도록 조금 늦춘다
        threading.Timer(0.3, lambda: self.submit(self.place_cmd, f'save {name}')).start()

    def set_initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.1 ** 2
        cov[35] = math.radians(10) ** 2
        m.pose.covariance = cov
        self.pub_init.publish(m)
        self.emit({'type': 'log', 'level': 'info', 'text': '로봇 위치 지정함'})

    def where_text(self):
        if not self.pose:
            return '지금 위치를 모르겠어요.'
        if not self.places:
            return '등록된 장소가 없어요.'
        name, p = min(self.places.items(),
                      key=lambda kv: math.hypot(kv[1]['x'] - self.pose[0], kv[1]['y'] - self.pose[1]))
        d = math.hypot(p['x'] - self.pose[0], p['y'] - self.pose[1])
        if d < 0.4:
            return f'지금 {name}에 있어요.'
        return f'{name}에서 {d:.1f}미터 떨어진 곳에 있어요.'


# =============================================================================
# 웹 쪽
# =============================================================================
class Hub:
    """연결된 브라우저들 + 최신 상태 (새로 들어온 브라우저에 바로 보내 줌)"""

    def __init__(self):
        self.clients = set()
        self.snapshot = {}
        self.loop = None

    def emit_threadsafe(self, msg):
        self.loop.add_callback(self.broadcast, msg)

    def broadcast(self, msg):
        if msg['type'] in ('map', 'places', 'walls', 'pose', 'battery', 'nav', 'plan'):
            self.snapshot[msg['type']] = msg
        if msg['type'] == 'nav':
            for c in list(self.clients):
                c.on_nav(msg)
        text = json.dumps(msg, ensure_ascii=False)
        for c in list(self.clients):
            try:
                c.write_message(text)
            except tornado.websocket.WebSocketClosedError:
                self.clients.discard(c)


SYSTEM_TEXT = """너는 연구실 안내 로봇 '터틀이'의 음성이다. 한국어로 짧고 친절하게 한두 문장으로 말한다.

등록된 장소: {places}

규칙:
1. 사용자가 장소로 가 달라고 하면 navigate_to 를 호출한다. 장소 이름만 말해도("3번", "정수기") 이동 요청으로 본다.
   '~ 어디야' 처럼 위치만 묻는 질문이면 바로 출발하지 말고 안내해 줄지 물어본다.
2. 등록되지 않은 장소는 절대 지어내지 마라. 비슷한 게 여러 개면 어느 것인지 되묻는다.
3. navigate_to 결과가 ok 면 "○○로 갈게요" 처럼 출발 안내를 한 문장만 말한다.
   그 다음부터는 도착하거나 사용자가 멈추라고/목적지를 바꾸라고 할 때까지 아무 말도 하지 마라.
   주변 대화나 잡음에는 반응하지 마라.
4. "멈춰", "정지", "그만" 이면 stop_navigation 을 호출한다.
5. 지금 어디냐고 물으면 get_status 를 호출해 답한다. 갈 수 있는 곳을 물으면 list_places.
6. "[시스템 안내]" 로 시작하는 말은 사용자가 아니라 로봇 시스템의 알림이다. 그 내용을 짧게 말로 전한다.
"""

TOOLS = [
    {'name': 'navigate_to', 'description': '등록된 장소로 로봇을 이동시킨다.',
     'parameters': {'type': 'OBJECT', 'properties': {
         'destination': {'type': 'STRING', 'description': '등록된 장소 이름 (예: 1번, 정수기)'}},
         'required': ['destination']}},
    {'name': 'stop_navigation', 'description': '이동을 즉시 멈춘다.',
     'parameters': {'type': 'OBJECT', 'properties': {}}},
    {'name': 'list_places', 'description': '갈 수 있는 장소 목록을 돌려준다.',
     'parameters': {'type': 'OBJECT', 'properties': {}}},
    {'name': 'get_status', 'description': '로봇의 현재 위치·이동 상태를 돌려준다.',
     'parameters': {'type': 'OBJECT', 'properties': {}}},
]


class VoiceSession:
    """브라우저 한 개 ↔ Gemini Live 세션 하나. 안내 멘트가 다 재생된 뒤 출발 (RobotNav dispatchAfterPlayback)."""

    SAFETY_SEC = 8.0

    def __init__(self, ws, node, conf):
        self.ws, self.node, self.conf = ws, node, conf
        self.live = None
        self.pending_go = None
        self.turn_done = False          # 안내 멘트 턴이 끝났는지
        self.spoke_after_tool = False   # navigate_to 이후 음성이 나왔는지 (턴 종료가 도구 턴·음성 턴 두 번 옴)
        self.safety = None
        self.nav_silent = False
        self.t_speech_end = None
        self.t_first_audio = None

    async def start(self):
        key = read_key()
        if not key:
            raise RuntimeError('Gemini API 키가 없습니다 (~/.config/robot_web/gemini_api_key 에 AIza… 한 줄)')
        places = ', '.join(f"{n}" + (f"(별칭: {', '.join(p['aliases'])})" if p.get('aliases') else '')
                           for n, p in self.node.places.items()) or '(없음)'
        g = self.conf['gemini']
        self.live = GeminiLive(key, g['model'], g['voice'], SYSTEM_TEXT.format(places=places), TOOLS,
                               on_audio=self._audio, on_event=self._event, on_tool=self._tool,
                               silence_ms=g.get('silence_ms', 600))
        await self.live.start()

    def close(self):
        self._cancel_safety()
        if self.live:
            self.live.close()
            self.live = None

    # --- Gemini → 브라우저 ---
    def _audio(self, pcm):
        if self.nav_silent:                   # 이동 중엔 모델이 뭐라 하든 내보내지 않음
            return
        if self.pending_go:
            self.spoke_after_tool = True
        if self.t_first_audio is None:
            self.t_first_audio = time.time()
            if self.t_speech_end:
                self.ws.send_json({'type': 'voice', 'event': 'latency',
                                   'sec': round(self.t_first_audio - self.t_speech_end, 2)})
        try:
            self.ws.write_message(pcm, binary=True)
        except tornado.websocket.WebSocketClosedError:
            pass

    def _event(self, e):
        t = e['type']
        if t == 'input_text':
            if intent.is_stop(e['text']):     # 비상 정지 — LLM 응답을 기다리지 않는다
                self.pending_go = None
                self._cancel_safety()
                self.node.submit(self.node.stop, '음성 정지')
            self.t_speech_end = time.time()   # 마지막 인식 조각 = 대략 발화 끝
            self.t_first_audio = None
        elif t == 'turn_complete':
            if self.pending_go and self.spoke_after_tool:   # 도구 턴의 종료는 건너뛴다
                self.turn_done = True
        elif t == 'interrupted':
            self.ws.send_json({'type': 'voice', 'event': 'interrupted'})
        if t in ('input_text', 'output_text') and self.nav_silent and t == 'output_text':
            return
        self.ws.send_json({'type': 'voice', 'event': t, **{k: v for k, v in e.items() if k != 'type'}})

    def _tool(self, name, args):
        n = self.node
        if name == 'navigate_to':
            dest = intent.resolve_place(args.get('destination', ''), n.places)
            if dest is None:
                return {'ok': False, 'error': '등록되지 않은 장소', 'places': list(n.places)}
            self.pending_go = dest
            self.turn_done = False
            self.spoke_after_tool = False
            self._arm_safety()
            return {'ok': True, 'destination': dest, 'note': '출발 안내를 한 문장 말하면 그 뒤에 출발한다'}
        if name == 'stop_navigation':
            self.pending_go = None
            self._cancel_safety()
            n.submit(n.stop, '음성 정지')
            return {'ok': True}
        if name == 'list_places':
            return {'places': list(n.places)}
        if name == 'get_status':
            return {'where': n.where_text(), 'nav': n.nav['state'], 'target': n.nav['target']}
        return {'ok': False, 'error': f'모르는 도구 {name}'}

    # --- 안내 멘트 재생이 끝나면 출발 ---
    def playback_idle(self):
        if self.pending_go and self.turn_done:
            self._dispatch('안내 끝')

    def _arm_safety(self):
        self._cancel_safety()
        self.safety = tornado.ioloop.IOLoop.current().call_later(
            self.SAFETY_SEC, lambda: self._dispatch('8초 안전망'))

    def _cancel_safety(self):
        if self.safety is not None:
            tornado.ioloop.IOLoop.current().remove_timeout(self.safety)
            self.safety = None

    def _dispatch(self, why):
        dest, self.pending_go = self.pending_go, None
        self._cancel_safety()
        if not dest:
            return
        self.nav_silent = True
        self.node.submit(self.node.goto, dest)
        self.ws.send_json({'type': 'voice', 'event': 'dispatch', 'destination': dest, 'why': why})

    def on_nav(self, msg):
        if msg['state'] not in ('arrived', 'failed', 'canceled') or not self.live:
            return
        if not self.nav_silent:
            return
        self.nav_silent = False
        text = {'arrived': f"{msg['target']}에 도착했습니다.",
                'failed': f"{msg['target']}(으)로 가지 못했습니다. {msg.get('reason') or ''}",
                'canceled': '이동을 멈췄습니다.'}[msg['state']]
        asyncio.ensure_future(self.live.send_text(
            f'[시스템 안내 · 최우선] 앞서 "이동 중에는 말하지 말라" 던 지시는 지금부터 무효다. '
            f'{text} 이 내용을 지금 짧게 말로 알려 줘.'))


class WS(tornado.websocket.WebSocketHandler):

    def initialize(self, hub, node, conf):
        self.hub, self.node, self.conf = hub, node, conf
        self.voice = None

    def check_origin(self, origin):
        return True

    def open(self):
        self.hub.clients.add(self)
        for msg in self.hub.snapshot.values():
            self.write_message(json.dumps(msg, ensure_ascii=False))
        self.send_json({'type': 'hello', 'gemini_key': read_key() is not None,
                        'model': self.conf['gemini']['model']})

    def on_close(self):
        self.hub.clients.discard(self)
        if self.voice:
            self.voice.close()

    def send_json(self, obj):
        try:
            self.write_message(json.dumps(obj, ensure_ascii=False))
        except tornado.websocket.WebSocketClosedError:
            pass

    def on_nav(self, msg):
        if self.voice:
            self.voice.on_nav(msg)

    async def on_message(self, message):
        if isinstance(message, bytes):                 # 마이크 PCM (Gemini 모드)
            if self.voice and self.voice.live:
                await self.voice.live.send_audio(message)
            return
        c = json.loads(message)
        cmd = c.get('cmd')
        n = self.node
        if cmd == 'goto':
            n.submit(n.goto, c['name'])
        elif cmd == 'stop':
            n.submit(n.stop, c.get('why', '정지 버튼'))
        elif cmd == 'wall_add':
            n.submit(n.wall_cmd, {'cmd': 'add', **{k: c[k] for k in ('x1', 'y1', 'x2', 'y2')}})
        elif cmd == 'wall_delete':
            n.submit(n.wall_cmd, {'cmd': 'delete', 'id': c['id']})
        elif cmd == 'wall_clear':
            n.submit(n.wall_cmd, {'cmd': 'clear'})
        elif cmd == 'place_add':
            n.submit(n.place_at, c['x'], c['y'], c['name'])
        elif cmd == 'place_cmd':
            n.submit(n.place_cmd, c['text'])
        elif cmd == 'initial_pose':
            n.submit(n.set_initial_pose, c['x'], c['y'], c['yaw'])
        elif cmd == 'say':                              # 브라우저 음성 인식 결과 (키 없이)
            self._say(c.get('text', ''))
        elif cmd == 'voice_start':
            await self._voice_start()
        elif cmd == 'voice_stop':
            if self.voice:
                self.voice.close()
                self.voice = None
            self.send_json({'type': 'voice', 'event': 'closed'})
        elif cmd == 'playback_idle':
            if self.voice:
                self.voice.playback_idle()

    def _say(self, text):
        n = self.node
        action, place, reply = intent.parse_command(text, n.places)
        if action == 'stop':
            n.submit(n.stop, '음성 정지')
        elif action == 'go':
            n.submit(n.goto, place)
        elif action == 'where':
            reply = n.where_text()
        self.send_json({'type': 'say_reply', 'heard': text, 'action': action, 'place': place, 'text': reply})

    async def _voice_start(self):
        if self.voice:
            self.voice.close()
        self.voice = VoiceSession(self, self.node, self.conf)
        try:
            await self.voice.start()
        except Exception as e:
            self.voice = None
            log.warning('Gemini 시작 실패: %s', e)
            self.send_json({'type': 'voice', 'event': 'error', 'text': f'Gemini 연결 실패: {e}'})


class MapHandler(tornado.web.RequestHandler):

    def initialize(self, node):
        self.node = node

    def get(self):
        if self.node.map_png is None:
            raise tornado.web.HTTPError(503, '지도 없음')
        self.set_header('Content-Type', 'image/png')
        self.set_header('Cache-Control', 'no-store')
        self.write(self.node.map_png)


class KeyHandler(tornado.web.RequestHandler):
    """페이지에서 Gemini 키 저장 (같은 네트워크 안에서만 쓰는 연구실용)"""

    def post(self):
        key = json.loads(self.request.body).get('key', '').strip()
        if len(key) < 20 or ' ' in key:
            raise tornado.web.HTTPError(400, '키 형식이 이상함')
        os.makedirs(CONF_DIR, exist_ok=True)
        with open(KEY_FILE, 'w', encoding='utf-8') as f:
            f.write(key)
        os.chmod(KEY_FILE, 0o600)
        self.write({'ok': True})


def ensure_cert():
    cert, keyf = os.path.join(CONF_DIR, 'cert.pem'), os.path.join(CONF_DIR, 'key.pem')
    if not os.path.exists(cert):
        os.makedirs(CONF_DIR, exist_ok=True)
        ips = subprocess.run(['hostname', '-I'], capture_output=True, text=True).stdout.split()
        san = ','.join([f'IP:{ip}' for ip in ips if ':' not in ip] + ['IP:127.0.0.1', 'DNS:localhost'])
        subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '3650',
                        '-keyout', keyf, '-out', cert, '-subj', '/CN=rapa1-robot',
                        '-addext', f'subjectAltName={san}'], check=True, capture_output=True)
        os.chmod(keyf, 0o600)
        log.info('자체 서명 인증서 생성: %s', san)
    return cert, keyf


def main():
    conf = load_conf()
    hub = Hub()
    rclpy.init()
    node = RobotNode(hub.emit_threadsafe)
    ex = MultiThreadedExecutor()
    ex.add_node(node)

    app = tornado.web.Application([
        (r'/', tornado.web.RedirectHandler, {'url': '/static/index.html'}),
        (r'/ws', WS, {'hub': hub, 'node': node, 'conf': conf}),
        (r'/map.png', MapHandler, {'node': node}),
        (r'/api/gemini_key', KeyHandler),
    ], static_path=os.path.join(HERE, 'static'), websocket_max_message_size=4 * 1024 * 1024)

    cert, keyf = ensure_cert()
    app.listen(conf['port'], ssl_options={'certfile': cert, 'keyfile': keyf})
    hub.loop = tornado.ioloop.IOLoop.current()
    # rclpy 스레드가 있으면 SIGTERM 에 안 꺼지므로 직접 루프를 멈춘다
    signal.signal(signal.SIGTERM, lambda *_: hub.loop.add_callback_from_signal(hub.loop.stop))
    threading.Thread(target=ex.spin, daemon=True).start()
    log.info('https://%s:%d', subprocess.run(['hostname', '-I'], capture_output=True, text=True)
             .stdout.split()[0], conf['port'])
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
