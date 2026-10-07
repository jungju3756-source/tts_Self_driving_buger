#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
voice_ptt.py  —  라파 마이크로 듣고 장소로 이동 (향후_계획.md 2·3단계)

mode:=listen (기본)  마이크를 계속 켜 두고 말소리 구간만 잘라 받아쓴다
    소리 크기(RMS)가 주변 소음보다 확실히 커지면 '말 시작', 0.6초 조용하면 '말 끝'
    0.5초보다 짧거나 6초보다 긴 소리는 Gemini 에 보내지도 않고 버림
    받아쓴 말은 web/intent.py parse_strict 로 엄격하게 해석:
        "1번으로 가" "3번 이동해" (N번 + 이동 동사) → 이동,  "멈춰/정지/그만" → 정지,  그 외 → 조용히 무시
    주행 중에는 '멈춰' 만 받는다 (모터 소리 오인식으로 목적지가 바뀌지 않게)
mode:=ptt            어제(2026-10-06) 방식 — B 를 누르고 있는 동안만 녹음
    두 모드 모두 B 누르고 말하기는 된다 (시끄러울 때 백업). B 로 한 말은 기존 parse_command 로 해석.

    받아쓰기 Gemini(REST) → Nav2 navigate_to_pose / 정지 = 모든 목표 취소 + 0 명령
    결과음   출발 BUTTON1, 도착/정지 ON, B 로 한 말을 못 알아들음/실패 = ERROR (로봇 부저 /sound)

    python3 voice_ptt.py --ros-args -p mode:=listen -p mic:=plughw:CARD=C920,DEV=0

마이크 장치를 열려면 audio 그룹 권한이 필요하다.
상태는 /voice/event (String JSON) 로 보낸다 → 웹 조종석이 표시
    {"state": "listening", "mode", "noise", "moving"}   (상시 듣기 대기 중, 5초마다)
    {"state": "speech"}  (상시 듣기: 말소리 감지)   {"state": "recording"}  (B 누름)
    {"state": "processing", "sec", "src"} → {"state": "result", "text", "action", "place", "reply", "stt_s", "src", "why"}
    action: go | stop | ignore (상시 듣기에서 명령 아님) | unknown/ask/... (B 로 한 말)
    {"state": "error", "error"},  {"state": "idle"},  {"state": "nav", "nav": moving|arrived|failed|canceled, "place"}
"""

import base64
import collections
import io
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.request
import wave
from array import array

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

try:
    from turtlebot3_msgs.srv import Sound
except ImportError:
    Sound = None

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'web'))
from intent import parse_command, parse_strict  # noqa: E402

KEY_FILE = os.path.expanduser('~/.config/robot_web/gemini_api_key')
API = 'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key={key}'
RATE = 16000
CHUNK = 3200           # 100 ms (16 kHz × 2 byte)
MIN_SEC = 0.4          # B 를 이보다 짧게 누르면 무시 (실수로 스친 것)
MAX_SEC = 15.0         # B 를 너무 오래 누르면 여기서 자름
BEEP = {'ON': 1, 'ERROR': 3, 'BUTTON1': 4}   # turtlebot3_msgs/Sound — 상수가 .srv 에 주석으로만 있음
BEEP_MUTE = 0.8        # 부저 소리를 말로 잡지 않게 이 시간 동안 말 시작 감지 안 함
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def read_key():
    with open(KEY_FILE, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                return line
    raise RuntimeError(f'Gemini 키 없음: {KEY_FILE}')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rms_of(chunk):
    a = array('h', chunk[:len(chunk) // 2 * 2])
    return math.sqrt(sum(x * x for x in a) / len(a)) if a else 0.0


class VoicePTT(Node):

    def __init__(self):
        super().__init__('voice_ptt')
        self.mode = self.declare_parameter('mode', 'listen').value          # listen | ptt
        self.button = self.declare_parameter('button', 1).value
        self.mic = self.declare_parameter('mic', 'plughw:CARD=C920,DEV=0').value
        self.model = self.declare_parameter('model', 'gemini-3.5-flash-lite').value
        # 상시 듣기 말소리 감지 (RMS, 16 bit 기준). 현장에서 noise 값 보고 조정
        self.ratio_on = self.declare_parameter('vad_ratio', 3.0).value      # 소음의 몇 배면 말로 보나
        self.min_on = self.declare_parameter('vad_min_rms', 400.0).value    # 아무리 조용해도 이 이상은 돼야 말
        self.start_n = self.declare_parameter('vad_start_chunks', 3).value  # 0.1 s 단위, 이만큼 연속 커야 시작
        self.end_n = self.declare_parameter('vad_end_chunks', 6).value      # 이만큼 연속 조용하면 끝
        self.seg_min = self.declare_parameter('seg_min_sec', 0.5).value
        self.seg_max = self.declare_parameter('seg_max_sec', 6.0).value
        self.key = read_key()

        self.places = {}
        self.pose = None
        self.nav_active = set()     # Nav2 에서 진행 중인 목표 id (웹·RViz 에서 보낸 것 포함)
        self.create_subscription(String, '/places/list', lambda m: setattr(self, 'places', json.loads(m.data)), LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, LATCHED)
        self.create_subscription(Joy, '/joy', self._joy, 10)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self._nav_status, 10)
        self.pub_event = self.create_publisher(String, '/voice/event', 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_cli = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.sound = self.create_client(Sound, '/sound') if Sound else None

        self.pressed = False
        self.ptt_buf = None         # B 누르는 동안 모으는 소리 (None = 안 누름)
        self.t_press = 0.0
        self.busy = False           # 받아쓰기 중엔 새 말 안 받음
        self.goal_handle = None
        self.mute_until = 0.0
        # 말소리 감지 상태
        self.noise = None
        self.pre = collections.deque(maxlen=3 + self.start_n)   # 말 시작 직전 0.3 s 포함
        self.seg = None
        self.loud = 0
        self.quiet = 0
        self.too_long = False

        threading.Thread(target=self._mic_loop, daemon=True).start()
        self.create_timer(5.0, self._status)
        what = '상시 듣기 ("N번으로 가" / "멈춰")' if self.mode == 'listen' else 'B 누르고 말하기만'
        self.get_logger().info(f'{what} + B 버튼({self.button}) — 마이크 {self.mic}, 모델 {self.model}')

    # ---------- 입력 ----------
    def _amcl(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _nav_status(self, m):
        self.nav_active = {bytes(s.goal_info.goal_id.uuid) for s in m.status_list if s.status in ACTIVE}

    @property
    def moving(self):
        return bool(self.nav_active)

    def _status(self):
        if self.mode == 'listen' and self.seg is None and self.ptt_buf is None and not self.busy:
            self._event(state='listening', mode=self.mode, noise=round(self.noise or 0), moving=self.moving)

    def _joy(self, m):
        down = len(m.buttons) > self.button and m.buttons[self.button] == 1
        if down and not self.pressed:
            self.pressed = True
            if self.busy:
                self.get_logger().warn('아직 앞의 말을 처리하는 중')
                return
            self.seg = None                     # 상시 듣기 중이던 구간은 버리고 B 우선
            self.t_press = time.time()
            self.ptt_buf = bytearray()
            self._beep('ON')
            self.get_logger().info('● 녹음 시작 (B)')
            self._event(state='recording')
        elif not down and self.pressed:
            self.pressed = False
            buf, self.ptt_buf = self.ptt_buf, None
            if buf is None:
                return
            sec = len(buf) / (RATE * 2)
            if sec < MIN_SEC:
                self.get_logger().info(f'너무 짧음 ({sec:.1f}s) — 무시')
                self._event(state='idle')
                return
            self._dispatch(bytes(buf), 'ptt')

    # ---------- 마이크 (항상 열어 둠) ----------
    def _mic_loop(self):
        while rclpy.ok():
            try:
                proc = subprocess.Popen(
                    ['arecord', '-q', '-D', self.mic, '-f', 'S16_LE', '-r', str(RATE), '-c', '1', '-t', 'raw'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except OSError as e:
                self.get_logger().error(f'arecord 실행 실패: {e}')
                time.sleep(5.0)
                continue
            self.proc = proc
            while True:
                chunk = proc.stdout.read(CHUNK)
                if not chunk:
                    break
                self._on_chunk(chunk)
            err = proc.stderr.read().decode(errors='replace').strip()
            proc.wait()
            if not rclpy.ok():
                return
            self.get_logger().error(f'마이크 끊김 — 3초 뒤 다시 엶 ({err or proc.returncode})')
            self._event(state='error', error=f'마이크 끊김: {err or proc.returncode}')
            time.sleep(3.0)

    def _on_chunk(self, chunk):
        if self.ptt_buf is not None:
            self.ptt_buf += chunk
            if len(self.ptt_buf) > MAX_SEC * RATE * 2:     # 너무 오래 누름 → 뗀 것처럼 처리
                buf, self.ptt_buf = self.ptt_buf, None
                self._dispatch(bytes(buf), 'ptt')
            return
        if self.mode != 'listen':
            return
        r = rms_of(chunk)
        if self.noise is None:
            self.noise = max(r, 50.0)
        on = max(self.noise * self.ratio_on, self.min_on)
        off = max(self.noise * 2.0, self.min_on * 0.6)

        if self.seg is None:
            self.pre.append(chunk)
            if r < on:                                     # 소음 수준은 말 아닐 때만 천천히 따라감
                self.noise = max(0.95 * self.noise + 0.05 * r, 50.0)
                self.loud = 0
                return
            if time.time() < self.mute_until or self.busy:
                self.loud = 0
                return
            self.loud += 1
            if self.loud >= self.start_n:
                self.seg = bytearray(b''.join(self.pre))
                self.quiet, self.too_long = 0, False
                self.get_logger().info(f'… 말소리 감지 (rms {r:.0f} / 소음 {self.noise:.0f})')
                self._event(state='speech')
            return

        self.seg += chunk
        self.quiet = self.quiet + 1 if r < off else 0
        if len(self.seg) > self.seg_max * RATE * 2 and not self.too_long:
            self.too_long = True                           # 계속 말하는 중 (대화·음악) → 끝날 때까지 기다렸다 버림
        if self.quiet < self.end_n:
            return
        seg, self.seg, self.loud = self.seg, None, 0
        self.pre.clear()
        voiced = len(seg) / (RATE * 2) - self.quiet * CHUNK / (RATE * 2)
        if self.too_long or voiced < self.seg_min:
            self.get_logger().info(f'버림 — {"너무 김" if self.too_long else f"너무 짧음 {voiced:.1f}s"}')
            self._event(state='idle')
            return
        self._dispatch(bytes(seg[:len(seg) - max(self.quiet - 2, 0) * CHUNK]), 'listen')

    def _dispatch(self, pcm, src):
        if self.busy:
            self._event(state='idle')
            return
        sec = len(pcm) / (RATE * 2)
        self.get_logger().info(f'■ {"B 녹음" if src == "ptt" else "말"} {sec:.1f}s → 받아쓰기')
        self._event(state='processing', sec=round(sec, 1), src=src)
        self.busy = True
        threading.Thread(target=self._handle, args=(pcm, src), daemon=True).start()

    # ---------- 받아쓰기 · 해석 ----------
    def _transcribe(self, pcm, src):
        wav = io.BytesIO()
        with wave.open(wav, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(pcm)
        if src == 'ptt':
            names = ', '.join(self.places) or '없음'
            prompt = ('로봇에게 하는 한국어 음성 명령이다. 들은 말을 그대로 받아 적어라. 받아 적은 문장만 출력하고, '
                      f'말이 없으면 아무것도 출력하지 마라. 등록된 장소 이름: {names}')
        else:   # 상시 듣기 — 장소 이름을 알려주면 잡음을 "1번으로 가" 로 지어낼 수 있어서 안 넣는다
            prompt = ('마이크에 들어온 소리다. 사람이 한국어로 말한 부분만 들은 그대로 받아 적어라. '
                      '받아 적은 문장만 출력하고, 사람 말이 없거나 알아들을 수 없으면 아무것도 출력하지 마라. '
                      '추측해서 채우지 마라.')
        body = {'contents': [{'parts': [
            {'text': prompt},
            {'inline_data': {'mime_type': 'audio/wav', 'data': base64.b64encode(wav.getvalue()).decode()}}]}]}
        req = urllib.request.Request(API.format(model=self.model, key=self.key), data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=15) as r:
            d = json.load(r)
        parts = d.get('candidates', [{}])[0].get('content', {}).get('parts', [])
        return ''.join(p.get('text', '') for p in parts).strip()

    def _handle(self, pcm, src):
        try:
            t = time.time()
            try:
                text = self._transcribe(pcm, src)
            except Exception as e:   # 네트워크·키 오류
                self.get_logger().error(f'받아쓰기 실패: {e}')
                if src == 'ptt':
                    self._beep('ERROR')
                self._event(state='error', error=f'받아쓰기 실패: {e}')
                return
            stt = round(time.time() - t, 2)
            if src == 'ptt':
                action, place, reply = parse_command(text, self.places)
            else:
                action, place, reply = parse_strict(text, self.places)
            why = None
            if action == 'go' and self.moving:
                action, place, reply, why = 'ignore', None, None, '주행 중 — "멈춰" 만 받음'
            self.get_logger().info(f'"{text}" ({stt}s, {src}) → {action} {place or ""} {why or reply or ""}')
            self._event(state='result', text=text, action=action, place=place, reply=reply,
                        stt_s=stt, src=src, why=why)
            if action == 'stop':
                self.stop()
                self._beep('ON')
            elif action == 'go':
                self.goto(place)
            elif src == 'ptt':
                self._beep('ERROR')     # 상시 듣기에서 무시한 말은 소리 안 냄
        finally:
            self.busy = False

    # ---------- 동작 ----------
    def goto(self, name):
        p = self.places.get(name)
        if p is None or not self.nav_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 준비 안 됨' if p else f"'{name}' 없음")
            self._beep('ERROR')
            self._event(state='nav', nav='failed', place=name)
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
        self.nav_ac.send_goal_async(g).add_done_callback(lambda f: self._accepted(f, name))

    def _accepted(self, fut, name):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('목표 거부됨')
            self._beep('ERROR')
            self._event(state='nav', nav='failed', place=name)
            return
        self.goal_handle = gh
        self._beep('BUTTON1')
        self.get_logger().info(f'→ {name} 출발')
        self._event(state='nav', nav='moving', place=name)
        gh.get_result_async().add_done_callback(lambda f: self._done(f, gh, name))

    def _done(self, fut, gh, name):
        if gh is not self.goal_handle:
            return
        self.goal_handle = None
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✓ {name} 도착')
            self._beep('ON')
            self._event(state='nav', nav='arrived', place=name)
        elif status == GoalStatus.STATUS_CANCELED:
            self._event(state='nav', nav='canceled', place=name)
        else:
            self.get_logger().warn(f'✗ {name} 실패 (status {status})')
            self._beep('ERROR')
            self._event(state='nav', nav='failed', place=name)

    def stop(self):
        # 빈 CancelGoal 요청 = 모든 목표 취소 (웹·RViz 에서 보낸 목표도 멈춤)
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())
        elif self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        for _ in range(3):
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(m)
        self.get_logger().warn('■ 정지')

    def _event(self, **kw):
        self.pub_event.publish(String(data=json.dumps(kw, ensure_ascii=False)))

    def _beep(self, kind):
        self.mute_until = time.time() + BEEP_MUTE
        if self.sound and self.sound.service_is_ready():
            self.sound.call_async(Sound.Request(value=BEEP[kind]))


def main():
    rclpy.init()
    node = VoicePTT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        proc = getattr(node, 'proc', None)
        if proc:
            proc.terminate()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
