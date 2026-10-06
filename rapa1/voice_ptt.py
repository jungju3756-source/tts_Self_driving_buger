#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
voice_ptt.py  —  패드 B 버튼을 누르고 있는 동안 라파 마이크로 녹음 → 장소로 이동 (향후_계획.md 2·3단계)

    B 누름   → '삑' + 녹음 시작 (arecord, 16 kHz mono)
    B 뗌     → Gemini 로 받아쓰기 → web/intent.py 규칙 해석 → Nav2 navigate_to_pose
               "1번 가줘" → 1번으로 이동,  "멈춰" → 목표 취소 + 정지
    결과음   성공 = BUTTON1, 못 알아들음/실패 = ERROR  (로봇 부저 /sound — 아직 스피커가 없어서)

    python3 voice_ptt.py --ros-args -p button:=1 -p mic:=plughw:CARD=C920,DEV=0

마이크 장치를 열려면 audio 그룹 권한이 필요하다 (재로그인 전이면 `sg audio -c "python3 voice_ptt.py"`).
상태는 /voice/event (String JSON) 로 보낸다 → 웹 조종석이 '듣는 중' 표시·대화창에 띄움
    {"state": "recording"} → {"state": "processing", "sec"} → {"state": "result", "text", "action", "place", "reply", "stt_s"}
    {"state": "error", "error"},  {"state": "nav", "nav": moving|arrived|failed|canceled, "place"}
"""

import base64
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

import rclpy
from action_msgs.msg import GoalStatus
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
from intent import parse_command  # noqa: E402

KEY_FILE = os.path.expanduser('~/.config/robot_web/gemini_api_key')
API = 'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key={key}'
RATE = 16000
MIN_SEC = 0.4          # 이보다 짧게 누르면 무시 (실수로 스친 것)
MAX_SEC = 15.0         # 너무 오래 누르면 여기서 자름
BEEP = {'ON': 1, 'ERROR': 3, 'BUTTON1': 4}   # turtlebot3_msgs/Sound — 상수가 .srv 에 주석으로만 있음
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


class VoicePTT(Node):

    def __init__(self):
        super().__init__('voice_ptt')
        self.button = self.declare_parameter('button', 1).value
        self.mic = self.declare_parameter('mic', 'plughw:CARD=C920,DEV=0').value
        self.model = self.declare_parameter('model', 'gemini-3.5-flash-lite').value
        self.key = read_key()

        self.places = {}
        self.pose = None
        self.create_subscription(String, '/places/list', lambda m: setattr(self, 'places', json.loads(m.data)), LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, LATCHED)
        self.create_subscription(Joy, '/joy', self._joy, 10)
        self.pub_event = self.create_publisher(String, '/voice/event', 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sound = self.create_client(Sound, '/sound') if Sound else None

        self.pressed = False
        self.rec = None             # arecord 프로세스
        self.buf = bytearray()
        self.t_press = 0.0
        self.busy = False           # 받아쓰기 중엔 새 녹음 안 받음
        self.goal_handle = None
        self.get_logger().info(f'B 버튼({self.button}) 누르고 말하기 — 마이크 {self.mic}, 모델 {self.model}')

    # ---------- 입력 ----------
    def _amcl(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _joy(self, m):
        down = len(m.buttons) > self.button and m.buttons[self.button] == 1
        if down and not self.pressed:
            self.pressed = True
            self._start_rec()
        elif not down and self.pressed:
            self.pressed = False
            self._stop_rec()

    def _start_rec(self):
        if self.busy:
            self.get_logger().warn('아직 앞의 말을 처리하는 중')
            return
        self.buf = bytearray()
        try:
            self.rec = subprocess.Popen(
                ['arecord', '-q', '-D', self.mic, '-f', 'S16_LE', '-r', str(RATE), '-c', '1', '-t', 'raw'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as e:
            self.get_logger().error(f'arecord 실행 실패: {e}')
            self.rec = None
            return
        self.t_press = time.time()
        threading.Thread(target=self._read_rec, args=(self.rec,), daemon=True).start()
        self._beep('ON')
        self.get_logger().info('● 녹음 시작')
        self._event(state='recording')

    def _read_rec(self, proc):
        while True:
            chunk = proc.stdout.read(3200)          # 100 ms
            if not chunk:
                break
            self.buf += chunk
            if len(self.buf) > MAX_SEC * RATE * 2:
                proc.terminate()
                break

    def _stop_rec(self):
        proc, self.rec = self.rec, None
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        err = proc.stderr.read().decode(errors='replace').strip()
        sec = len(self.buf) / (RATE * 2)
        if sec < MIN_SEC:
            if err:
                self.get_logger().error(f'녹음 실패: {err}')
                self._beep('ERROR')
                self._event(state='error', error=f'녹음 실패: {err}')
            else:
                self.get_logger().info(f'너무 짧음 ({sec:.1f}s) — 무시')
                self._event(state='idle')
            return
        self.get_logger().info(f'■ 녹음 끝 {sec:.1f}s → 받아쓰기')
        self._event(state='processing', sec=round(sec, 1))
        self.busy = True
        threading.Thread(target=self._handle, args=(bytes(self.buf),), daemon=True).start()

    # ---------- 받아쓰기 · 해석 ----------
    def _transcribe(self, pcm):
        wav = io.BytesIO()
        with wave.open(wav, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(pcm)
        names = ', '.join(self.places) or '없음'
        prompt = ('로봇에게 하는 한국어 음성 명령이다. 들은 말을 그대로 받아 적어라. 받아 적은 문장만 출력하고, '
                  f'말이 없으면 아무것도 출력하지 마라. 등록된 장소 이름: {names}')
        body = {'contents': [{'parts': [
            {'text': prompt},
            {'inline_data': {'mime_type': 'audio/wav', 'data': base64.b64encode(wav.getvalue()).decode()}}]}]}
        req = urllib.request.Request(API.format(model=self.model, key=self.key), data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=15) as r:
            d = json.load(r)
        parts = d.get('candidates', [{}])[0].get('content', {}).get('parts', [])
        return ''.join(p.get('text', '') for p in parts).strip()

    def _handle(self, pcm):
        try:
            t = time.time()
            try:
                text = self._transcribe(pcm)
            except Exception as e:   # 네트워크·키 오류
                self.get_logger().error(f'받아쓰기 실패: {e}')
                self._beep('ERROR')
                self._event(state='error', error=f'받아쓰기 실패: {e}')
                return
            stt = round(time.time() - t, 2)
            action, place, reply = parse_command(text, self.places)
            self.get_logger().info(f'"{text}" ({stt}s) → {action} {place or ""} {reply or ""}')
            self._event(state='result', text=text, action=action, place=place, reply=reply, stt_s=stt)
            if action == 'stop':
                self.stop()
                self._beep('ON')
            elif action == 'go':
                self.goto(place)
            else:
                self._beep('ERROR')
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
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        for _ in range(3):
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(m)
        self.get_logger().warn('■ 정지')

    def _event(self, **kw):
        self.pub_event.publish(String(data=json.dumps(kw, ensure_ascii=False)))

    def _beep(self, kind):
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
        if node.rec:
            node.rec.terminate()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
