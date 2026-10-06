#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
place_manager.py  —  장소(포인트) 저장·삭제·조회 + RViz 마커 (캡스톤 4단계)

    python3 ~/tts_Self_driving_buger/rapa1/place_manager.py

RViz 에서 점을 찍고 이름을 붙인다:
    ① RViz 'Publish Point' 로 지도 클릭        → /clicked_point (방향 없음)
       또는 '2D Goal Pose' 로 클릭+드래그        → /place_pose    (방향 포함)
       → 노란 '?' 마커가 뜬다 (이름 대기 중)
    ② 이름 붙이기:  python3 tools/place.py save 정수기

패드로 저장 (save_button 파라미터를 켰을 때):
    패드로 로봇을 원하는 자리까지 몰고 가서 버튼을 1초 누르면
    로봇 현재 위치·방향이 'N번' (비어 있는 가장 작은 번호) 으로 저장된다.
    성공하면 로봇이 '삑'(BUTTON1), 실패하면 ERROR 소리. 이름은 나중에 rename 으로 바꾼다.
        python3 place_manager.py --ros-args -p save_button:=0

명령 (/places/cmd, std_msgs/String — 음성 에이전트도 rosbridge 로 같은 걸 쓴다):
    save <이름>             방금 찍은 점을 <이름> 으로 저장 (같은 이름이면 덮어씀)
    here <이름>             로봇의 현재 위치·방향을 저장 (TF map→base_footprint)
    delete <이름>
    rename <이전> <새이름>
    alias <이름> <별칭>     별칭 추가 (음성 인식 보정용, 예: alias 정수기 물)
    list

결과:  /places/event (String, JSON 한 줄)  예: {"ok": true, "cmd": "save", "name": "정수기", ...}
목록:  /places/list  (String, JSON, transient local — 늦게 붙어도 최신 목록을 받음)
마커:  /places/markers (MarkerArray, transient local)
저장:  ~/maps/places.yaml  (map 좌표계, 지도와 같이 보관)
"""

import json
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from turtlebot3_msgs.srv import Sound
except ImportError:            # WSL 등 turtlebot3_msgs 가 없는 곳에서도 돌게
    Sound = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
MAX_NAME = 40          # 이름이 길거나 이상하면 거부 (모델 출력이 잘못 들어오는 사고 방지)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PlaceManager(Node):

    def __init__(self):
        super().__init__('place_manager')
        self.declare_parameter('places_file', os.path.expanduser('~/maps/places.yaml'))
        self.declare_parameter('frame', 'map')
        self.declare_parameter('save_button', -1)      # -1 = 패드 저장 끔
        self.declare_parameter('hold_sec', 1.0)
        self.path = self.get_parameter('places_file').value
        self.save_button = self.get_parameter('save_button').value
        self.hold_sec = self.get_parameter('hold_sec').value
        self.btn_since = None          # 버튼을 누르기 시작한 시각
        self.btn_fired = False         # 이번 누름에서 이미 저장했는지
        self.frame = self.get_parameter('frame').value

        self.places = self._load()
        self.pending = None              # {'x','y','yaw'(None 가능)}

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.pub_markers = self.create_publisher(MarkerArray, '/places/markers', LATCHED)
        self.pub_list = self.create_publisher(String, '/places/list', LATCHED)
        self.pub_event = self.create_publisher(String, '/places/event', 10)

        self.create_subscription(PointStamped, '/clicked_point', self._clicked, 10)
        self.create_subscription(PoseStamped, '/place_pose', self._posed, 10)
        self.create_subscription(String, '/places/cmd', self._cmd, 10)
        if self.save_button >= 0:
            self.create_subscription(Joy, '/joy', self._joy, 10)
            self.get_logger().info(f'패드 저장: 버튼 {self.save_button} 을 {self.hold_sec:.1f}초 누르면 현재 위치 저장')
        self.sound = self.create_client(Sound, '/sound') if Sound else None

        self._publish()
        self.get_logger().info(f'장소 {len(self.places)}개 로드: {self.path}')

    # ---------- 저장소 ----------
    def _load(self):
        if not os.path.exists(self.path):
            return {}
        with open(self.path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data.get('places', {})

    def _save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            yaml.safe_dump({'frame': self.frame, 'places': self.places}, f,
                           allow_unicode=True, sort_keys=False)
        os.replace(tmp, self.path)

    # ---------- 입력 ----------
    def _clicked(self, m):
        self.pending = {'x': m.point.x, 'y': m.point.y, 'yaw': None}
        self._pending_info()

    def _posed(self, m):
        p = m.pose.position
        self.pending = {'x': p.x, 'y': p.y, 'yaw': yaw_of(m.pose.orientation)}
        self._pending_info()

    def _pending_info(self):
        p = self.pending
        yaw = '방향 없음' if p['yaw'] is None else f"{math.degrees(p['yaw']):.0f}°"
        self.get_logger().info(f"점 대기: ({p['x']:.2f}, {p['y']:.2f}) {yaw} → 'save <이름>' 으로 저장")
        self._publish()

    def _joy(self, m):
        pressed = self.save_button < len(m.buttons) and m.buttons[self.save_button] == 1
        now = time.monotonic()
        if not pressed:
            self.btn_since, self.btn_fired = None, False
            return
        if self.btn_since is None:
            self.btn_since = now
        if not self.btn_fired and now - self.btn_since >= self.hold_sec:
            self.btn_fired = True                      # 계속 누르고 있어도 한 번만
            n = 1
            while f'{n}번' in self.places:
                n += 1
            res = self._exec('here', [f'{n}번'])
            self._beep(4 if res['ok'] else 3)   # Sound BUTTON1 / ERROR (상수는 .srv 주석에만 있음)

    def _beep(self, value):
        if self.sound and self.sound.service_is_ready():
            self.sound.call_async(Sound.Request(value=value))

    def _robot_pose(self):
        t = self.tf.lookup_transform(self.frame, 'base_footprint', rclpy.time.Time(),
                                     timeout=Duration(seconds=1.0))
        tr = t.transform
        return {'x': tr.translation.x, 'y': tr.translation.y, 'yaw': yaw_of(tr.rotation)}

    def _cmd(self, m):
        parts = m.data.strip().split()
        if parts:
            self._exec(parts[0].lower(), parts[1:])

    def _exec(self, cmd, args):
        try:
            res = self._run(cmd, args)
            res.update(ok=True, cmd=cmd)
        except Exception as e:                      # 명령 하나 실패가 노드를 죽이면 안 됨
            res = {'ok': False, 'cmd': cmd, 'error': str(e)}
        text = json.dumps(res, ensure_ascii=False)
        if res['ok']:          # rclpy 는 같은 줄에서 등급을 바꿔 부르면 예외 → 줄을 나눔
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        self.pub_event.publish(String(data=json.dumps(res, ensure_ascii=False)))
        return res

    def _run(self, cmd, args):
        if cmd == 'list':
            return {'places': self.places}

        if cmd in ('save', 'here'):
            name = self._name(args, 1)
            if cmd == 'save':
                if self.pending is None:
                    raise ValueError('찍은 점이 없음 — RViz 에서 Publish Point 로 먼저 클릭')
                pose = self.pending
            else:
                pose = self._robot_pose()
            old = self.places.get(name, {})
            self.places[name] = {
                'x': round(pose['x'], 3), 'y': round(pose['y'], 3),
                'yaw': None if pose['yaw'] is None else round(pose['yaw'], 3),
                'aliases': old.get('aliases', []),
                'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
            if cmd == 'save':
                self.pending = None
            self._commit()
            return {'name': name, 'place': self.places[name], 'overwritten': bool(old)}

        if cmd == 'delete':
            name = self._name(args, 1)
            self._must_exist(name)
            del self.places[name]
            self._commit()
            return {'name': name}

        if cmd == 'rename':
            if len(args) != 2:
                raise ValueError('사용법: rename <이전> <새이름>')
            old, new = args
            self._must_exist(old)
            self._check(new)
            if new in self.places:
                raise ValueError(f"'{new}' 이미 있음")
            self.places[new] = self.places.pop(old)
            self._commit()
            return {'name': new, 'from': old}

        if cmd == 'alias':
            if len(args) != 2:
                raise ValueError('사용법: alias <이름> <별칭>')
            name, alias = args
            self._must_exist(name)
            self._check(alias)
            al = self.places[name].setdefault('aliases', [])
            if alias not in al:
                al.append(alias)
            self._commit()
            return {'name': name, 'aliases': al}

        raise ValueError(f'모르는 명령: {cmd} (save/here/delete/rename/alias/list)')

    def _name(self, args, n):
        if len(args) != n:
            raise ValueError('이름은 띄어쓰기 없이 한 단어로 (예: 정수기, 회의실1)')
        self._check(args[0])
        return args[0]

    @staticmethod
    def _check(name):
        if len(name) > MAX_NAME or any(c in name for c in '<>{}[]"\''):
            raise ValueError(f'이름이 이상함: {name!r}')

    def _must_exist(self, name):
        if name not in self.places:
            raise ValueError(f"'{name}' 없음 (있는 것: {', '.join(self.places) or '없음'})")

    def _commit(self):
        self._save()
        self._publish()

    # ---------- 출력 ----------
    def _publish(self):
        self.pub_list.publish(String(data=json.dumps(self.places, ensure_ascii=False)))

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        items = list(self.places.items())
        if self.pending:
            items.append(('?', self.pending))
        for name, p in items:
            pend = name == '?'
            color = (1.0, 0.85, 0.0) if pend else (0.1, 0.6, 1.0)
            for kind in ('pin', 'text', 'arrow'):
                if kind == 'arrow' and p.get('yaw') is None:
                    continue
                m = Marker()
                m.header.frame_id = self.frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns, m.id = kind, mid
                m.pose.position.x, m.pose.position.y = p['x'], p['y']
                m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
                if kind == 'pin':
                    m.type = Marker.CYLINDER
                    m.scale.x = m.scale.y = 0.15
                    m.scale.z = 0.05
                elif kind == 'text':
                    m.type = Marker.TEXT_VIEW_FACING
                    m.text = name
                    m.pose.position.z = 0.35
                    m.scale.z = 0.25
                    m.color.r = m.color.g = m.color.b = 0.1     # 지도 바탕이 흰색이라 어두운 글자
                else:
                    m.type = Marker.ARROW
                    m.pose.orientation.z = math.sin(p['yaw'] / 2)
                    m.pose.orientation.w = math.cos(p['yaw'] / 2)
                    m.scale.x, m.scale.y, m.scale.z = 0.35, 0.05, 0.05
                arr.markers.append(m)
            mid += 1
        self.pub_markers.publish(arr)


def main():
    rclpy.init()
    node = PlaceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
