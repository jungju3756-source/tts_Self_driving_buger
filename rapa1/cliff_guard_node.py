#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cliff_guard_node.py  —  라파1 (터틀봇 본체)

블루투스 조이스틱 명령을 그대로 통과시키되, 라파2의 cliff_bridge 가 MQTT 로
내려보낸 회피 지시가 있으면 그 지시로 덮어쓴다 (하드 오버라이드).
라파1 쪽만 ROS2 를 쓴다 — 모터가 OpenCR/turtlebot3_node 에 물려 있기 때문이다.

    [8BitDo 패드] --BT--> joy --> teleop_twist_joy --> /cmd_vel_manual
                                                            |
                            (MQTT) wheelchair/safety/command |
                                        |                   v
                                        +-------> cliff_guard_node
                                                            |
                                                            v
                                                  /cmd_vel (TwistStamped)
                                                            |
                                                     turtlebot3_node -> OpenCR

동작 규칙
---------
  CLEAR          조이스틱 그대로 통과
  STOP           전진 금지. 제자리 회전과 후진은 허용 (탈출 경로를 막지 않는다)
  DETOUR_L/R     사용자가 전진을 원할 때만 저속으로 해당 방향 우회
  CORRECT_L/R    전진 속도는 유지하고 각속도에만 보정을 더함
  DEGRADED       센서 이상 — 정지시키지 않고 속도만 제한 (휠체어는 못 움직이면 그것도 위험)
  FAULT          위험 중 링크 상실 — STOP 과 동일하게 취급
  (무신호)       watchdog_s 초과 시 마지막 오버라이드는 유지, 아니면 DEGRADED

실행:
  python3 cliff_guard_node.py --ros-args -p config_file:=/home/jungju/cliff_mqtt/config/cliff_config.yaml
또는 단순히:
  python3 cliff_guard_node.py
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import yaml

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'cliff_config.yaml')

FULL_STOP_ACTIONS = ('STOP', 'FAULT')


def _make_client(client_id):
    """paho-mqtt 1.x / 2.x 양쪽에서 동작하는 클라이언트 생성."""
    try:
        from paho.mqtt.client import CallbackAPIVersion
        return mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        return mqtt.Client(client_id=client_id)


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class CliffGuardNode(Node):

    def __init__(self):
        super().__init__('cliff_guard')

        self.declare_parameter('config_file', DEFAULT_CONFIG)
        self.declare_parameter('cmd_in_stamped', False)
        cfg_path = self.get_parameter('config_file').value
        self.in_stamped = bool(self.get_parameter('cmd_in_stamped').value)

        with open(cfg_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        g = cfg['guard']
        self.mcfg = cfg['mqtt']
        self.topic_cmd = self.mcfg['topics']['command']

        self.cmd_in = g['cmd_in']
        self.cmd_out = g['cmd_out']
        self.rate = float(g['rate_hz'])
        self.watchdog_s = float(g['watchdog_s'])
        self.degraded_scale = float(g['degraded_speed_scale'])
        self.detour_lin = float(g['detour_linear'])
        self.detour_ang = float(g['detour_angular'])
        self.max_ang = float(g['max_angular'])
        self.allow_reverse = bool(g.get('allow_reverse_when_stopped', True))

        # ---- 공유 상태 (MQTT 스레드 ↔ ROS 타이머) ----
        self._lock = threading.Lock()
        self._cmd = None
        self._cmd_t = 0.0

        self._manual = Twist()
        self._manual_t = None

        # ---- ROS 인터페이스 ----
        if self.in_stamped:
            self.create_subscription(TwistStamped, self.cmd_in,
                                     self._on_manual_stamped, 10)
        else:
            self.create_subscription(Twist, self.cmd_in, self._on_manual, 10)
        self._pub = self.create_publisher(TwistStamped, self.cmd_out, 10)
        self._pub_state = self.create_publisher(String, g['state_topic'], 10)

        self._last_action = None
        self.create_timer(1.0 / self.rate, self._tick)

        self._start_mqtt()
        self.get_logger().info(
            'cliff_guard: {} -> {} | MQTT {}:{} topic={}'.format(
                self.cmd_in, self.cmd_out, self.mcfg['host'], self.mcfg['port'],
                self.topic_cmd))

    # ------------------------------------------------------------------ MQTT
    def _start_mqtt(self):
        if mqtt is None:
            self.get_logger().error('paho-mqtt 없음 — pip3 install paho-mqtt')
            return
        self.client = _make_client('cliff-guard')
        user = self.mcfg.get('username') or ''
        if user:
            self.client.username_pw_set(user, self.mcfg.get('password') or '')
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = lambda *a: self.get_logger().warning(
            'MQTT 연결 끊김 — 재연결 시도')
        self.client.connect_async(self.mcfg['host'], int(self.mcfg['port']),
                                  int(self.mcfg.get('keepalive', 15)))
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, *a):
        if rc == 0:
            client.subscribe(self.topic_cmd, qos=1)
            self.get_logger().info('MQTT 연결됨 — 구독: {}'.format(self.topic_cmd))
        else:
            self.get_logger().error('MQTT 연결 실패 rc={}'.format(rc))

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except Exception:
            return
        with self._lock:
            self._cmd = data
            self._cmd_t = time.time()

    # ------------------------------------------------------------------- ROS
    def _on_manual(self, msg: Twist):
        self._manual = msg
        self._manual_t = time.time()

    def _on_manual_stamped(self, msg: TwistStamped):
        self._manual = msg.twist
        self._manual_t = time.time()

    # -------------------------------------------------------------- 오버라이드
    def _apply(self, action, gain, user: Twist):
        """조이스틱 명령에 안전 지시를 적용해 최종 Twist 를 만든다."""
        out = Twist()
        lin = float(user.linear.x)
        ang = float(user.angular.z)

        if action in FULL_STOP_ACTIONS:
            # 전진만 막는다. 후진·제자리회전은 탈출 수단이므로 남겨 둔다.
            out.linear.x = min(lin, 0.0) if self.allow_reverse else 0.0
            out.angular.z = _clip(ang, -self.max_ang, self.max_ang)

        elif action in ('DETOUR_LEFT', 'DETOUR_RIGHT'):
            if lin > 0.02:
                # 사용자가 계속 전진을 원할 때만 우회를 실행한다
                out.linear.x = min(self.detour_lin, lin)
                turn = self.detour_ang if action == 'DETOUR_LEFT' else -self.detour_ang
                out.angular.z = _clip(ang + turn, -self.max_ang, self.max_ang)
            else:
                out.linear.x = min(lin, 0.0) if self.allow_reverse else 0.0
                out.angular.z = _clip(ang, -self.max_ang, self.max_ang)

        elif action in ('CORRECT_LEFT', 'CORRECT_RIGHT'):
            # 전진 유지 + 반대편으로 방향 보정 (PDF 시나리오 1-②)
            out.linear.x = lin
            bias = self.detour_ang * float(gain)
            bias = bias if action == 'CORRECT_LEFT' else -bias
            out.angular.z = _clip(ang + bias, -self.max_ang, self.max_ang)

        elif action == 'DEGRADED':
            out.linear.x = lin * self.degraded_scale
            out.angular.z = _clip(ang * self.degraded_scale, -self.max_ang, self.max_ang)

        else:   # CLEAR
            out.linear.x = lin
            out.angular.z = _clip(ang, -self.max_ang, self.max_ang)

        return out

    def _tick(self):
        """타이머 진입점. 안에서 무슨 일이 나든 명령 발행은 멈추지 않는다.

        이 노드가 조용히 죽으면 /cmd_vel 이 끊기는데, 그러면 하위 컨트롤러가
        마지막 명령을 붙들고 계속 달릴 수 있다. 예외가 나면 정지 명령을 낸다.
        """
        try:
            self._tick_impl()
        except Exception as exc:
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            self._pub.publish(out)          # 영속도 = 정지
            try:
                self.get_logger().error(
                    'tick 예외 — 정지 명령 발행: {}'.format(exc),
                    throttle_duration_sec=2.0)
            except Exception:
                pass

    def _tick_impl(self):
        now = time.time()
        with self._lock:
            cmd = dict(self._cmd) if self._cmd else None
            age = now - self._cmd_t if self._cmd else None

        if cmd is None:
            action, gain, reason = 'DEGRADED', 1.0, 'no_command_yet'
        elif age > self.watchdog_s:
            # 라파2와의 링크가 끊겼다. 마지막이 개입 지시였다면 그대로 유지한다.
            if cmd.get('override'):
                action, gain, reason = 'FAULT', 1.0, 'link_lost_while_override'
            else:
                action, gain, reason = 'DEGRADED', 1.0, 'link_lost'
        else:
            action = cmd.get('action', 'CLEAR')
            gain = float(cmd.get('gain', 1.0))
            reason = cmd.get('reason', '')

        # 조이스틱 입력이 끊긴 경우엔 정지 상태를 출력한다
        user = self._manual if (self._manual_t and now - self._manual_t <= 0.5) else Twist()

        twist = self._apply(action, gain, user)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = twist
        self._pub.publish(out)

        s = String()
        s.data = '{}|{}'.format(action, reason)
        self._pub_state.publish(s)

        if action != self._last_action:
            self._last_action = action
            text = '안전 상태: {} ({}) zone={} range={}m depth={}m'.format(
                action, reason, (cmd or {}).get('zone'),
                (cmd or {}).get('range_m'), (cmd or {}).get('depth_m'))
            # rclpy 는 같은 호출 지점에서 심각도를 바꾸면 예외를 던진다.
            # 로깅 때문에 제어 루프가 멈추는 일은 절대 없어야 하므로
            # 심각도별로 호출 지점을 나누고, 그래도 실패하면 삼킨다.
            try:
                if action == 'CLEAR':
                    self.get_logger().info(text)
                else:
                    self.get_logger().warning(text)
            except Exception:
                pass

    def destroy_node(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CliffGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
