#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drive_monitor.py  —  주행/개입 실시간 모니터 (라파1)

조이스틱 입력이 안전 로직을 거쳐 모터로 나가는 과정을 한 줄로 보여준다.
시연할 때 이 창을 띄워두면 개입이 언제 걸리는지 눈으로 보인다.

    조이스틱  0.20 →  모터  0.20   CLEAR
    조이스틱  0.20 →  모터  0.00   STOP          ← 낭떠러지, 전진 차단
    조이스틱  0.20 →  모터  0.08   DETOUR_LEFT   ← 정지 1초 뒤 우회

실행:
    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=40
    python3 ~/cliff_mqtt/rapa1/drive_monitor.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

G, Y, R, D, RST = '\033[92m', '\033[93m', '\033[91m', '\033[2m', '\033[0m'


class DriveMonitor(Node):

    def __init__(self):
        super().__init__('drive_monitor')
        self.manual = (0.0, 0.0)
        self.out = (0.0, 0.0)
        self.state = '(대기)'
        self.manual_t = 0.0
        self.peak_gap = 0.0

        self.create_subscription(Twist, '/cmd_vel_manual', self._manual, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._out, 10)
        self.create_subscription(String, '/cliff/state', self._state, 10)
        self.create_timer(0.1, self._print)
        print('조이스틱을 움직여 보세요.  Ctrl+C 로 종료\n')

    def _manual(self, m):
        self.manual = (m.linear.x, m.angular.z)
        self.manual_t = time.time()

    def _out(self, m):
        self.out = (m.twist.linear.x, m.twist.angular.z)

    def _state(self, m):
        self.state = m.data

    def _print(self):
        mx, mz = self.manual
        ox, oz = self.out
        st = self.state.split('|')[0]

        # 개입이 실제로 명령을 바꿨는지
        blocked = abs(mx) > 0.02 and abs(ox) < 0.02
        reduced = abs(mx) > 0.02 and 0.02 <= abs(ox) < abs(mx) * 0.9

        if blocked:
            color, tag = R, '← 전진 차단됨'
        elif reduced:
            color, tag = Y, '← 감속/우회'
        elif st == 'CLEAR':
            color, tag = G, ''
        else:
            color, tag = D, ''

        stale = D + ' (입력 없음)' + RST if time.time() - self.manual_t > 0.5 else ''

        sys.stdout.write(
            '\r  조이스틱 {:+.2f}/{:+.2f}  →  모터 {}{:+.2f}/{:+.2f}{}   '
            '{:<22} {}{}     '
            .format(mx, mz, color, ox, oz, RST, st, tag, stale))
        sys.stdout.flush()


def main():
    rclpy.init()
    node = DriveMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass          # 이미 내려간 컨텍스트 — 종료 중이므로 무시


if __name__ == '__main__':
    main()
