#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pad_gate.py  —  패드 명령이 Nav2 를 방해하지 않게 거르는 중계

    teleop_twist_joy ─(/cmd_vel_joy)─► pad_gate ─(/cmd_vel)─► turtlebot3_node
                                         Nav2 ─(/cmd_vel)─┘

teleop_twist_joy(require_enable_button: false)는 패드를 안 만져도 joy 메시지가 올 때마다
(autorepeat 20Hz) '정지(0)' 를 /cmd_vel 에 계속 보낸다. 그러면 Nav2 주행 명령과 번갈아
들어가 로봇이 덜컥거린다 (2026-10-02 실측: 대기 중 0 명령 ~14Hz).

규칙:
    0 이 아닌 명령      → 그대로 전달 (누르고 있는 동안 20Hz)
    0 으로 바뀌는 순간  → 정지 명령을 3번만 보내고 그 뒤로는 침묵
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

STOP_REPEAT = 3


class PadGate(Node):

    def __init__(self):
        super().__init__('pad_gate')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/cmd_vel_joy', self._cb, 10)
        self.stops_left = 0

    def _cb(self, m):
        t = m.twist
        moving = any(abs(v) > 1e-6 for v in (t.linear.x, t.linear.y, t.angular.z))
        if moving:
            self.stops_left = STOP_REPEAT
            self.pub.publish(m)
        elif self.stops_left > 0:
            self.stops_left -= 1
            self.pub.publish(m)


def main():
    rclpy.init()
    node = PadGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
