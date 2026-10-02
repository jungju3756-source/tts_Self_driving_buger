#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_keeper.py  —  마지막 AMCL 위치를 기억했다가 다음 실행 때 초기 위치로 넣어줌

    /amcl_pose ──(5초마다)──► ~/maps/last_pose.yaml
    시작 시 last_pose.yaml ──► /initialpose  (AMCL 이 구독할 때까지 기다렸다가 한 번)

AMCL 은 초기 위치를 받기 전엔 map→odom TF 를 안 내보내고, 그동안 Nav2 costmap 이
활성화를 못 해 bringup 이 실패한다 (2026-10-02). 껐던 자리에서 다시 켜면 그대로 맞는다.
로봇을 옮겼으면 RViz '2D Pose Estimate' 로 다시 찍으면 된다.
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class PoseKeeper(Node):

    def __init__(self):
        super().__init__('pose_keeper')
        self.declare_parameter('pose_file', os.path.expanduser('~/maps/last_pose.yaml'))
        self.path = self.get_parameter('pose_file').value
        self.latest = None
        self.saved = None
        self.sent = False          # AMCL 이 받아들였는지 (/amcl_pose 가 나오면 True)
        self.tries = 0

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, LATCHED)
        self.create_timer(2.0, self._send_initial)   # AMCL 이 활성화 전이면 무시하므로 받을 때까지 재전송
        self.create_timer(5.0, self._save)

    def _amcl(self, m):
        if self.tries and not self.sent:
            self.sent = True
            self.get_logger().info('AMCL 이 초기 위치를 받음')
        p = m.pose.pose
        q = p.orientation
        self.latest = {'x': round(p.position.x, 3), 'y': round(p.position.y, 3),
                       'yaw': round(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                               1 - 2 * (q.y * q.y + q.z * q.z)), 3)}

    def _send_initial(self):
        if self.sent:
            return
        if not os.path.exists(self.path):
            self.get_logger().warning(f'{self.path} 없음 — RViz 2D Pose Estimate 로 위치를 찍을 것')
            self.sent = True
            return
        if self.pub.get_subscription_count() == 0:      # AMCL 이 아직 안 떴음
            return
        with open(self.path, encoding='utf-8') as f:
            p = yaml.safe_load(f)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = p['x'], p['y']
        m.pose.pose.orientation.z = math.sin(p['yaw'] / 2)
        m.pose.pose.orientation.w = math.cos(p['yaw'] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.1 ** 2          # ±0.1 m
        cov[35] = math.radians(10) ** 2     # ±10°
        m.pose.covariance = cov
        self.pub.publish(m)
        self.tries += 1
        if self.tries > 30:
            self.get_logger().warning('AMCL 응답 없음 — 재전송 중단')
            self.sent = True
        self.get_logger().info(f"초기 위치 ({p['x']:.2f}, {p['y']:.2f}, {math.degrees(p['yaw']):.0f}°) 전송")

    def _save(self):
        if self.latest is None or self.latest == self.saved:
            return
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.latest, f)
        os.replace(tmp, self.path)
        self.saved = dict(self.latest)


def main():
    rclpy.init()
    node = PoseKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
