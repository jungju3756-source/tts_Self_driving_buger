#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goto.py  —  저장한 장소로 Nav2 주행 + 도착 오차 측정 (캡스톤 3단계, 음성 없이)

    python3 goto.py 1번              # 1번으로 이동
    python3 goto.py 1번 2번 1번 2번   # 차례로 왕복 (마지막에 성공률·오차 요약)

장소는 /places/list (place_manager) 에서 받는다.
방향(yaw)이 없는 장소는 '출발 위치 → 목표' 방향을 도착 방향으로 쓴다.
도착 오차 = 목표 좌표와 AMCL 추정 위치(/amcl_pose)의 거리.
  ※ AMCL 기준이라 '로봇이 생각하는 오차'다. RFP ±0.25m 실측은 바닥 테이프로 따로 잰다.

Ctrl+C → 진행 중인 목표 취소 (로봇 정지).
"""

import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
TIMEOUT = 120.0     # 한 목표당 최대 [s]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoTo:

    def __init__(self):
        self.n = rclpy.create_node('goto_cli')
        self.places = None
        self.pose = None
        self.n.create_subscription(String, '/places/list',
                                   lambda m: setattr(self, 'places', json.loads(m.data)), LATCHED)
        self.n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, LATCHED)
        self.ac = ActionClient(self.n, NavigateToPose, 'navigate_to_pose')

    def _amcl(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def wait_ready(self):
        end = time.time() + 10
        while time.time() < end and (self.places is None or self.pose is None):
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.places is None:
            sys.exit('✗ /places/list 없음 — place_manager 실행 중인지 확인')
        if self.pose is None:
            sys.exit('✗ /amcl_pose 없음 — RViz 2D Pose Estimate 로 위치를 먼저 찍을 것')
        if not self.ac.wait_for_server(timeout_sec=10):
            sys.exit('✗ navigate_to_pose 액션 없음 — Nav2 실행 중인지 확인')

    def go(self, name):
        if name not in self.places:
            print(f"✗ '{name}' 없음 (있는 것: {', '.join(self.places)})")
            return None
        p = self.places[name]
        sx, sy, _ = self.pose
        yaw = p['yaw'] if p.get('yaw') is not None else math.atan2(p['y'] - sy, p['x'] - sx)

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.n.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = p['x'], p['y']
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal.pose = ps

        print(f"→ {name} ({p['x']:.2f}, {p['y']:.2f}) 출발", flush=True)
        t0 = time.time()
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=5)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('  ✗ 목표 거부됨')
            return False
        res = gh.get_result_async()
        try:
            while not res.done():
                rclpy.spin_once(self.n, timeout_sec=0.1)
                if time.time() - t0 > TIMEOUT:
                    gh.cancel_goal_async()
                    print(f'  ✗ {TIMEOUT:.0f}초 초과 — 취소')
                    return False
        except KeyboardInterrupt:
            c = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self.n, c, timeout_sec=3)
            print('\n  ■ 취소 (로봇 정지)')
            raise

        # 도착 직후 AMCL 이 한 번 더 갱신되도록 잠깐 대기
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        x, y, th = self.pose
        err = math.hypot(x - p['x'], y - p['y'])
        dyaw = abs(math.degrees(math.atan2(math.sin(th - yaw), math.cos(th - yaw))))
        ok = res.result().status == GoalStatus.STATUS_SUCCEEDED
        mark = '✓ 도착' if ok else f'✗ 실패(status {res.result().status})'
        print(f'  {mark}  {time.time() - t0:.1f}s  오차 {err:.3f} m  방향차 {dyaw:.0f}°', flush=True)
        return ok, err


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    rclpy.init()
    g = GoTo()
    g.wait_ready()
    results = []
    try:
        for name in sys.argv[1:]:
            r = g.go(name)
            if r is None:
                break
            results.append(r if isinstance(r, tuple) else (False, float('nan')))
    except KeyboardInterrupt:
        pass
    if len(results) > 1:
        oks = [e for ok, e in results if ok]
        print(f'== {len(oks)}/{len(results)} 성공', end='')
        if oks:
            print(f'  오차 평균 {sum(oks) / len(oks):.3f} m, 최대 {max(oks):.3f} m'
                  f'  (≤0.25m: {sum(e <= 0.25 for e in oks)}/{len(results)})')
        else:
            print()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
