#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
place.py  —  place_manager 에 명령 보내고 결과 받기 (라파1·WSL 어디서든)

    python3 place.py save 정수기        # RViz 에서 방금 찍은 점 저장
    python3 place.py here 충전기        # 로봇 현재 위치 저장
    python3 place.py delete 정수기
    python3 place.py rename 정수기 물
    python3 place.py alias 정수기 물마시는곳
    python3 place.py list
"""

import json
import math
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    rclpy.init()
    n = rclpy.create_node('place_cli')
    got = []
    n.create_subscription(String, '/places/event', lambda m: got.append(json.loads(m.data)), 10)
    pub = n.create_publisher(String, '/places/cmd', QoSProfile(
        depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE))

    end = time.time() + 5.0                      # place_manager 를 찾을 때까지 대기
    while pub.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        sys.exit('✗ place_manager 가 안 보임 (라파1 에서 실행 중인지 확인)')
    time.sleep(0.3)                              # 이벤트 구독이 연결될 시간
    pub.publish(String(data=' '.join(sys.argv[1:])))

    end = time.time() + 3.0
    while not got and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not got:
        sys.exit('✗ 응답 없음')
    r = got[0]
    if not r['ok']:
        sys.exit(f"✗ {r['error']}")
    if r['cmd'] == 'list':
        for name, p in r['places'].items():
            yaw = '-' if p.get('yaw') is None else f"{math.degrees(p['yaw']):.0f}°"
            al = f"  별칭: {', '.join(p['aliases'])}" if p.get('aliases') else ''
            print(f"  {name:10s} ({p['x']:6.2f}, {p['y']:6.2f})  {yaw:>5s}{al}")
        print(f"  — {len(r['places'])}개")
    else:
        print('✓', json.dumps({k: v for k, v in r.items() if k not in ('ok',)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
