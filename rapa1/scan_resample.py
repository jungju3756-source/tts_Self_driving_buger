#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_resample.py  —  LiDAR 스캔을 고정 개수로 재샘플 (라파1, SLAM 용)

LDS-02(ld08_driver) 는 한 바퀴마다 점 개수가 205~209 개로 들쭉날쭉하다.
slam_toolbox(Karto) 는 처음 받은 스캔의 개수를 기억하고, 개수가 다른 스캔은
"LaserRangeScan contains N range readings, expected M" 경고와 함께 버린다.
→ 지도가 거의 안 그려진다.

그래서 /scan 을 1° 간격 360 칸으로 다시 나눠 /scan_fixed 로 내보낸다.
칸 안에 점이 여러 개면 가장 가까운 값(장애물 쪽, 보수적)을 쓴다.

    /scan (205~209 점)  ──►  scan_resample  ──►  /scan_fixed (항상 360 점)

실행:
    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=40
    python3 ~/tts_Self_driving_buger/rapa1/scan_resample.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

BINS = 360
INC = 2.0 * math.pi / BINS
RANGE_MIN = 0.16     # LDS-02 사양 최소 거리 [m]
RANGE_MAX = 8.0      # LDS-02 사양 최대 거리 [m]


class ScanResample(Node):

    def __init__(self):
        super().__init__('scan_resample')
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)

    def _scan(self, m):
        out = [math.inf] * BINS
        for i, r in enumerate(m.ranges):
            if not (RANGE_MIN <= r <= RANGE_MAX):
                continue
            a = (m.angle_min + i * m.angle_increment) % (2.0 * math.pi)
            b = int(a / INC) % BINS
            if r < out[b]:
                out[b] = r

        s = LaserScan()
        s.header = m.header
        s.angle_min = 0.0
        s.angle_max = (BINS - 1) * INC
        s.angle_increment = INC
        s.scan_time = m.scan_time
        s.time_increment = m.scan_time / BINS if m.scan_time > 0 else 0.0
        s.range_min = RANGE_MIN
        s.range_max = RANGE_MAX
        s.ranges = out
        self.pub.publish(s)


def main():
    rclpy.init()
    node = ScanResample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
