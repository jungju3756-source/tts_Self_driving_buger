#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wall_manager.py  —  가상 벽(선분) 저장 + Nav2 Keepout 마스크 발행

    python3 ~/tts_Self_driving_buger/rapa1/wall_manager.py

웹 페이지에서 지도 위에 그린 선분을 '로봇이 넘어가면 안 되는 벽' 으로 만든다.

    /walls/cmd (String, JSON)  ─► wall_manager ─► /keepout_filter_mask (OccupancyGrid)
                                       │        └► /costmap_filter_info (Nav2 KeepoutFilter 가 구독)
                                       └► /walls/list (String, JSON, transient local)

명령 (JSON 한 줄):
    {"cmd": "add", "x1": 0.5, "y1": 0.2, "x2": 1.5, "y2": 0.2}
    {"cmd": "delete", "id": 3}
    {"cmd": "clear"}

마스크의 벽 두께: 선분 양쪽으로 half_width (기본 0.20 m).
Keepout 필터는 inflation 이후에 적용돼 부풀려지지 않으므로, 로봇 반지름(0.22)만큼
두껍게 칠해야 로봇 몸이 벽을 넘지 않는다 (경로 계획은 로봇 중심만 본다).

저장:  ~/maps/walls.yaml  (map 좌표계)
"""

import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
MASK_TOPIC = '/keepout_filter_mask'


class WallManager(Node):

    def __init__(self):
        super().__init__('wall_manager')
        self.declare_parameter('walls_file', os.path.expanduser('~/maps/walls.yaml'))
        self.declare_parameter('half_width', 0.20)
        self.path = self.get_parameter('walls_file').value
        self.half = self.get_parameter('half_width').value

        self.walls = self._load()
        self.map_info = None

        self.pub_mask = self.create_publisher(OccupancyGrid, MASK_TOPIC, LATCHED)
        self.pub_info = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info', LATCHED)
        self.pub_list = self.create_publisher(String, '/walls/list', LATCHED)
        self.pub_event = self.create_publisher(String, '/walls/event', 10)
        self.create_subscription(OccupancyGrid, '/map', self._map, LATCHED)
        self.create_subscription(String, '/walls/cmd', self._cmd, 10)

        info = CostmapFilterInfo()
        info.header.frame_id = 'map'
        info.type = 0                     # keepout
        info.filter_mask_topic = MASK_TOPIC
        info.base, info.multiplier = 0.0, 1.0
        self.pub_info.publish(info)
        self._publish_list()
        self.get_logger().info(f'벽 {len(self.walls)}개 로드, 지도 기다리는 중')

    # ---------- 저장소 ----------
    def _load(self):
        if not os.path.exists(self.path):
            return []
        with open(self.path, encoding='utf-8') as f:
            return (yaml.safe_load(f) or {}).get('walls', [])

    def _save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            yaml.safe_dump({'frame': 'map', 'walls': self.walls}, f, sort_keys=False)
        os.replace(tmp, self.path)

    # ---------- 입력 ----------
    def _map(self, m):
        first = self.map_info is None
        self.map_info = m.info
        self._publish_mask()
        if first:
            self.get_logger().info(f'지도 {m.info.width}x{m.info.height} 받음 → 마스크 발행')

    def _cmd(self, m):
        try:
            c = json.loads(m.data)
            cmd = c.get('cmd')
            if cmd == 'add':
                pts = [float(c[k]) for k in ('x1', 'y1', 'x2', 'y2')]
                if math.hypot(pts[2] - pts[0], pts[3] - pts[1]) < 0.05:
                    raise ValueError('벽이 너무 짧음 (5 cm 미만)')
                wid = max([w['id'] for w in self.walls], default=0) + 1
                self.walls.append({'id': wid, 'x1': round(pts[0], 3), 'y1': round(pts[1], 3),
                                   'x2': round(pts[2], 3), 'y2': round(pts[3], 3),
                                   'saved': time.strftime('%Y-%m-%d %H:%M:%S')})
                res = {'id': wid}
            elif cmd == 'delete':
                wid = int(c['id'])
                before = len(self.walls)
                self.walls = [w for w in self.walls if w['id'] != wid]
                if len(self.walls) == before:
                    raise ValueError(f'벽 {wid} 없음')
                res = {'id': wid}
            elif cmd == 'clear':
                res = {'count': len(self.walls)}
                self.walls = []
            else:
                raise ValueError(f'모르는 명령: {cmd} (add/delete/clear)')
            self._save()
            self._publish_mask()
            self._publish_list()
            res.update(ok=True, cmd=cmd)
        except Exception as e:                    # 명령 하나 실패가 노드를 죽이면 안 됨
            res = {'ok': False, 'error': str(e)}
        self.pub_event.publish(String(data=json.dumps(res, ensure_ascii=False)))
        if res['ok']:
            self.get_logger().info(json.dumps(res, ensure_ascii=False))
        else:
            self.get_logger().warning(json.dumps(res, ensure_ascii=False))

    # ---------- 출력 ----------
    def _publish_list(self):
        self.pub_list.publish(String(data=json.dumps(
            {'walls': self.walls, 'half_width': self.half}, ensure_ascii=False)))

    def _publish_mask(self):
        if self.map_info is None:
            return
        info = self.map_info
        w, h, r = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        # 셀 중심 좌표 격자
        xs = ox + (np.arange(w) + 0.5) * r
        ys = oy + (np.arange(h) + 0.5) * r
        gx, gy = np.meshgrid(xs, ys)               # (h, w), 행 0 = 지도 아래쪽 (ROS 규약)
        mask = np.zeros((h, w), dtype=bool)
        for wl in self.walls:
            ax, ay, bx, by = wl['x1'], wl['y1'], wl['x2'], wl['y2']
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = np.clip(((gx - ax) * dx + (gy - ay) * dy) / L2, 0.0, 1.0)
            dist = np.hypot(gx - (ax + t * dx), gy - (ay + t * dy))
            mask |= dist <= self.half

        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info = info
        g.data = np.where(mask, 100, 0).astype(np.int8).flatten().tolist()
        self.pub_mask.publish(g)


def main():
    rclpy.init()
    node = WallManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
