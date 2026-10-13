#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
토마토 측정 결과 TCP 중계 노드

ROS 2를 사용하지 않는 상대 시스템에 측정 결과를 전달하기 위한 브릿지입니다.
이 노드가 TCP 서버로 동작하며, 상대 프로그램이 접속해 옵니다.

  [토마토 노드] --ROS2 토픽--> [이 노드] --TCP / JSON Lines--> [상대 프로그램]

프로토콜
  - 한 줄에 JSON 객체 하나, 줄바꿈(\n)으로 구분 (JSON Lines)
  - 문자 인코딩 UTF-8

수신 (상대 -> 이 노드)
  {"cmd": "capture"}          촬영 요청. /tomato/capture 를 발행합니다.
  {"cmd": "ping"}             연결 확인. {"type":"pong"} 으로 응답합니다.
  {"cmd": "gripper", "position": 3185}
                              그리퍼 목표 위치(0~4095). /set_pos 를 발행하고
                              {"type":"ack","cmd":"gripper","position":3185} 로 답합니다.
                              (그리퍼 USB 가 이 장비에 꽂혀 있어 로봇 PC 가 직접
                              못 움직이므로 여기를 거친다. protocol 2 에서 추가)

송신 (이 노드 -> 상대)
  {"type": "hello", "protocol": 1}
  {"type": "pong"}
  {"type": "result",
   "stamp": 1789538649.991,           측정 시각 (유닉스 초)
   "frame_id": "camera_color_optical_frame",
   "count": 2,
   "tomatoes": [
     {"index": 0,
      "position": {"x": 0.0193, "y": -0.0339, "z": 0.3035},   단위 m
      "weight_g": 60.2,                                        단위 g
      "height_m": 0.0386}                                      단위 m
   ]}

  count 가 0 이면 검출된 토마토가 없다는 뜻입니다 (촬영은 정상 완료).
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray, Int32, String
from geometry_msgs.msg import PoseArray

PROTOCOL_VERSION = 2   # 2: gripper 명령 추가 (1 과 하위 호환)


class TomatoTcpBridge(Node):
    def __init__(self):
        super().__init__('tomato_tcp_bridge')

        # ------------------------------------------------------------
        # 파라미터
        # ------------------------------------------------------------
        self.declare_parameter('host', '0.0.0.0')   # 모든 인터페이스에서 수신
        self.declare_parameter('port', 9000)
        self.declare_parameter('assemble_sec', 0.3)  # 세 토픽을 모으는 대기 시간
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.assemble_sec = float(self.get_parameter('assemble_sec').value)

        # ------------------------------------------------------------
        # ROS 2 연결
        # ------------------------------------------------------------
        self.create_subscription(
            PoseArray, '/tomato/center_positions', self.pose_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/tomato/estimated_weights', self.weight_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/tomato/heights', self.height_callback, 10)

        self.capture_pub = self.create_publisher(Empty, '/tomato/capture', 10)
        # 그리퍼(다이나믹셀) 목표 위치. gripper_dynamixel_node / dyn_test 가 받는다.
        self.declare_parameter('gripper_topic', '/set_pos')
        self.gripper_pub = self.create_publisher(
            Int32, str(self.get_parameter('gripper_topic').value), 10)

        # ------------------------------------------------------------
        # 수신 버퍼 (세 토픽이 따로 도착하므로 잠시 모읍니다)
        # ------------------------------------------------------------
        self.buf_poses = None
        self.buf_stamp = None
        self.buf_frame = ''
        self.buf_weights = []
        self.buf_heights = []
        self.assemble_timer = None

        # ------------------------------------------------------------
        # 소켓 서버
        # ------------------------------------------------------------
        self.conns = []                 # [(conn, addr), ...]
        self.conns_lock = threading.Lock()
        self.running = True

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(5)

        threading.Thread(target=self.accept_loop, daemon=True).start()

        # 지금 누가 붙어 있는지 1 초마다 알린다(Orin 패널이 "로봇 PC 연결됨" 을 보여 준다).
        self.status_pub = self.create_publisher(String, 'vision/bridge/server_status', 10)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'=== 토마토 TCP 중계 노드 시작 === {self.host}:{self.port} 대기 중')

    # ================================================================
    # 소켓 — 접속 수락
    # ================================================================
    def accept_loop(self):
        while self.running:
            try:
                conn, addr = self.server.accept()
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.conns_lock:
                self.conns.append((conn, addr))
            self.get_logger().info(f'클라이언트 접속: {addr[0]}:{addr[1]}')
            self.send_to(conn, {'type': 'hello', 'protocol': PROTOCOL_VERSION})
            threading.Thread(target=self.recv_loop, args=(conn, addr), daemon=True).start()

    # ================================================================
    # 소켓 — 명령 수신
    # ================================================================
    def recv_loop(self, conn, addr):
        buf = b''
        try:
            while self.running:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                # 줄 단위로 잘라서 처리 (메시지 경계 복원)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    self.handle_command(conn, line.decode('utf-8', 'replace').strip())
        except OSError:
            pass
        finally:
            self.drop_client(conn, addr)

    def handle_command(self, conn, line):
        if not line:
            return
        try:
            req = json.loads(line)
        except json.JSONDecodeError:
            self.send_to(conn, {'type': 'error', 'message': 'invalid json'})
            return

        cmd = req.get('cmd')
        if cmd == 'capture':
            self.capture_pub.publish(Empty())
            self.get_logger().info('촬영 요청 수신 -> /tomato/capture 발행')
        elif cmd == 'ping':
            self.send_to(conn, {'type': 'pong'})
        elif cmd == 'gripper':
            position = req.get('position')
            # 범위 밖 값으로 그리퍼가 끝까지 물리는 일을 막는다.
            if not isinstance(position, int) or not 0 <= position <= 4095:
                self.send_to(conn, {'type': 'error',
                                    'message': f'gripper position must be int 0~4095: {position!r}'})
                return
            self.gripper_pub.publish(Int32(data=position))
            self.send_to(conn, {'type': 'ack', 'cmd': 'gripper', 'position': position})
            self.get_logger().info(f'그리퍼 명령 수신 -> /set_pos {position}')
        else:
            self.send_to(conn, {'type': 'error', 'message': f'unknown cmd: {cmd}'})

    # ================================================================
    # 소켓 — 송신
    # ================================================================
    def send_to(self, conn, obj):
        try:
            conn.sendall((json.dumps(obj, ensure_ascii=False) + '\n').encode('utf-8'))
            return True
        except OSError:
            return False

    def broadcast(self, obj):
        payload = (json.dumps(obj, ensure_ascii=False) + '\n').encode('utf-8')
        dead = []
        with self.conns_lock:
            targets = list(self.conns)
        for conn, addr in targets:
            try:
                conn.sendall(payload)
            except OSError:
                dead.append((conn, addr))
        for conn, addr in dead:
            self.drop_client(conn, addr)

    def drop_client(self, conn, addr):
        with self.conns_lock:
            if (conn, addr) in self.conns:
                self.conns.remove((conn, addr))
                self.get_logger().info(f'클라이언트 해제: {addr[0]}:{addr[1]}')
        try:
            conn.close()
        except OSError:
            pass

    # ================================================================
    # ROS 2 콜백 — 세 토픽을 모아서 한 번에 전송
    # ================================================================
    def pose_callback(self, msg):
        self.buf_poses = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.buf_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.buf_frame = msg.header.frame_id
        self.schedule_emit()

    def weight_callback(self, msg):
        self.buf_weights = list(msg.data)
        self.schedule_emit()

    def height_callback(self, msg):
        self.buf_heights = list(msg.data)
        self.schedule_emit()

    def schedule_emit(self):
        """마지막 토픽이 도착하고 assemble_sec 뒤에 한 번만 전송합니다."""
        if self.assemble_timer is not None:
            self.assemble_timer.cancel()
            self.destroy_timer(self.assemble_timer)
        self.assemble_timer = self.create_timer(self.assemble_sec, self.emit_result)

    def emit_result(self):
        if self.assemble_timer is not None:
            self.assemble_timer.cancel()
            self.destroy_timer(self.assemble_timer)
            self.assemble_timer = None

        if self.buf_poses is None:
            return

        tomatoes = []
        for i, (x, y, z) in enumerate(self.buf_poses):
            tomatoes.append({
                'index': i,
                'position': {'x': round(x, 5), 'y': round(y, 5), 'z': round(z, 5)},
                'weight_g': round(self.buf_weights[i], 2) if i < len(self.buf_weights) else None,
                'height_m': round(self.buf_heights[i], 5) if i < len(self.buf_heights) else None,
            })

        self.broadcast({
            'type': 'result',
            'stamp': round(self.buf_stamp, 3) if self.buf_stamp else time.time(),
            'frame_id': self.buf_frame,
            'count': len(tomatoes),
            'tomatoes': tomatoes,
        })

        with self.conns_lock:
            n = len(self.conns)
        self.get_logger().info(f'결과 전송: 토마토 {len(tomatoes)}개 -> 클라이언트 {n}곳')

        self.buf_poses = None
        self.buf_weights = []
        self.buf_heights = []

    def publish_status(self):
        with self.conns_lock:
            clients = [f'{addr[0]}:{addr[1]}' for _, addr in self.conns]
        self.status_pub.publish(String(data=json.dumps(
            {'port': self.port, 'clients': clients}, ensure_ascii=False)))

    # ================================================================
    def shutdown(self):
        self.running = False
        try:
            self.server.close()
        except OSError:
            pass
        with self.conns_lock:
            for conn, _ in self.conns:
                try:
                    conn.close()
                except OSError:
                    pass
            self.conns.clear()


def main():
    rclpy.init()
    node = TomatoTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
