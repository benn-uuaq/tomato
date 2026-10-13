#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
토마토 측정 결과 수신 클라이언트 예제 (상대 시스템 측)

ROS 2가 필요 없습니다. 파이썬 표준 라이브러리만 사용합니다.
같은 방식으로 C#, C++, Java 등 어떤 언어로도 구현할 수 있습니다.
"""

import json
import socket

HOST = '192.168.1.10'    # 중계 노드가 실행 중인 PC의 IP
PORT = 9000


def main():
    sock = socket.create_connection((HOST, PORT))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f'접속 완료: {HOST}:{PORT}')

    # 촬영 요청
    sock.sendall(b'{"cmd": "capture"}\n')
    print('촬영 요청 전송')

    buf = b''
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            print('연결이 끊겼습니다.')
            break
        buf += chunk

        # 줄 단위로 잘라서 처리 (TCP는 메시지 경계를 보장하지 않음)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            if not line.strip():
                continue
            msg = json.loads(line.decode('utf-8'))
            handle(msg)


def handle(msg):
    kind = msg.get('type')

    if kind == 'hello':
        print(f"서버 연결 확인 (프로토콜 v{msg['protocol']})")

    elif kind == 'result':
        n = msg['count']
        if n == 0:
            print('촬영 완료 — 검출된 토마토 없음')
            return

        print(f"촬영 완료 — 토마토 {n}개 (기준 좌표계: {msg['frame_id']})")
        for t in msg['tomatoes']:
            p = t['position']
            # 단위: position 은 m, weight_g 는 g, height_m 은 m
            print(f"  [{t['index']}] "
                  f"x={p['x']*1000:7.1f} mm  y={p['y']*1000:7.1f} mm  z={p['z']*1000:7.1f} mm  "
                  f"무게 {t['weight_g']:.1f} g  높이 {t['height_m']*1000:.1f} mm")

            # 여기서 로봇 좌표계로 변환한 뒤 이동 명령을 생성합니다.
            # target = camera_to_robot(p['x'], p['y'], p['z'])

    elif kind == 'error':
        print(f"오류: {msg['message']}")


if __name__ == '__main__':
    main()
