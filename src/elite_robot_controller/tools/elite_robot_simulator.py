#!/usr/bin/env python3
"""Elite CS612 없이 elite_robot_controller를 시험하기 위한 가짜 로봇.

29999(Dashboard), 30001(Primary), Modbus TCP 세 채널을 흉내 낸다.
레지스터 주소는 `config/modbus_registers.json`을 그대로 읽으므로 노드와
시뮬레이터가 서로 다른 주소를 쓰는 일이 없다.

## 실행

```bash
python3 tools/elite_robot_simulator.py
```

기본으로 127.0.0.1:29999 / 30001 / 5502 에서 대기한다. 502는 리눅스에서
권한이 필요해 기본값을 5502로 뒀다. 노드는 다음처럼 띄운다.

```bash
ros2 run elite_robot_controller robot_control_node --ros-args \\
    -p robot_ip:=127.0.0.1 -p modbus_port:=5502
```

## 흉내 내는 동작

- **연결**: 세 포트 모두 접속을 받아준다. 여러 번 연결·해제해도 된다.
- **Dashboard 명령**: robotMode/status/robotControl -on|off/brakeRelease/
  play/pause/stop 에 짧은 텍스트로 응답한다. stop 은 조그 속도도 0으로 만든다.
- **조그**: 30001 로 들어오는 `speedj(...)`/`speedl(...)` 스크립트를 읽어
  해당 속도로 관절(73~78)과 TCP(384~389) 레지스터를 계속 갱신한다.
  `stop` 명령이나 연결 해제로 멈춘다. 목표 속도까지 도달하지 않으면
  기존 값을 계속 쓰지 않도록 처리한다.
- **홈 이동**: `movej(...)` 스크립트를 읽으면 짧은 지연 후 관절 레지스터를
  목표값으로 옮긴다.
- **쓰기 레지스터**: UI가 306(작업 속도), 307(속도 비율), 308(pose_src),
  310~321(기준 위치)에 쓴 값을 그대로 저장하고 돌려준다.
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import struct
import sys
import threading
import time

# 소스 트리에서 바로 실행해도 패키지를 찾도록 경로를 보정한다.
_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from elite_robot_controller import register_map  # noqa: E402
from elite_robot_controller.tomato import geometry  # noqa: E402
import collections  # noqa: E402
import math  # noqa: E402


def to_unsigned16(value: int) -> int:
    return value & 0xFFFF


def to_signed16(value: int) -> int:
    value &= 0xFFFF
    return value - 0x10000 if value > 0x7FFF else value


class RegisterBank:
    """레지스터 값을 보관하고 잠금으로 여러 스레드의 접근을 보호한다."""

    def __init__(self, registers: register_map.RegisterMap):
        self.registers = registers
        self._lock = threading.Lock()
        self._values: dict[int, int] = {}
        self._seed_defaults()

    def _seed_defaults(self) -> None:
        defaults = {
            "robot_mode": [7],                          # RUNNING
            "control_method": [2],                       # 원격 제어
            "operation_mode": [0],                        # 자동
            "joint_position": [207, -1466, -1875, -1371, 1570, 207],
            "tcp_absolute": [6368, -473, 5810, 3141, 0, -1570],
            "tcp_zero_relative": [0, 0, 1000, 0, 0, 0],
        }
        for name, values in defaults.items():
            entry = self.registers.read_entry(name)
            if not entry.available:
                continue
            for offset, value in enumerate(values):
                self._values[entry.address + offset] = value
        for name in self.registers.write.keys():
            entry = self.registers.write_entry(name)
            for offset in range(entry.count):
                self._values.setdefault(entry.address + offset, 0)

    def read(self, address: int, count: int) -> list[int]:
        with self._lock:
            return [self._values.get(address + i, 0) for i in range(count)]

    def write(self, address: int, value: int) -> None:
        with self._lock:
            self._values[address] = value

    def write_many(self, address: int, values: list[int]) -> None:
        with self._lock:
            for offset, value in enumerate(values):
                self._values[address + offset] = value

    def entry_by_address(self, target: str) -> register_map.RegisterEntry:
        return self.registers.read_entry(target)


class ModbusServer(threading.Thread):
    """FC3(읽기)/FC6·FC16(쓰기)만 지원하는 최소 Modbus TCP 서버."""

    def __init__(self, bank: RegisterBank, host: str, port: int):
        super().__init__(daemon=True)
        self.bank = bank
        self.host, self.port = host, port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(4)

    def run(self) -> None:
        print(f"[Modbus] {self.host}:{self.port} 에서 대기")
        while True:
            conn, addr = self._sock.accept()
            threading.Thread(target=self._handle, args=(conn, addr), daemon=True).start()

    def _handle(self, conn: socket.socket, addr) -> None:
        with conn:
            while True:
                header = self._recv_exact(conn, 7)
                if header is None:
                    return
                tx_id, proto_id, length, unit_id = struct.unpack(">HHHB", header)
                pdu = self._recv_exact(conn, length - 1)
                if pdu is None:
                    return
                response_pdu = self._handle_pdu(pdu)
                resp_header = struct.pack(">HHHB", tx_id, proto_id, len(response_pdu) + 1, unit_id)
                conn.sendall(resp_header + response_pdu)

    @staticmethod
    def _recv_exact(conn: socket.socket, size: int) -> bytes | None:
        buf = b""
        while len(buf) < size:
            chunk = conn.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _handle_pdu(self, pdu: bytes) -> bytes:
        func = pdu[0]
        if func == 0x03:  # Read Holding Registers
            address, count = struct.unpack(">HH", pdu[1:5])
            values = [to_unsigned16(v) for v in self.bank.read(address, count)]
            payload = struct.pack(">B", count * 2) + b"".join(struct.pack(">H", v) for v in values)
            return struct.pack(">B", 0x03) + payload
        if func == 0x06:  # Write Single Register
            address, value = struct.unpack(">HH", pdu[1:5])
            self.bank.write(address, to_signed16(value))
            return pdu[:5]
        if func == 0x10:  # Write Multiple Registers
            address, count, byte_count = struct.unpack(">HHB", pdu[1:6])
            raw = pdu[6:6 + byte_count]
            values = [to_signed16(v) for v, in struct.iter_unpack(">H", raw)]
            self.bank.write_many(address, values)
            return struct.pack(">BHH", 0x10, address, count)
        # 지원하지 않는 기능 코드: 예외 응답(0x01 illegal function)
        return struct.pack(">BB", func | 0x80, 0x01)


class ScanTaskSim(threading.Thread):
    """`play` 를 받으면 로봇 태스크(dusan_v1)의 한 셀 스캔을 흉내 낸다.

    실제 태스크가 레지스터 290~298에 쓰는 진행 상태를 같은 순서로 채운다.
    운영 UI의 격자 순회가 이 값으로 셀 완료를 판정하므로, 이게 없으면
    첫 셀에서 영영 넘어가지 못한다.

    ㄹ자 경로를 실제로 그리지는 않고 단계와 줄 수만 흉내 낸다. 순회 로직을
    검증하는 게 목적이지 로봇 기구학을 재현하는 게 아니다.
    """

    # 레지스터 290~298. 로봇 태스크의 dus_*.script 와 같은 뜻이다.
    STATE, ROW_IDX, ROWS, ALIVE = 290, 291, 292, 293
    ZERO_OK, FINISHED, PITCH, PROGRESS = 294, 295, 296, 298
    # 작업 영역(256~259). 태스크가 읽어 ㄹ자 줄 수를 계산한다.
    APP_WIDTH, APP_HEIGHT, SCAN_H, OVERLAP = 256, 257, 258, 259

    STEP_SECONDS = 0.35

    def __init__(self, bank: RegisterBank):
        super().__init__(daemon=True)
        self.bank = bank
        self._start_requested = threading.Event()
        self._abort = threading.Event()
        self._alive_count = 0

    def play(self) -> None:
        """제로점에서 프로그램을 다시 재생한다."""
        self._abort.set()          # 돌고 있으면 먼저 접는다
        self._start_requested.set()

    def stop(self) -> None:
        self._abort.set()
        self._start_requested.clear()
        self._set(self.STATE, 0)
        self._set(self.FINISHED, 0)

    def _set(self, address: int, value: int) -> None:
        self.bank.write(address, int(value))

    def _tick(self, seconds: float) -> bool:
        """지정한 시간만큼 대기한다. 중단 요청이 오면 False."""
        end = time.time() + seconds
        while time.time() < end:
            if self._abort.is_set():
                return False
            self._alive_count = (self._alive_count + 1) % 30000
            self._set(self.ALIVE, self._alive_count)
            time.sleep(0.05)
        return True

    def _planned_rows(self) -> int:
        """작업 영역으로 ㄹ자 줄 수를 센다. 로봇 태스크와 같은 규칙이다."""
        height = self.bank.read(self.APP_HEIGHT, 1)[0]
        scan_h = self.bank.read(self.SCAN_H, 1)[0] or 150
        overlap = self.bank.read(self.OVERLAP, 1)[0]
        pitch = max(scan_h - overlap, 1)
        rows = 1
        while (rows - 1) * pitch + scan_h < height:
            rows += 1
            if rows > 200:      # 값이 이상해도 무한 루프에 빠지지 않는다
                break
        return rows

    def run(self) -> None:
        while True:
            self._start_requested.wait()
            self._start_requested.clear()
            self._abort.clear()
            self._run_once()

    def _run_once(self) -> None:
        rows = self._planned_rows()
        pitch = max(
            self.bank.read(self.SCAN_H, 1)[0] - self.bank.read(self.OVERLAP, 1)[0], 1
        )
        self._set(self.FINISHED, 0)
        self._set(self.ROWS, rows)
        self._set(self.PITCH, pitch)
        self._set(self.PROGRESS, 0)
        self._set(self.ZERO_OK, 0)

        # 탐색 → probe → 원점 복귀 (state 1, 2, 3)
        for state in (1, 2, 3):
            self._set(self.STATE, state)
            if not self._tick(self.STEP_SECONDS):
                return
        self._set(self.ZERO_OK, 1)

        # ㄹ자 스캔 (state 4). 줄마다 row_idx 와 진행률을 올린다.
        self._set(self.STATE, 4)
        for row in range(rows):
            self._set(self.ROW_IDX, row)
            self._set(self.PROGRESS, int((row + 1) / rows * 100))
            if not self._tick(self.STEP_SECONDS):
                return

        # 피니시 (state 5). 운영 UI 는 이 값으로 셀 완료를 판정한다.
        self._set(self.STATE, 5)
        self._set(self.FINISHED, 1)
        self._set(self.PROGRESS, 100)
        print(f"[Task] 셀 스캔 완료 ({rows} 행)")


class DashboardServer(threading.Thread):
    """29999 텍스트 명령 서버."""

    def __init__(self, bank: RegisterBank, motion: "MotionSim",
                 task: "ScanTaskSim", host: str, port: int):
        super().__init__(daemon=True)
        self.bank, self.motion, self.task = bank, motion, task
        self.host, self.port = host, port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(4)

    def run(self) -> None:
        print(f"[Dashboard 29999] {self.host}:{self.port} 에서 대기")
        while True:
            conn, addr = self._sock.accept()
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn: socket.socket) -> None:
        with conn:
            try:
                conn.sendall(b"Connected: Elite Robot Dashboard Simulator\n")
                buf = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        return
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._respond(conn, line.decode("utf-8", "ignore").strip())
            except (ConnectionError, OSError):
                return

    def _respond(self, conn: socket.socket, command: str) -> None:
        replies = {
            "robotMode": "Robotmode: RUNNING",
            "status": "Status: RUNNING",
            "robotControl -on": "Powering on",
            "robotControl -off": "Powering off",
            "brakeRelease": "Brake releasing",
            "pause": "Pausing program",
        }
        if command == "play":
            # 실제 로봇처럼 제로점에서 태스크를 처음부터 다시 돌린다.
            self.task.play()
            reply = "Starting program"
        elif command == "stop":
            self.motion.stop_all()
            self.task.stop()
            reply = "Stopping program"
        else:
            reply = replies.get(command, f"ok: {command}")
        print(f"[Dashboard] {command!r} -> {reply!r}")
        conn.sendall((reply + "\n").encode("utf-8"))


class MotionSim:
    """로봇 움직임 흉내 — 조그 속도, 직선 이동, 관절 이동(경유점 차례대로).

    관절 이동 중에는 정기구학으로 TCP 를 계산해 레지스터에 쓴다(회전은 RPY,
    CS616 이 보고하는 표기와 같게). 그래야 시퀀스가 관절 이동 뒤 TCP 기준
    이동을 할 때 실제와 같은 조건이 된다. 로봇 모양은 UR10e 와 비슷한 DH 를
    쓴다(시험용 — 실제 CS616 치수는 아니다).

    `def ... end` 안의 이동 명령 여러 개는 **차례대로** 처리한다(대기열).
    """

    RATE_HZ = 20
    DH_A = [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0]
    DH_D = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655]
    DH_ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]

    def __init__(self, bank: RegisterBank):
        self.bank = bank
        self._lock = threading.Lock()
        self._joint_velocity = [0.0] * 6   # rad/s (speedj)
        self._tcp_velocity = [0.0] * 6      # [m/s]*3 + [rad/s]*3 (speedl)
        self._joint_entry = bank.registers.read_entry("joint_position")
        self._tcp_entry = bank.registers.read_entry("tcp_absolute")
        # 관절은 rad, TCP 는 레지스터 카운트(0.1 mm / mrad) 로 들고 있는다.
        self._q = [v / 1000.0 for v in bank.read(self._joint_entry.address, 6)]
        self._tcp_pos = [float(v) for v in bank.read(self._tcp_entry.address, 6)]
        self._queue = collections.deque()
        self._active = None
        geometry.set_orientation_format("rpy")

    # ------------------------------------------------------------ 명령
    def set_joint_velocity(self, qd: list[float]) -> None:
        with self._lock:
            self._joint_velocity = list(qd)

    def set_tcp_velocity(self, xd: list[float]) -> None:
        with self._lock:
            self._tcp_velocity = list(xd)

    def stop_all(self) -> None:
        with self._lock:
            self._joint_velocity = [0.0] * 6
            self._tcp_velocity = [0.0] * 6
            self._queue.clear()
            self._active = None

    def queue_linear(self, pose_si: list[float], speed: float) -> None:
        self._queue_motion(("pose", list(pose_si), max(0.001, float(speed))))
        print(f"[Motion] 직선 목표 {[round(v, 4) for v in pose_si]} v={speed}")

    def queue_joints(self, joints_rad: list[float], speed: float) -> None:
        self._queue_motion(("joints", list(joints_rad), max(0.01, float(speed))))
        print(f"[Motion] 관절 목표 {[round(v, 3) for v in joints_rad]} v={speed}")

    def _queue_motion(self, motion) -> None:
        with self._lock:
            self._queue.append(motion)

    # ------------------------------------------------------------ 계산
    def _fk_counts(self, q):
        rot, pos = geometry.fk_dh(q, self.DH_A, self.DH_D, self.DH_ALPHA)
        rpy = geometry.matrix_to_rpy(rot)
        scale = self.bank.registers.scales_for(self._tcp_entry)
        values = [pos[0] * 1000.0, pos[1] * 1000.0, pos[2] * 1000.0,
                  rpy[0] * 1000.0, rpy[1] * 1000.0, rpy[2] * 1000.0]
        return [values[i] / scale[i] for i in range(6)]

    def _step(self, dt: float, tcp_scale: list) -> None:
        """대기열의 이동을 한 주기만큼 진행한다. 락 안에서 부른다."""
        if self._active is None and self._queue:
            kind, target, speed = self._queue.popleft()
            if kind == "pose":
                target = [(target[i] * 1000.0) / tcp_scale[i] for i in range(6)]
            self._active = (kind, target, speed)
        if self._active is None:
            return
        kind, target, speed = self._active
        if kind == "joints":
            delta = [target[i] - self._q[i] for i in range(6)]
            biggest = max(abs(d) for d in delta)
            step = speed * dt
            if biggest <= step:
                self._q = list(target)
                self._active = None
            else:
                self._q = [self._q[i] + delta[i] * (step / biggest) for i in range(6)]
            self._tcp_pos = self._fk_counts(self._q)
            return
        # 직선: 위치는 설정 속도로, 회전은 바로 맞춘다(시퀀스는 자세를 안 바꾼다).
        delta = [target[i] - self._tcp_pos[i] for i in range(3)]
        dist = math.sqrt(sum(d * d for d in delta))
        step = (speed * 1000.0 / tcp_scale[0]) * dt
        if dist <= step:
            self._tcp_pos = list(target)
            self._active = None
        else:
            for i in range(3):
                self._tcp_pos[i] += delta[i] * (step / dist)
            for i in range(3, 6):
                self._tcp_pos[i] = target[i]

    def run_forever(self) -> None:
        dt = 1.0 / self.RATE_HZ
        tcp_scale = self.bank.registers.scales_for(self._tcp_entry)
        while True:
            time.sleep(dt)
            with self._lock:
                self._step(dt, tcp_scale)
                jv, tv = list(self._joint_velocity), list(self._tcp_velocity)
                for i in range(6):
                    self._q[i] += jv[i] * dt
                    self._tcp_pos[i] += (tv[i] * 1000.0 / tcp_scale[i]) * dt
                joint_raw = [int(round(v * 1000.0)) for v in self._q]
                tcp_raw = [int(round(v)) for v in self._tcp_pos]
            self.bank.write_many(self._joint_entry.address, joint_raw)
            self.bank.write_many(self._tcp_entry.address, tcp_raw)


_SPEEDJ_RE = re.compile(r"speedj\((\[[^\]]*\])")
_SPEEDL_RE = re.compile(r"speedl\((\[[^\]]*\])")
# 픽앤플레이스가 쓰는 세 형태. Elite 의 pose 는 p[...] 가 아니라 그냥 리스트다.
_MOVEL_RE = re.compile(r"movel\((\[[^\]]*\])(.*)")
_MOVEJ_IK_RE = re.compile(r"movej\(get_inverse_kin\((\[[^\]]*\])\)(.*)")
_MOVEJ_RE = re.compile(r"movej\((\[[^\]]*\])(.*)")
_SPEED_RE = re.compile(r"v\s*=\s*([0-9.]+)")


class PrimaryServer(threading.Thread):
    """30001 스크립트 수신 서버. 조그와 홈 이동 스크립트를 해석한다."""

    def __init__(self, motion: MotionSim, host: str, port: int):
        super().__init__(daemon=True)
        self.motion = motion
        self.host, self.port = host, port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(4)

    def run(self) -> None:
        print(f"[Primary 30001] {self.host}:{self.port} 에서 대기")
        while True:
            conn, addr = self._sock.accept()
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn: socket.socket) -> None:
        with conn:
            buf = ""
            try:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        return
                    buf += chunk.decode("utf-8", "ignore")
                    # 줄 단위로 처리한다. 버퍼 전체를 다시 훑으면 지난
                    # 명령이 새 명령이 올 때마다 다시 걸린다.
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self._scan(line)
            except (ConnectionError, OSError):
                self.motion.stop_all()
                return

    def _scan(self, line: str) -> None:
        def speed(rest, default):
            m = _SPEED_RE.search(rest)
            return float(m.group(1)) if m else default

        m = _MOVEL_RE.search(line)
        if m:
            self.motion.queue_linear(eval(m.group(1)), speed(m.group(2), 0.1))  # noqa: S307
            return
        m = _MOVEJ_IK_RE.search(line)
        if m:
            # 역기구학은 흉내 내지 않는다 — TCP 를 목표로 직선 이동시킨다.
            self.motion.queue_linear(eval(m.group(1)), 0.25)  # noqa: S307
            return
        m = _MOVEJ_RE.search(line)
        if m:
            self.motion.queue_joints(eval(m.group(1)), speed(m.group(2), 1.0))  # noqa: S307
            return
        m = _SPEEDJ_RE.search(line)
        if m:
            self.motion.set_joint_velocity(eval(m.group(1)))  # noqa: S307
        m = _SPEEDL_RE.search(line)
        if m:
            self.motion.set_tcp_velocity(eval(m.group(1)))  # noqa: S307


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--dash-port", type=int, default=29999)
    parser.add_argument("--primary-port", type=int, default=30001)
    parser.add_argument("--modbus-port", type=int, default=5502)
    args = parser.parse_args()

    registers = register_map.load()
    bank = RegisterBank(registers)
    motion = MotionSim(bank)
    task = ScanTaskSim(bank)
    task.start()

    ModbusServer(bank, args.host, args.modbus_port).start()
    DashboardServer(bank, motion, task, args.host, args.dash_port).start()
    PrimaryServer(motion, args.host, args.primary_port).start()

    print(
        f"실행 준비 완료. 다음처럼 노드를 띄우세요:\n"
        f"  ros2 run elite_robot_controller robot_control_node --ros-args "
        f"-p robot_ip:={args.host} -p modbus_port:={args.modbus_port}\n"
        f"Ctrl+C 로 종료합니다."
    )
    try:
        motion.run_forever()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
