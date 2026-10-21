"""다이나믹셀 그리퍼(XD540-T270) 제어 노드.

`doosan_vision_grasp` 의 `dyn_test`(dxl_test.py)와 같은 일을 하되, 포트·ID·
토픽을 **파라미터로** 받는다. 워크스페이스 하나만 띄우면 로봇·비전·그리퍼가
모두 올라오게 하려고 이 패키지에 같이 둔다.

    ros2 run elite_robot_controller gripper_dynamixel_node \
        --ros-args -p device:=/dev/ttyUSB0

`/set_pos` (std_msgs/Int32) 로 목표 위치(0 ~ max_position, 기본 8000)를 받는다.
0 이 열림, 클수록 닫힘이다. 닫기·열기 위치는 작업 UI 의 [그리퍼] 칸에서 정한다.

**여러 바퀴 (확장 위치 모드)**: 4095 가 한 바퀴라 그보다 큰 값을 쓰려면 모터가
확장 위치 모드(Operating Mode 4)여야 한다. max_position 이 4095 보다 크면 시작할 때
모드를 확인하고 아니면 바꾼다(토크 OFF -> 모드 -> 토크 ON, 한양대 dxl_test 와 같다).
여러 바퀴를 기본 속도로 돌면 빠르므로 프로파일 속도·가속도도 정한다 — 이 값은
모터가 재부팅되면 지워지므로 시작할 때와 복구할 때마다 다시 쓴다.

**하드웨어 에러 복구** (한양대 dxl_test.py 2026-10 판과 같은 방식): 세게 물면
과부하 에러가 래치돼 모터가 멈춘다. 0.1 초마다 에러 상태를 읽어, 걸려 있으면
재부팅 -> 토크 ON -> 마지막 목표 재전송을 한다. 명령 하나당 최대 3 번까지만
(무한 재부팅 방지) — 새 명령이 오면 다시 센다.

  재부팅하면 모터는 **몇 바퀴째였는지를 잊고** 현재 위치를 한 바퀴 안(0~4095)으로
  다시 센다. 그대로 마지막 목표(예: 6000)를 다시 보내면 한 바퀴를 더 조인다. 그래서
  재부팅 직전 위치와 비교해 잃어버린 바퀴 수만큼 보정(turn_offset)한 뒤 보낸다.
  전원을 껐다 켰을 때도 같은 일이 생긴다 — 마지막 위치를 파일(state_file)에 남겨 두고,
  켤 때 위치가 그 값과 "바퀴 수만 다르면" 같은 방법으로 보정한다. 바퀴 수 말고도
  다르면(누가 손이나 다른 프로그램으로 움직였으면) 보정하지 않고 경고만 남긴다.

`dynamixel_sdk` 가 없거나 포트를 못 열면 노드는 죽지 않고 경고만 남긴다 —
그리퍼 없이 로봇 동작만 시험하는 경우가 많기 때문이다.
"""

from __future__ import annotations

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32

from .pick_config import declare_loose

#: XD540-T270 제어 테이블 주소.
ADDR_OPERATING_MODE = 11     # 1바이트(EEPROM): 3 = 위치(한 바퀴), 4 = 확장 위치(여러 바퀴)
ADDR_TORQUE_ENABLE = 64      # 1바이트: 1 = On, 0 = Off
ADDR_HARDWARE_ERROR_STATUS = 70   # 1바이트: 하드웨어 에러 래치 상태
ADDR_PROFILE_ACCELERATION = 108   # 4바이트(RAM): 재부팅하면 0(최대)으로 돌아간다
ADDR_PROFILE_VELOCITY = 112       # 4바이트(RAM): 〃
ADDR_GOAL_POSITION = 116     # 4바이트: 목표 위치
ADDR_PRESENT_POSITION = 132  # 4바이트: 현재 위치

MODE_EXTENDED_POSITION = 4
TICKS_PER_REV = 4096
#: 전원을 켰을 때 "바퀴 수만 달라졌다" 고 볼 허용 오차 [tick].
TURN_MATCH_TICKS = 200

#: Hardware Error Status 비트별 의미
HW_ERROR_BITS = {0: "입력 전압 에러", 2: "과열 에러", 3: "엔코더 에러",
                 4: "전기적 충격 에러", 5: "과부하(Overload) 에러"}
MAX_RECOVERY_TRY = 3


def turns_lost(before: int, now: int) -> int:
    """재부팅·전원 재투입으로 잃어버린 바퀴 수 — 보정량은 이 값 x 4096."""
    return int(round((int(before) - int(now)) / TICKS_PER_REV))


def same_but_turns(before: int, now: int, tolerance: int = TURN_MATCH_TICKS) -> bool:
    """두 위치가 바퀴 수만 다르고 한 바퀴 안의 자리는 같은지."""
    rest = (int(before) - int(now)) % TICKS_PER_REV
    return min(rest, TICKS_PER_REV - rest) <= tolerance


class GripperDynamixelNode(Node):

    def __init__(self):
        super().__init__("gripper_dynamixel")
        self.declare_parameter("device", "/dev/ttyUSB0")
        declare_loose(self, "baudrate", 57600)
        declare_loose(self, "dxl_id", 1)
        declare_loose(self, "protocol", 2.0)
        self.declare_parameter("command_topic", "/set_pos")
        #: 목표 위치 허용 범위. 범위를 벗어난 값은 버린다(잘못된 명령으로
        #: 그리퍼가 끝까지 물리는 것을 막는다). 4095 를 넘으면 확장 위치 모드로 쓴다.
        declare_loose(self, "min_position", 0)
        declare_loose(self, "max_position", 8000)
        #: 프로파일 속도(x0.229 rpm)·가속도(x214.577 rev/min^2). 0 이면 건드리지 않는다.
        declare_loose(self, "profile_velocity", 150)
        declare_loose(self, "profile_acceleration", 50)
        #: 마지막 위치를 남겨 두는 파일 — 전원을 껐다 켰을 때 바퀴 수를 되찾는 데 쓴다.
        self.declare_parameter("state_file", "~/.tomato_pick/gripper_state.txt")

        self.port = None
        self.packet = None
        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.state_file = os.path.expanduser(str(self.get_parameter("state_file").value))
        #: 모터가 잊은 바퀴 수 보정 [tick]. 우리 쪽 위치 = 모터 위치 + turn_offset.
        self.turn_offset = 0
        self.last_present = None       # 마지막으로 읽은 위치(우리 쪽 값)
        self._saved_present = None     # 파일에 마지막으로 적은 값
        self._read_failed = False
        self.last_goal = None          # 복구 뒤 다시 보낼 마지막 목표(우리 쪽 값)
        self.recovery_count = 0
        self._tick = 0
        self._open_port()

        topic = str(self.get_parameter("command_topic").value)
        self.create_subscription(Int32, topic, self.cb_set_position, 10)
        self.get_logger().info(f"[gripper] 목표 위치 대기: {topic} (Int32)")

        # 모터가 실제로 어디 있는지 알린다(패널이 "열림/닫힘" 과 "응답 없음" 을
        # 구분해 보여 준다). 모터가 답하지 않으면 아무것도 내보내지 않는다.
        self.pub_present = self.create_publisher(Int32, "gripper/present_position", 10)
        # 0.1 초마다 에러를 보고, 0.5 초마다 현재 위치를 알린다.
        self.create_timer(0.1, self._watchdog)

    # ------------------------------------------------------------ 연결·설정
    def _open_port(self):
        device = str(self.get_parameter("device").value)
        baud = int(self.get_parameter("baudrate").value)
        protocol = float(self.get_parameter("protocol").value)
        try:
            from dynamixel_sdk import PacketHandler, PortHandler
        except ImportError:
            self.get_logger().error(
                "[gripper] dynamixel_sdk 가 없습니다 — 명령을 받아도 모터는 움직이지 "
                "않습니다. (pip install dynamixel-sdk)")
            return

        self.port = PortHandler(device)
        self.packet = PacketHandler(protocol)
        if not self.port.openPort():
            self.get_logger().error(f"[gripper] 포트를 열지 못했습니다: {device}")
            self.port = None
            return
        if not self.port.setBaudRate(baud):
            self.get_logger().error(f"[gripper] 통신 속도 설정 실패: {baud}")
            self.port = None
            return
        if self._read_hw_error() != 0:
            self.get_logger().warn("[gripper] 시작할 때 하드웨어 에러가 남아 있어 재부팅합니다.")
            self._reboot()
        self._write_torque(False)          # 모드(EEPROM)는 토크가 꺼져 있어야 바뀐다
        self._ensure_mode()
        self._write_profile()
        self._restore_turns_from_file()
        self._write_torque(True)
        self.get_logger().info(
            f"[gripper] {device} @{baud} 연결, 토크 ON (ID {self.dxl_id}), "
            f"범위 {self.get_parameter('min_position').value}~"
            f"{self.get_parameter('max_position').value}, 현재 위치 {self.last_present}")

    def _write1(self, addr, value, what) -> bool:
        result, error = self.packet.write1ByteTxRx(self.port, self.dxl_id, addr, int(value))
        if result != 0 or error != 0:
            reason = (self.packet.getTxRxResult(result) if result != 0
                      else self.packet.getRxPacketError(error))
            self.get_logger().error(f"[gripper] {what} 실패: {reason}")
            return False
        return True

    def _write4(self, addr, value, what) -> bool:
        # 음수는 2의 보수로 보낸다(확장 위치 모드는 음수 위치도 받는다).
        result, error = self.packet.write4ByteTxRx(
            self.port, self.dxl_id, addr, int(value) & 0xFFFFFFFF)
        if result != 0 or error != 0:
            reason = (self.packet.getTxRxResult(result) if result != 0
                      else self.packet.getRxPacketError(error))
            self.get_logger().error(f"[gripper] {what} 실패: {reason}")
            return False
        return True

    def _write_torque(self, on: bool):
        if self.port is None:
            return
        self._write1(ADDR_TORQUE_ENABLE, 1 if on else 0, f"토크 {'ON' if on else 'OFF'}")

    def _ensure_mode(self):
        """max_position 이 한 바퀴(4095)를 넘으면 확장 위치 모드여야 한다."""
        if int(self.get_parameter("max_position").value) < TICKS_PER_REV:
            return
        mode, result, _ = self.packet.read1ByteTxRx(self.port, self.dxl_id, ADDR_OPERATING_MODE)
        if result != 0:
            self.get_logger().error("[gripper] 동작 모드를 읽지 못했습니다.")
            return
        if mode == MODE_EXTENDED_POSITION:
            return
        self.get_logger().warn(
            f"[gripper] 동작 모드가 {mode} 입니다 — 4095 를 넘는 위치를 쓰려고 확장 위치 "
            "모드(4)로 바꿉니다.")
        self._write1(ADDR_OPERATING_MODE, MODE_EXTENDED_POSITION, "동작 모드 변경")

    def _write_profile(self):
        """프로파일 속도·가속도. RAM 값이라 재부팅하면 지워지므로 그때마다 다시 쓴다."""
        accel = int(self.get_parameter("profile_acceleration").value)
        velocity = int(self.get_parameter("profile_velocity").value)
        if accel > 0:
            self._write4(ADDR_PROFILE_ACCELERATION, accel, "프로파일 가속도")
        if velocity > 0:
            self._write4(ADDR_PROFILE_VELOCITY, velocity, "프로파일 속도")

    # ------------------------------------------------------------ 위치
    def _read_motor_position(self):
        """모터가 보고하는 위치(부호 있는 값). 못 읽으면 None."""
        value, result, _ = self.packet.read4ByteTxRx(
            self.port, self.dxl_id, ADDR_PRESENT_POSITION)
        if result != 0:
            return None
        return value - 0x100000000 if value > 0x7FFFFFFF else value

    def _restore_turns_from_file(self):
        """켤 때: 파일의 마지막 위치와 바퀴 수만 다르면 그만큼 보정한다."""
        motor = self._read_motor_position()
        if motor is None:
            return
        saved = None
        try:
            with open(self.state_file, encoding="utf-8") as handle:
                saved = int(handle.read().split()[0])
        except (OSError, ValueError, IndexError):
            pass
        self.turn_offset = 0
        if saved is not None and abs(saved - motor) > TICKS_PER_REV // 2:
            turns = turns_lost(saved, motor)
            if same_but_turns(saved, motor):
                self.turn_offset = turns * TICKS_PER_REV
                self.get_logger().warn(
                    f"[gripper] 모터 위치 {motor} 가 마지막 위치 {saved} 와 {turns} 바퀴 다릅니다 "
                    f"— 전원을 다시 켜 바퀴 수가 지워진 것으로 보고 보정합니다(현재 "
                    f"{motor + self.turn_offset}).")
            else:
                self.get_logger().warn(
                    f"[gripper] 모터 위치 {motor} 가 마지막 위치 {saved} 와 다릅니다(바퀴 수만 "
                    "다른 게 아님 — 손이나 다른 프로그램으로 움직인 듯) — 보정하지 않습니다. "
                    "그리퍼가 실제로 이 위치인지 확인하세요.")
        self.last_present = motor + self.turn_offset
        self._save_present(self.last_present)

    def _save_present(self, position: int):
        if self._saved_present is not None and abs(position - self._saved_present) < 10:
            return
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            with open(self.state_file, "w", encoding="utf-8") as handle:
                handle.write(f"{int(position)}\n")
            self._saved_present = int(position)
        except OSError:
            pass

    # ------------------------------------------------------------ 감시·복구
    def _watchdog(self):
        self._tick += 1
        if self.port is None:
            return
        self._check_and_recover()
        if self._tick % 5 == 0:
            self._publish_present()

    def _read_hw_error(self) -> int:
        value, result, _ = self.packet.read1ByteTxRx(
            self.port, self.dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        return value if result == 0 else 0

    def _check_and_recover(self):
        status = self._read_hw_error()
        if status == 0 or self.recovery_count >= MAX_RECOVERY_TRY:
            return
        names = [name for bit, name in HW_ERROR_BITS.items() if status & (1 << bit)]
        self.get_logger().error(
            f"[gripper] 하드웨어 에러: {', '.join(names) or hex(status)} — 복구합니다")
        self._recover()

    def _reboot(self) -> bool:
        import time
        result, _ = self.packet.reboot(self.port, self.dxl_id)
        if result != 0:
            self.get_logger().error(f"[gripper] 재부팅 실패: {self.packet.getTxRxResult(result)}")
            return False
        time.sleep(0.5)             # 모터가 다시 뜰 시간
        return True

    def _recover(self):
        """재부팅으로 에러 래치를 풀고, 잊은 바퀴 수를 되찾은 뒤 마지막 목표를 다시 보낸다."""
        self.recovery_count += 1
        self.get_logger().warn(f"[gripper] 에러 초기화 시도 ({self.recovery_count}/{MAX_RECOVERY_TRY})")
        before = self.last_present          # 재부팅 직전 위치(우리 쪽 값)
        if not self._reboot():
            return
        if self._read_hw_error() != 0:
            self.get_logger().error("[gripper] 초기화 뒤에도 에러가 남았습니다 — 부하를 확인하세요.")
            return
        motor = self._read_motor_position()
        if before is not None and motor is not None:
            turns = turns_lost(before, motor)
            if turns:
                self.get_logger().warn(
                    f"[gripper] 재부팅으로 바퀴 수가 지워졌습니다(모터 {motor}, 직전 {before}) "
                    f"— {turns} 바퀴 보정합니다.")
            self.turn_offset = turns * TICKS_PER_REV
        elif self.last_goal is not None:
            self.get_logger().error(
                "[gripper] 재부팅 직전 위치를 몰라 바퀴 수를 되찾을 수 없습니다 — 목표를 다시 "
                "보내지 않습니다(한 바퀴 더 조일 수 있어서).")
            self.last_goal = None
        self._write_profile()
        self._write_torque(True)
        if self.last_goal is not None:
            motor_goal = self.last_goal - self.turn_offset
            self.get_logger().info(
                f"[gripper] 마지막 목표 재전송: {self.last_goal} (모터 값 {motor_goal})")
            self._write4(ADDR_GOAL_POSITION, motor_goal, "목표 재전송")
        if self.recovery_count >= MAX_RECOVERY_TRY:
            self.get_logger().error("[gripper] 복구 한도에 닿았습니다 — 기구 부하나 전원을 점검하세요.")

    def _publish_present(self):
        if self.port is None:
            return
        motor = self._read_motor_position()
        if motor is None:
            if not self._read_failed:
                self.get_logger().warn("[gripper] 현재 위치를 읽지 못했습니다(모터 응답 없음).")
                self._read_failed = True
            return
        if self._read_failed:
            self.get_logger().info("[gripper] 모터가 다시 응답합니다.")
            self._read_failed = False
        self.last_present = motor + self.turn_offset
        self._save_present(self.last_present)
        self.pub_present.publish(Int32(data=int(self.last_present)))

    # ------------------------------------------------------------ 명령
    def cb_set_position(self, msg):
        goal = int(msg.data)
        low = int(self.get_parameter("min_position").value)
        high = int(self.get_parameter("max_position").value)
        if not (low <= goal <= high):
            self.get_logger().warn(
                f"[gripper] 목표 위치가 범위({low}~{high})를 벗어났습니다({goal}) — 무시합니다.")
            return
        if self.port is None:
            self.get_logger().warn(f"[gripper] (연결 없음) 목표 위치 {goal} 를 버립니다.")
            return
        self.last_goal = goal
        self.recovery_count = 0      # 새 명령이 오면 복구 횟수를 다시 센다
        if self._write4(ADDR_GOAL_POSITION, goal - self.turn_offset, "명령 전송"):
            note = f" (모터 값 {goal - self.turn_offset})" if self.turn_offset else ""
            self.get_logger().info(f"[gripper] 목표 위치 {goal}{note}")

    def destroy_node(self):
        # 종료할 때는 토크를 뺀다 — 전원이 들어온 채로 물고 있지 않게.
        self._write_torque(False)
        if self.port is not None:
            try:
                self.port.closePort()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperDynamixelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
