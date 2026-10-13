"""그리퍼 제어를 한 곳에 모은다.

지금 쓰는 그리퍼는 다이나믹셀 **XD540-T270** 이고, `gripper_dynamixel_node`
(또는 기존 `dyn_test`)가 `/set_pos` 로 받은 목표 위치를 모터에 그대로
써 준다. 그래서 기본 백엔드는 `int_topic` 이다 — README 기준 **3185 가 파지,
0 이 열림**이다.

백엔드는 갈아 끼울 수 있다. `none` 이면 명령을 내보내지 않고 로그만 남겨서
로봇 동작만 먼저 시험할 수 있고, `serial` 은 Orin 에서 직접 시리얼로
문자열을 보낸다(명령 문자열은 파라미터로 넣는다).
"""

from __future__ import annotations


class GripperClient:
    """열기/닫기 두 동작만 있는 최소 인터페이스."""

    def __init__(self, node, config):
        self.node = node
        self.config = config
        self.last_command = ""
        self._pub = None
        self._serial = None
        self._setup()

    def _setup(self):
        backend = self.config.backend
        if backend == "int_topic":
            if not self.config.topic:
                self.node.get_logger().warn("[gripper] int_topic 인데 토픽 이름이 비었습니다.")
                return
            from std_msgs.msg import Int32
            self._pub = self.node.create_publisher(Int32, self.config.topic, 10)
            self.node.get_logger().info(
                f"[gripper] 다이나믹셀 위치 제어: {self.config.topic} "
                f"(닫기 {self.config.close_value} / 열기 {self.config.open_value})")
        elif backend == "topic":
            if not self.config.topic:
                self.node.get_logger().warn("[gripper] topic 백엔드인데 토픽 이름이 비었습니다.")
                return
            from std_msgs.msg import Bool
            self._pub = self.node.create_publisher(Bool, self.config.topic, 10)
            self.node.get_logger().info(f"[gripper] 토픽 제어: {self.config.topic}")
        elif backend == "serial":
            if not self.config.serial_port:
                self.node.get_logger().warn("[gripper] serial 백엔드인데 포트가 비었습니다.")
                return
            try:
                import serial  # pyserial. 시리얼을 쓸 때만 필요하다.
                self._serial = serial.Serial(
                    self.config.serial_port, self.config.serial_baud, timeout=0.5)
                self.node.get_logger().info(
                    f"[gripper] 시리얼 제어: {self.config.serial_port} "
                    f"@{self.config.serial_baud}")
            except Exception as exc:
                self.node.get_logger().error(f"[gripper] 시리얼 열기 실패: {exc}")
        elif backend == "none":
            self.node.get_logger().warn(
                "[gripper] 제어 방법이 아직 설정되지 않았습니다 — 명령을 로그로만 남깁니다.")
        else:
            self.node.get_logger().error(f"[gripper] 모르는 backend 입니다: {backend!r}")

    @property
    def ready(self) -> bool:
        return self._pub is not None or self._serial is not None

    def close(self) -> tuple:
        """오므리기. (성공, 안내문)."""
        return self._send(True, self.config.close_command, "close")

    def open(self) -> tuple:
        """열기. (성공, 안내문)."""
        return self._send(False, self.config.open_command, "open")

    def _send(self, flag: bool, command: str, label: str) -> tuple:
        self.last_command = label
        if self._pub is not None:
            if self.config.backend == "int_topic":
                from std_msgs.msg import Int32
                goal = self.config.close_value if flag else self.config.open_value
                self._pub.publish(Int32(data=int(goal)))
                return True, f"[gripper] {label}({goal}) -> {self.config.topic}"
            from std_msgs.msg import Bool
            self._pub.publish(Bool(data=flag))
            return True, f"[gripper] {label} -> {self.config.topic}"
        if self._serial is not None:
            if not command:
                return False, f"[gripper] {label} 시리얼 명령 문자열이 비어 있습니다."
            try:
                self._serial.write(command.encode("utf-8"))
                self._serial.flush()
            except Exception as exc:
                return False, f"[gripper] {label} 시리얼 전송 실패: {exc}"
            return True, f"[gripper] {label} -> {self.config.serial_port}"
        # 설정 전: 시퀀스를 막지는 않고(동작 시험용) 로그만 남긴다.
        return True, f"[gripper] (미설정) {label} 명령을 내보내지 않았습니다."

    def shutdown(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
