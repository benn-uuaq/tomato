"""다이나믹셀 그리퍼(XD540-T270) 제어 노드.

`doosan_vision_grasp` 의 `dyn_test`(dxl_test.py)와 같은 일을 하되, 포트·ID·
토픽을 **파라미터로** 받는다. 워크스페이스 하나만 띄우면 로봇·비전·그리퍼가
모두 올라오게 하려고 이 패키지에 같이 둔다.

    ros2 run elite_robot_controller gripper_dynamixel_node \
        --ros-args -p device:=/dev/ttyUSB0

`/set_pos` (std_msgs/Int32) 로 목표 위치(0~4095)를 받는다. README 기준
**3185 가 파지, 0 이 열림**이다. 시퀀스 노드가 이 값을 보낸다.

`dynamixel_sdk` 가 없거나 포트를 못 열면 노드는 죽지 않고 경고만 남긴다 —
그리퍼 없이 로봇 동작만 시험하는 경우가 많기 때문이다.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

#: XD540-T270 제어 테이블 주소.
ADDR_TORQUE_ENABLE = 64      # 1바이트: 1 = On, 0 = Off
ADDR_GOAL_POSITION = 116     # 4바이트: 목표 위치 (0~4095)
ADDR_PRESENT_POSITION = 132  # 4바이트: 현재 위치


class GripperDynamixelNode(Node):

    def __init__(self):
        super().__init__("gripper_dynamixel")
        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("protocol", 2.0)
        self.declare_parameter("command_topic", "/set_pos")
        #: 목표 위치 허용 범위. 범위를 벗어난 값은 버린다(잘못된 명령으로
        #: 그리퍼가 끝까지 물리는 것을 막는다).
        self.declare_parameter("min_position", 0)
        self.declare_parameter("max_position", 4095)

        self.port = None
        self.packet = None
        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self._open_port()

        topic = str(self.get_parameter("command_topic").value)
        self.create_subscription(Int32, topic, self.cb_set_position, 10)
        self.get_logger().info(f"[gripper] 목표 위치 대기: {topic} (Int32)")

        # 모터가 실제로 어디 있는지 알린다(패널이 "열림/닫힘" 과 "응답 없음" 을
        # 구분해 보여 준다). 모터가 답하지 않으면 아무것도 내보내지 않는다.
        self.pub_present = self.create_publisher(Int32, "gripper/present_position", 10)
        self._read_failed = False
        self.create_timer(0.5, self._publish_present)

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
        self._write_torque(True)
        self.get_logger().info(f"[gripper] {device} @{baud} 연결, 토크 ON (ID {self.dxl_id})")

    def _write_torque(self, on: bool):
        if self.port is None:
            return
        result, error = self.packet.write1ByteTxRx(
            self.port, self.dxl_id, ADDR_TORQUE_ENABLE, 1 if on else 0)
        if result != 0:
            self.get_logger().error(
                f"[gripper] 토크 {'ON' if on else 'OFF'} 실패: "
                f"{self.packet.getTxRxResult(result)}")

    def _publish_present(self):
        if self.port is None:
            return
        value, result, error = self.packet.read4ByteTxRx(
            self.port, self.dxl_id, ADDR_PRESENT_POSITION)
        if result != 0:
            if not self._read_failed:
                self.get_logger().warn(
                    f"[gripper] 현재 위치를 읽지 못했습니다(모터 응답 없음): "
                    f"{self.packet.getTxRxResult(result)}")
                self._read_failed = True
            return
        if self._read_failed:
            self.get_logger().info("[gripper] 모터가 다시 응답합니다.")
            self._read_failed = False
        # 4 바이트 부호 있는 값으로 온다.
        if value > 0x7FFFFFFF:
            value -= 0x100000000
        self.pub_present.publish(Int32(data=int(value)))

    def cb_set_position(self, msg):
        goal = int(msg.data)
        low = int(self.get_parameter("min_position").value)
        high = int(self.get_parameter("max_position").value)
        if not (low <= goal <= high):
            self.get_logger().warn(
                f"[gripper] 목표 위치가 범위를 벗어났습니다({goal}) — 무시합니다.")
            return
        if self.port is None:
            self.get_logger().warn(f"[gripper] (연결 없음) 목표 위치 {goal} 를 버립니다.")
            return
        result, error = self.packet.write4ByteTxRx(
            self.port, self.dxl_id, ADDR_GOAL_POSITION, goal)
        if result != 0:
            self.get_logger().error(
                f"[gripper] 명령 전송 실패: {self.packet.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().warn(
                f"[gripper] 모터가 오류를 알렸습니다: {self.packet.getRxPacketError(error)}")
        else:
            self.get_logger().info(f"[gripper] 목표 위치 {goal}")

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
