import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from dynamixel_sdk import *  # 설치한 Dynamixel SDK 임포트

# ---------- X 시리즈 제어 테이블 주소 (Protocol 2.0 공통) ----------
ADDR_TORQUE_ENABLE          = 64    # 1바이트: 1=On, 0=Off
ADDR_HARDWARE_ERROR_STATUS  = 70    # 1바이트: 하드웨어 에러 래치 상태
ADDR_GOAL_POSITION          = 116   # 4바이트: 목표 위치 (0~4095)
ADDR_PRESENT_POSITION       = 132   # 4바이트: 현재 위치 읽기

# Hardware Error Status 비트별 의미
HW_ERROR_BITS = {
    0: "입력 전압 에러",
    2: "과열 에러",
    3: "엔코더 에러",
    4: "전기적 충격 에러",
    5: "과부하(Overload) 에러",
}

DXL_MIN_POSITION = 0
DXL_MAX_POSITION = 4095
MAX_RECOVERY_TRY = 3        # 연속 복구 시도 한계 (무한 리부팅 방지)
# ------------------------------------------------------------------


class DxlSimpleControlNode(Node):
    def __init__(self):
        super().__init__('dxl_simple_control_node')

        # 1. 통신 기본 설정
        self.dxl_id = 1
        self.baudrate = 57600
        self.device_name = '/dev/ttyUSB0'
        self.protocol_version = 2.0

        self.last_goal = None       # 복구 후 재전송할 마지막 목표 위치
        self.recovery_count = 0     # 연속 복구 시도 횟수

        # 2. 포트 및 패킷 핸들러 초기화
        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        # 3. 포트 열기
        if self.portHandler.openPort():
            self.get_logger().info(f"성공: 포트를 열었습니다 ({self.device_name})")
        else:
            self.get_logger().error("실패: 포트를 열 수 없습니다!")
            return

        # 4. 보레이트 설정
        if self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().info(f"성공: 통신 속도를 설정했습니다 ({self.baudrate})")
        else:
            self.get_logger().error("실패: 통신 속도 설정에 실패했습니다!")
            return

        # 5. 시작 시 남아있는 에러가 있으면 먼저 정리
        if self.read_hw_error() != 0:
            self.get_logger().warn("시작 시 하드웨어 에러가 남아있어 초기화합니다.")
            self.recover_from_error()
        else:
            self.enable_torque()

        # 6. 구독자 생성: 'set_pos' 토픽을 받으면 이동
        self.subscription = self.create_subscription(
            Int32, 'set_pos', self.position_callback, 10)

        # 7. 감시 타이머: 10Hz로 에러 상태를 확인
        self.watchdog = self.create_timer(0.1, self.check_and_recover)

    # ---------------- 기본 동작 ----------------

    def enable_torque(self):
        result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, 1)
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"토크 ON 실패: {self.packetHandler.getTxRxResult(result)}")
            return False
        self.get_logger().info("다이나믹셀 토크 ON 완료!")
        return True

    def send_goal(self, goal_pos):
        result, error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_GOAL_POSITION, goal_pos)
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"명령 전송 실패: {self.packetHandler.getTxRxResult(result)}")
            return False
        return True

    def position_callback(self, msg):
        goal_pos = max(DXL_MIN_POSITION, min(DXL_MAX_POSITION, msg.data))
        if goal_pos != msg.data:
            self.get_logger().warn(f"범위 밖 입력 {msg.data} -> {goal_pos}로 제한")

        self.get_logger().info(f"목표 위치 명령 수신: {goal_pos}")
        self.last_goal = goal_pos
        self.recovery_count = 0     # 새 명령이 오면 복구 시도 횟수 초기화
        self.send_goal(goal_pos)

    # ---------------- 에러 감시 및 복구 ----------------

    def read_hw_error(self):
        """하드웨어 에러 상태를 읽어 반환. 통신 실패 시 0."""
        value, result, error = self.packetHandler.read1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        if result != COMM_SUCCESS:
            return 0
        return value

    def describe_error(self, status):
        names = [name for bit, name in HW_ERROR_BITS.items() if status & (1 << bit)]
        return ", ".join(names) if names else f"알 수 없는 에러(0x{status:02X})"

    def check_and_recover(self):
        """주기적으로 에러를 확인하고, 발생했으면 초기화 후 명령을 재전송"""
        status = self.read_hw_error()
        if status == 0:
            return

        if self.recovery_count >= MAX_RECOVERY_TRY:
            return      # 한계 초과 시 더 이상 리부팅하지 않음

        self.get_logger().error(f"하드웨어 에러 감지: {self.describe_error(status)}")
        self.recover_from_error()

    def recover_from_error(self):
        """Reboot으로 에러 래치를 해제하고 토크를 다시 켠 뒤 목표 위치를 재전송"""
        self.recovery_count += 1
        self.get_logger().warn(f"에러 초기화 시도 ({self.recovery_count}/{MAX_RECOVERY_TRY})")

        result, error = self.packetHandler.reboot(self.portHandler, self.dxl_id)
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"리부팅 실패: {self.packetHandler.getTxRxResult(result)}")
            return

        time.sleep(0.5)             # 모터가 재기동할 시간 확보

        if self.read_hw_error() != 0:
            self.get_logger().error("초기화 후에도 에러가 남아있습니다. 부하를 확인하세요.")
            return

        if not self.enable_torque():
            return

        if self.last_goal is not None:
            self.get_logger().info(f"마지막 목표 위치 재전송: {self.last_goal}")
            self.send_goal(self.last_goal)

        if self.recovery_count >= MAX_RECOVERY_TRY:
            self.get_logger().error(
                "복구 시도 한계에 도달했습니다. 기구부 부하나 전원을 점검해 주세요.")


def main(args=None):
    rclpy.init(args=args)
    node = DxlSimpleControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료 중...')
    finally:
        # 종료 시 토크 OFF (안전을 위해 모터 힘을 뺌)
        node.packetHandler.write1ByteTxRx(node.portHandler, node.dxl_id, ADDR_TORQUE_ENABLE, 0)
        node.portHandler.closePort()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
