import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from dynamixel_sdk import * # 설치한 Dynamixel SDK 임포트

# ---------- XD540-T270 제어 테이블 주소 (이미지 확인 완료) ----------
ADDR_TORQUE_ENABLE      = 64    # 1바이트: 1=On, 0=Off
ADDR_GOAL_POSITION      = 116   # 4바이트: 목표 위치 (0~4095)
ADDR_PRESENT_POSITION   = 132   # 4바이트: 현재 위치 읽기
# -----------------------------------------------------------

class DxlSimpleControlNode(Node):
    def __init__(self):
        super().__init__('dxl_simple_control_node')
        
        # 1. 통신 기본 설정
        self.dxl_id = 1
        self.baudrate = 57600
        self.device_name = '/dev/ttyUSB0'
        self.protocol_version = 2.0

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

        # 5. 토크 ON (모터를 움직이기 위해 필수)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, 1)
        
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"통신 에러: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        else:
            self.get_logger().info("다이나믹셀 토크 ON 완료!")

        # 6. 구독자 생성: 'set_pos' 토픽을 받으면 이동
        self.subscription = self.create_subscription(
            Int32,
            'set_pos',
            self.position_callback,
            10)

    def position_callback(self, msg):
        # 0 ~ 4095 사이의 값을 목표 위치로 전송
        goal_pos = msg.data
        self.get_logger().info(f"목표 위치 명령 수신: {goal_pos}")
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_GOAL_POSITION, goal_pos)
        
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"명령 전송 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()