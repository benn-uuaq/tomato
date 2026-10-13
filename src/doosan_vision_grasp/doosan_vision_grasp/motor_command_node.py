import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        # 시리얼 포트 설정 (포트 경로는 /dev/ttyUSB0 등으로 확인 필요)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        
        # 'set_degree' 토픽을 구독함
        self.subscription = self.create_subscription(
            Float32,
            'set_degree',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 받은 각도를 문자열로 변환하여 STM32로 전송 (예: "90.0\n")
        command = f"{msg.data}\n"
        self.ser.write(command.encode())
        self.get_logger().info(f'Sent to STM32: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()