import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine
from geometry_msgs.msg import Point, Pose  # Pose 추가
from std_msgs.msg import Float32, Float64, Int32, Bool
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class DoosanDeliveryNode(Node):
    def __init__(self):
        super().__init__('doosan_delivery_node')
        
        self.group = ReentrantCallbackGroup()
        self.cli = self.create_client(MoveLine, '/dsr01/motion/move_line', callback_group=self.group)
        
        self.degree_pub = self.create_publisher(Float32, 'set_degree', 10)
        self.dxl_pub = self.create_publisher(Int32, 'set_pos', 10)
        
        # [추가] YOLO 활성화 스위치 및 최종 정보 발행자
        self.yolo_switch_pub = self.create_publisher(Bool, '/enable_yolo', 10)
        self.final_info_pub = self.create_publisher(Pose, '/final_delivery_info', 10)
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        self.home_pos = [-550.0, 250.0, 350.0-50.0+300*0, 180.0, 180.0, 270.0]  #90
        self.is_ready_for_target = False  
        self.is_working = False           
        self.current_target_angle = 0.0
        self.current_target_weight = 0.0 # 무게 저장 변수
        
        self.angle_received = False
        self.weight_received = False # 무게 수신 플래그
        
        self.delivery_count = 0 

        # 구독자 설정
        self.sub = self.create_subscription(
            Point, '/target_xyz', self.target_callback, 1, callback_group=self.group)

        self.angle_sub = self.create_subscription(
            Float64, '/filtered_angle', self.angle_callback, 10, callback_group=self.group)

        self.weight_sub = self.create_subscription(
            Float64, '/filtered_weight', self.weight_callback, 10, callback_group=self.group)

        self.init_timer = self.create_timer(0.1, self.initial_home_move, callback_group=self.group)

    def angle_callback(self, msg):
        self.current_target_angle = msg.data
        self.angle_received = True

    def weight_callback(self, msg):
        self.current_target_weight = msg.data
        self.weight_received = True

    def publish_degree(self, degree):
        msg = Float32()
        msg.data = float(degree)
        self.degree_pub.publish(msg)

    def publish_dxl_pos(self, pos_value):
        msg = Int32()
        msg.data = int(pos_value)
        self.dxl_pub.publish(msg)

    def initial_home_move(self):
        if self.init_timer is not None:
            self.init_timer.cancel()
            self.destroy_timer(self.init_timer)
            self.init_timer = None 
        self.send_move_line(self.home_pos)
        self.publish_degree(0.0)
        self.publish_dxl_pos(3072)
        self.yolo_switch_pub.publish(Bool(data=True))
        time.sleep(1.5)
        self.is_ready_for_target = True
        self.get_logger().info('✅ [준비 완료] 시스템 가동 시작')

    def target_callback(self, msg):
        # 모든 데이터가 준비되었는지 확인
        if not self.is_ready_for_target or self.is_working or not self.angle_received or not self.weight_received:
            return

        # 동작 시작 전 플래그 처리 (중복 진입 방지)
        self.is_working = True
        self.is_ready_for_target = False

        time.sleep(0.1) 

        # ================= [좌표 계산 로직 수정] =================
        # 의도하신 배치:
        # (뒷줄 Y=-110) [6번째(idx5): -670] [5번째(idx4): -610] [4번째(idx3): -550]
        # (앞줄 Y= -50) [3번째(idx2): -670] [2번째(idx1): -610] [1번째(idx0): -550]
        
        full_idx = self.delivery_count % 6
        y_idx = full_idx // 3  # 0이면 앞줄, 1이면 뒷줄
        x_idx = full_idx % 3   # 0, 1, 2 (오른쪽에서 왼쪽으로)

        # 기준점 설정
        base_x = -670.0
        base_y = -50.0
        
        current_delivery_x = base_x + (x_idx * 60.0)
        current_delivery_y = base_y + (y_idx * -60.0)
        # =======================================================

        # 최종 정보 발행 및 로그
        final_msg = Pose()
        final_msg.position.x, final_msg.position.y, final_msg.position.z = msg.x, msg.y, msg.z
        final_msg.orientation.x = self.current_target_angle
        final_msg.orientation.y = self.current_target_weight
        self.final_info_pub.publish(final_msg)

        self.get_logger().info(f"🍓 [{full_idx + 1}/6번째 작업] X:{current_delivery_x}, Y:{current_delivery_y}")

        # YOLO 끄기
        self.yolo_switch_pub.publish(Bool(data=False))
        
        t_x, t_y, t_z = msg.x, msg.y, msg.z
        final_j6_angle = 270.0 + self.current_target_angle
        calculated_z = t_z + 100.0-10.0  # 이거 z 좌표가 두 직선의 교점의 x,y에서의 z 좌표라 그립점 까지는 높이를 조금 더 내려야한다!!

        steps = [
            ("2단계: 수평 이동", [t_x, t_y, 300.0, 180.0, 180.0, final_j6_angle]),
            ("3단계: 수직 하강", [t_x, t_y, calculated_z, 180.0, 180.0, final_j6_angle]),
            ("5단계: 수직 상승", [t_x, t_y, 300.0, 180.0, 180.0, 270.0]),
            ("6단계: 배송지 이동", [current_delivery_x, current_delivery_y, 300.0, 180.0, 180.0, 270.0]),
            ("7단계: 포장 하강", [current_delivery_x, current_delivery_y, calculated_z + 55.0, 180.0, 180.0, 270.0]),
            ("8단계: 수직 상승", [current_delivery_x, current_delivery_y, 300.0, 180.0, 180.0, 270.0]),
            ("9단계: 홈 복귀", self.home_pos)
        ]

        # 로봇 동작 실행
        for name, pos in steps:
            if "6단계" in name: self.publish_dxl_pos(1024)
            elif "9단계" in name: self.publish_dxl_pos(3072)
            self.send_move_line(pos)
            if "3단계" in name:
                self.publish_degree(-110.0)  #원래 -80 이었느데 d컷 나갔는지 안 잡힘
                time.sleep(0.5)
            elif "7단계" in name:
                self.publish_degree(-70.0)
                time.sleep(0.5)
            elif "8단계" in name:
                # [추가] 수직 상승 완료 후: 손가락을 완전히 벌림 (0도)
                self.publish_degree(0.0)
                time.sleep(0.2)

        # 작업 완료 후 카운트 증가 및 플래그 초기화
        self.delivery_count += 1
        self.angle_received = False
        self.weight_received = False
        self.is_working = False
        
        time.sleep(0.5)
        self.yolo_switch_pub.publish(Bool(data=True))
        time.sleep(1.5) 
        self.is_ready_for_target = True

    def send_move_line(self, pos):
        req = MoveLine.Request()
        req.pos = pos
        req.vel, req.acc = [200.0, 200.0], [150.0, 150.0]
        future = self.cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = DoosanDeliveryNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()