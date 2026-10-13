import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO
import os
from datetime import datetime

class Open3DTomatoViewer(Node):
    def __init__(self):
        super().__init__('open3d_tomato_viewer')
        
        self.get_logger().info('YOLO 토마토 segmentation 모델 로드 중...')
        self.model = YOLO('/home/smartfarm/doosan_manipulator/tomato_v1.pt')
        self.bridge = CvBridge()

        # 내재변수 초기화
        self.pinhole_camera_intrinsic = None
        self.camera_info_received = False

        # CameraInfo 토픽의 안정적 수신을 위한 QoS 설정
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 💡 [토픽명 수정]: 제공해주신 topic list 기반으로 정확하게 매칭
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data
        )
        
        # 💡 기존의 /color/camera_info 대신 실제 매핑에 사용되는 aligned 뎁스 정보 토픽으로 변경합니다.
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, info_qos
        )

        self.latest_depth_img = None
        self.get_logger().info('=== 토마토 3D 데이터 수집 시스템 정상 가동 ===')
        self.get_logger().info('👉 2D 모니터 화면 활성화 중... (카메라 변수 매칭 대기)')

    def info_callback(self, msg):
        """카메라 고유 내재변수를 실시간으로 받아와 세팅합니다."""
        if not self.camera_info_received:
            width = msg.width
            height = msg.height
            fx = msg.k[0]
            fy = msg.k[4]
            ppx = msg.k[2]
            ppy = msg.k[5]

            # 실시간 해상도 데이터 기반 Open3D 카메라 세팅 자동 생성
            self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                width, height, fx, fy, ppx, ppy
            )
            self.camera_info_received = True
            self.get_logger().info(f'🟢 [성공] aligned_depth_to_color 내재변수 매칭 완료! 해상도: {width}x{height}')

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e:
            pass

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_image is None or cv_image.size == 0:
                return

            # YOLOv8 segmentation 추론 및 바운딩 박스/마스크 시각화
            results = self.model(cv_image, verbose=False, conf=0.25)
            result = results[0]
            annotated_frame = result.plot()

            # 💡 카메라 정보 매칭 여부와 관계없이 2D 화면은 막힘없이 실시간 표시
            cv2.imshow("Tomato 2D Monitor", annotated_frame)
            key = cv2.waitKey(1) & 0xFF

            # 's' 키를 누르면 포인트 클라우드 파일 저장
            if key == ord('s'):
                if self.latest_depth_img is None:
                    self.get_logger().warn('❌ 아직 뎁스 이미지가 들어오지 않았습니다!')
                    return
                if not self.camera_info_received:
                    self.get_logger().warn('❌ 아직 aligned 카메라 내재변수를 수신하지 못했습니다!')
                    return
                
                if result.masks is not None:
                    self.get_logger().info('🎬 고정밀 반구형 3D 데이터 파일 추출 시작...')
                    
                    mask_np = result.masks.data[0].cpu().numpy().astype(np.uint8)
                    if mask_np.shape != cv_image.shape[:2]:
                        mask_np = cv2.resize(mask_np, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)

                    # YOLO 마스크 테두리 3픽셀 침식 연산 (가장자리 산 모양 노이즈 원천 커팅)
                    kernel = np.ones((3, 3), np.uint8)
                    mask_np = cv2.erode(mask_np, kernel, iterations=1)

                    tomato_depth = self.latest_depth_img.copy()
                    tomato_depth[mask_np == 0] = 0
                    tomato_depth = tomato_depth.astype(np.uint16)
                    
                    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                    o3d_color = o3d.geometry.Image(rgb_image)
                    o3d_depth = o3d.geometry.Image(tomato_depth)

                    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        o3d_color, o3d_depth, depth_scale=1000.0, depth_trunc=0.5, convert_rgb_to_intensity=False
                    )

                    # 자동으로 연동된 정확한 내재변수로 3D 변환
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                        rgbd_image, self.pinhole_camera_intrinsic
                    )

                    # 오목함 방지 및 공간 정방향 매칭 변환 행렬
                    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

                    # 미세 아웃라이어 필터
                    if len(pcd.points) > 10:
                        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.2)
                        pcd = pcd.select_by_index(ind)

                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    target_dir = "/home/smartfarm/Desktop/pcd files"
                    
                    if not os.path.exists(target_dir):
                        os.makedirs(target_dir)
                        
                    save_path = os.path.join(target_dir, f"tomato_{timestamp}.pcd")
                    
                    o3d.io.write_point_cloud(save_path, pcd)
                    self.get_logger().info(f"🟢 💾 정확한 정렬 변수 기반 고정밀 PCD 저장 성공: {save_path}")
                else:
                    self.get_logger().warn('화면에 토마토가 보이지 않습니다! ❌')

        except Exception as e:
            self.get_logger().error(f"❌ 이미지 콜백 연산 에러: {e}")

def main():
    rclpy.init()
    node = Open3DTomatoViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()