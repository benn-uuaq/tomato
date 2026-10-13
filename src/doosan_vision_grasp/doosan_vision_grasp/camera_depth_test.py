import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CameraDepthTester(Node):
    def __init__(self):
        super().__init__('camera_depth_test')
        self.bridge = CvBridge()

        # 카메라 내재 변수(주점 위치) 초기화
        self.cx = None
        self.cy = None

        # 이미지 및 정보 구독
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile_sensor_data
        )

        self.latest_depth_img = None
        self.last_log_time = time.time()

        self.get_logger().info('=== 📷 리얼센스 순수 Depth(Z) 값 정밀 테스트 시스템 시작 ===')

    def info_callback(self, msg):
        # 실시간 해상도에 맞는 이미지 정중앙(주점 cx, cy) 좌표 획득
        if self.cx is None:
            self.cx = int(msg.k[2])
            self.cy = int(msg.k[5])
            self.get_logger().info(f"🎯 카메라 주점 매핑 완료 -> 중심 픽셀 좌표: (X:{self.cx}, Y:{self.cy})")

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"깊이 이미지 변환 실패: {e}")
            return

        if self.latest_depth_img is None or self.cx is None:
            return

        current_time = time.time()
        # ⏱️ 과도한 로그 방지를 위해 1초에 한 번씩만 출력
        if current_time - self.last_log_time >= 1.0:
            try:
                # 1️⃣ 화면 정중앙 픽셀의 깊이(Z) 값 직접 추출
                center_z = float(self.latest_depth_img[self.cy, self.cx])

                # 2️⃣ 만약 정중앙 픽셀이 홀(Hole, 0)인 경우 주변 5x5 영역의 중간값으로 대체 (노이즈 방지)
                if center_z <= 0:
                    roi = self.latest_depth_img[self.cy-2:self.cy+3, self.cx-2:self.cx+3]
                    valid_roi = roi[roi > 0]
                    if len(valid_roi) > 0:
                        center_z = float(np.median(valid_roi))

                # 3️⃣ 터미널에 raw Z값 출력
                if center_z > 0:
                    self.get_logger().info(f"📐 [실시간 중심점 깊이] Z = {center_z:.1f} mm")
                else:
                    self.get_logger().warn("⚠️ 현재 중심점 깊이를 측정할 수 없습니다. (Too close or Invalid)")

                self.last_log_time = current_time

            except Exception as e:
                self.get_logger().error(f"깊이 연산 에러: {e}")

            # 모니터링용 화면 표시 (정중앙에 빨간 점 표시)
            # 깊이 이미지를 보기 편하게 8비트로 변환하여 보여줍니다.
            depth_show = cv2.normalize(self.latest_depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_show, cv2.COLORMAP_JET)
            cv2.circle(depth_color, (self.cx, self.cy), 5, (0, 0, 255), -1)
            cv2.imshow("Center Depth Pointer", depth_color)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraDepthTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()