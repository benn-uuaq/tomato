import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, Bool  # Bool 메시지 타입 추가
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO

class YoloPoseNodeD405(Node):
    def __init__(self):
        super().__init__('yolo_pose_node_d405')
        self.model = YOLO('/home/smartfarm/doosan_manipulator/rot_v6.pt')
        self.bridge = CvBridge()
        
        # 보정 계수 (Y: 1.176배 확대, X: 0.95배 축소)
        self.scale_x = 1.078   #1.08
        self.scale_y = 1.15

        # [추가] YOLO 활성화 스위치 구독
        self.enable_sub = self.create_subscription(Bool, '/enable_yolo', self.enable_cb, 10)
        self.is_enabled = True  # 기본값은 활성화 상태

        self.image_sub = self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.image_callback, qos_profile_sensor_data)  #/camera/camera/color/image_rect_raw/compressed' 였는데 먹통됨
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, qos_profile_sensor_data)

        self.target_pub = self.create_publisher(PointStamped, '/object_in_camera', 10)
        self.angle_pub = self.create_publisher(Float64, '/object_angle', 10)
        self.weight_raw_pub = self.create_publisher(Float64, '/object_weight_raw', 10)

        self.latest_depth = None
        self.intrinsics = None
        self.get_logger().info("D405 시각화 및 보정 노드 실행 중... (무게 계산 기능 추가)")

    def enable_cb(self, msg):
        """스위치 신호에 따라 활성화 상태를 업데이트합니다."""
        self.is_enabled = msg.data
        status = "활성화" if self.is_enabled else "비활성화"
        self.get_logger().info(f"YOLO 상태 변경: {status}")

    def info_callback(self, msg):
        self.intrinsics = msg.k

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

    def image_callback(self, msg):
        # 욜로가 꺼져 있어도 화면을 계속 띄우기 위해 is_enabled 체크로 바로 return하지 않음 [수정]
        if self.latest_depth is None or self.intrinsics is None:
            return

        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # [추가] YOLO 추론 및 데이터 처리는 활성화 상태일 때만 수행
        if self.is_enabled:
            results = self.model(cv_image, verbose=False, conf=0.25)

            all_targets = []
            for r in results:
                if r.boxes is not None:
                    for box in r.boxes:
                        b = box.xyxy[0].cpu().numpy().astype(int)
                        cv2.rectangle(cv_image, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)

                if r.keypoints is not None:
                    for i in range(len(r.keypoints.data)):
                        kp = r.keypoints.data[i].cpu().numpy()
                        pts = {j: kp[j][:2] for j in range(4) if len(kp) > j and kp[j][2] > 0.4}
                        
                        if len(pts) == 4:
                            target_data = self.process_target(pts)
                            if target_data:
                                all_targets.append(target_data)
                                ix, iy = int(target_data['ix']), int(target_data['iy'])
                                cv2.line(cv_image, (int(pts[0][0]), int(pts[0][1])), (int(pts[1][0]), int(pts[1][1])), (0, 0, 0), 2)
                                cv2.line(cv_image, (int(pts[2][0]), int(pts[2][1])), (int(pts[3][0]), int(pts[3][1])), (255, 255, 255), 2)
                                cv2.circle(cv_image, (ix, iy), 7, (0, 255, 255), -1)
                                
                                cv2.putText(cv_image, f"{target_data['angle']:.1f} deg", (ix+15, iy-35), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                                cv2.putText(cv_image, f"V: {target_data['v_len']:.1f}mm", (ix+15, iy-15), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                cv2.putText(cv_image, f"H: {target_data['h_len']:.1f}mm", (ix+15, iy+5), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                                cv2.putText(cv_image, f"W: {target_data['weight']:.1f}g", (ix+15, iy+25), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            if all_targets:
                best_target = max(all_targets, key=lambda x: x['iy']) 
                self.publish_data(best_target)
        else:
            # [선택] 욜로가 비활성화된 동안 화면에 표시할 문구
            pass

        # [중요] cv2.imshow는 is_enabled 상태와 상관없이 매 콜백마다 실행
        cv2.imshow("D405 YOLO Calibration & Visualizer", cv_image)
        cv2.waitKey(1) 

    def process_target(self, pts):
        x1, y1 = pts[0]
        x2, y2 = pts[1]
        x3, y3 = pts[2]
        x4, y4 = pts[3]

        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(denom) < 1e-6: return None
        ix = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
        iy = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom

        h, w = self.latest_depth.shape
        if not (0 <= int(ix) < w and 0 <= int(iy) < h):
            return None

        z_c = self.latest_depth[int(iy), int(ix)] / 1000.0
        if z_c <= 0: return None

        # [수정] 각 점의 Z(깊이) 좌표를 개별적으로 추출
        def get_pt_z(px, py):
            ix_p, iy_p = int(px), int(py)
            if 0 <= ix_p < w and 0 <= iy_p < h:
                z = self.latest_depth[iy_p, ix_p] / 1000.0
                return z if z > 0 else z_c
            return z_c

        z1, z2 = get_pt_z(x1, y1), get_pt_z(x2, y2)
        z3, z4 = get_pt_z(x3, y3), get_pt_z(x4, y4)

        fx, cx = self.intrinsics[0], self.intrinsics[2]
        fy, cy = self.intrinsics[4], self.intrinsics[5]

        # 물리 좌표 계산 (각 점의 개별 Z값 반영)
        real_x1 = (x1 - cx) * z1 / (fx / self.scale_x)
        real_y1 = (y1 - cy) * z1 / (fy / self.scale_y)
        real_x2 = (x2 - cx) * z2 / (fx / self.scale_x)
        real_y2 = (y2 - cy) * z2 / (fy / self.scale_y)
        real_x3 = (x3 - cx) * z3 / (fx / self.scale_x)
        real_y3 = (y3 - cy) * z3 / (fy / self.scale_y)
        real_x4 = (x4 - cx) * z4 / (fx / self.scale_x)
        real_y4 = (y4 - cy) * z4 / (fy / self.scale_y)

        real_ix = (ix - cx) * z_c / (fx / self.scale_x)
        real_iy = (iy - cy) * z_c / (fy / self.scale_y)

        # 물리적 각도 및 길이 계산
        real_dx = real_x1 - real_x2
        real_dy = real_y1 - real_y2
        angle_rad = math.atan2(real_dy, real_dx)
        angle_deg = math.degrees(angle_rad) - 90.0

        if angle_deg > 180:
            final_angle = angle_deg - 360
        elif angle_deg < -180:
            final_angle = angle_deg + 360
        else:
            final_angle = angle_deg

        # angle_rad = math.atan2(real_dx, real_dy) 각도 공삭 수정하려다 일단 실패
        #   final_angle = math.degrees(angle_rad)

        # [수정] Z축 차이까지 반영한 3차원 거리 공식 적용
        # v_len = math.sqrt((real_x1 - real_x2)**2 + (real_y1 - real_y2)**2 + (z1 - z2)**2) * 1000.0
        # h_len = math.sqrt((real_x3 - real_x4)**2 + (real_y3 - real_y4)**2 + (z3 - z4)**2) * 1000.0

        v_len = math.sqrt((real_x1 - real_x2)**2 + (real_y1 - real_y2)**2 ) * 1000.0
        h_len = math.sqrt((real_x3 - real_x4)**2 + (real_y3 - real_y4)**2 ) * 1000.0

        # [추가] 무게 회귀함수 적용 (제공해주신 공식)
        try:
            if v_len > 0:
                # weight = 1.142747 * h_len + 0.568602 * v_len - 44.035557
                weight = math.exp(1.72737928 * math.log(max(h_len, 1e-6)) + 1.02992568 * math.log(max(v_len, 1e-6)) - 6.9943622)
            else:
                weight = 0.0
        except Exception:
            weight = 0.0


        return {
            'x': real_ix, 'y': real_iy, 'z': z_c, 
            'angle': final_angle, 'ix': ix, 'iy': iy,
            'v_len': v_len, 'h_len': h_len,
            'weight': weight  # 무게 데이터 추가
        }

    def publish_data(self, target):
        t_msg = PointStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = 'camera_color_optical_frame'
        t_msg.point.x, t_msg.point.y, t_msg.point.z = target['x'], target['y'], target['z']
        self.target_pub.publish(t_msg)
        self.angle_pub.publish(Float64(data=float(target['angle'])))
        self.weight_raw_pub.publish(Float64(data=float(target['weight'])))

def main():
    rclpy.init()
    node = YoloPoseNodeD405()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()