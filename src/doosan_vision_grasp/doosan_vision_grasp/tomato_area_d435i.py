import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo  
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time

class TomatoHeightAndAreaMeasurer(Node):
    def __init__(self):
        super().__init__('tomato_area_d435i')
        
        self.get_logger().info('YOLO 토마토 segmentation 모델 로드 중...')
        self.model = YOLO('/home/smartfarm/doosan_manipulator/tomato_v1.pt')
        self.bridge = CvBridge()
        self.get_logger().info('YOLO 모델 로드 완료!')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ⚖️ 토마토 밀도 설정 상수 (g/cm3)
        self.TOMATO_DENSITY = 1.086  

        # 📡 구독자(Subscriber) 설정
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile_sensor_data
        )

        # 📡 발행자(Publisher) 추가
        self.pose_pub = self.create_publisher(Point, '/tomato/center_position', 10)
        self.weight_pub = self.create_publisher(Float32, '/tomato/estimated_weight', 10)

        self.latest_depth_img = None
        self.last_log_time = time.time()
        self.tomato_buffer = {}

        self.get_logger().info('=== [D435i] 실시간 카메라인포 반영 및 왜곡 제어 시스템 시작 ===')

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f"🎯 D435i 카메라 파라미터 로드 완료 -> fx: {self.fx:.1f}, fy: {self.fy:.1f}, cx: {self.cx:.1f}, cy: {self.cy:.1f}")

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"깊이 이미지 변환 실패: {e}")

    def image_callback(self, msg):
        if self.latest_depth_img is None or self.fx is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_image is None or cv_image.size == 0:
                return

            results = self.model(cv_image, verbose=False, conf=0.25, retina_masks=True)
            result = results[0]
            annotated_frame = result.plot()

            overlay = annotated_frame.copy()

            if result.masks is not None:
                for i, mask in enumerate(result.masks.data):
                    mask_np = mask.cpu().numpy().astype(np.uint8)
                    # ↓ 진단용 임시 코드
                    if i == 0:
                        self.get_logger().info(
                            f"[진단] mask shape: {mask_np.shape} / image shape: {cv_image.shape[:2]}",
                            throttle_duration_sec=2.0)                        
                    
                    
                    if mask_np.shape != cv_image.shape[:2]:
                        mask_np = cv2.resize(mask_np, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)

                    v_indices, u_indices = np.where(mask_np == 1)
                    if len(u_indices) == 0:
                        continue

                    Z_pixels = self.latest_depth_img[v_indices, u_indices].astype(float)
                    valid_mask = Z_pixels > 0
                    if not np.any(valid_mask):
                        continue
                    
                    Z_vals = Z_pixels[valid_mask]

                    # 📐 3D (X, Y) 물리 공간 좌표를 이용한 수직 정사영 면적 구하기
                    contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if not contours: 
                        continue
                    contour = max(contours, key=cv2.contourArea) #원래는 contour = contours[0]  
                    cv2.drawContours(annotated_frame, [contour], -1, (255, 0, 255), 2)  # 진단용
                    
                    real_world_points = []
                    for pt in contour:
                        u_px, v_px = pt[0][0], pt[0][1]
                        z_val = float(self.latest_depth_img[v_px, u_px])
                        if z_val <= 0:
                            z_val = np.median(Z_vals)
                        
                        x_mm = (u_px - self.cx) * z_val / self.fx
                        y_mm = (v_px - self.cy) * z_val / self.fy
                        real_world_points.append([x_mm, y_mm])

                    real_world_points = np.array(real_world_points, dtype=np.float32)
                    real_area_mm2 = cv2.contourArea(real_world_points)

                    # 📐 [장축 / 단축 물리 길이 계산 파트]
                    real_major_axis_mm = 0.0
                    real_minor_axis_mm = 0.0
                    if len(real_world_points) >= 5:
                        _, (d1, d2), _ = cv2.fitEllipse(real_world_points)
                        real_major_axis_mm = max(d1, d2)
                        real_minor_axis_mm = min(d1, d2)

                    # 중심점 및 Z_center 구하기
                    M = cv2.moments(contour)
                    if M["m00"] == 0: continue
                    cx_px = int(M["m10"] / M["m00"])
                    cy_px = int(M["m01"] / M["m00"])

                    Z_center = float(self.latest_depth_img[cy_px, cx_px])
                    if Z_center <= 0:
                        Z_center = np.median(Z_vals)

                

                    X_center = (cx_px - self.cx) * Z_center / self.fx
                    Y_center = (cy_px - self.cy) * Z_center / self.fy

                    # 🍩 D435i 화각 및 원거리 촬영 환경을 반영한 도넛 영역 확장 파트
                    kernel = np.ones((15, 15), np.uint8)  # D435i에 맞춰 커널 크기 최적화 (35 -> 15)
                    dilated_mask = cv2.dilate(mask_np, kernel, iterations=1)
                    boundary_mask = cv2.bitwise_xor(dilated_mask, mask_np)
                    overlay[boundary_mask == 1] = [0, 255, 255]

                    floor_depths = self.latest_depth_img[boundary_mask == 1]
                    valid_floor_depths = floor_depths[floor_depths > 0]
                    if len(valid_floor_depths) == 0: continue
                    Z_floor = np.median(valid_floor_depths)

                    real_height_mm = Z_floor - Z_center
                    if real_height_mm < 0: real_height_mm = 0.0

                    # 진단용
                    if i == 0:
                        self.get_logger().info(
                            f"[진단] Z_floor:{Z_floor:.1f} Z_center:{Z_center:.1f} H:{Z_floor-Z_center:.1f}")

                    # 🍇 [부피 추정 연산] 회전 타원체 모델 (Volume = 2/3 * Area * Height)
                    real_volume_cm3 = (2.0 / 3.0) * real_area_mm2 * real_height_mm / 1000.0

                    # ⚖️ [무게 연산 파트] 무게(g) = 부피(cm3) * 지정된 밀도(g/cm3)
                    real_weight_g = real_volume_cm3 * self.TOMATO_DENSITY

                    cv2.circle(annotated_frame, (cx_px, cy_px), 5, (0, 0, 255), -1)
                    cv2.putText(
                        annotated_frame, 
                        f"A:{real_area_mm2:.1f} H:{real_height_mm:.1f} V:{real_volume_cm3:.1f} cm3", 
                        (cx_px - 100, cy_px - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 2
                    )

                    # 📊 1초 버퍼 데이터 축적
                    if i not in self.tomato_buffer:
                        self.tomato_buffer[i] = {
                            'areas': [], 'heights': [], 'xs': [], 'ys': [], 'zs': [],
                            'z_floors': [], 'z_centers': [], 'volumes': [],
                            'majors': [], 'minors': [], 'weights': []
                        }
                    self.tomato_buffer[i]['areas'].append(real_area_mm2)
                    self.tomato_buffer[i]['heights'].append(real_height_mm)
                    self.tomato_buffer[i]['xs'].append(X_center)
                    self.tomato_buffer[i]['ys'].append(Y_center)
                    self.tomato_buffer[i]['zs'].append(Z_center)
                    self.tomato_buffer[i]['z_floors'].append(Z_floor)
                    self.tomato_buffer[i]['z_centers'].append(Z_center)
                    self.tomato_buffer[i]['volumes'].append(real_volume_cm3)
                    self.tomato_buffer[i]['majors'].append(real_major_axis_mm)
                    self.tomato_buffer[i]['minors'].append(real_minor_axis_mm)
                    self.tomato_buffer[i]['weights'].append(real_weight_g)

            alpha = 0.4
            annotated_frame = cv2.addWeighted(overlay, alpha, annotated_frame, 1 - alpha, 0)

            current_time = time.time()
            if current_time - self.last_log_time >= 1.0:
                if self.tomato_buffer:
                    for idx, data in self.tomato_buffer.items():
                        areas, heights, volumes, weights = data['areas'], data['heights'], data['volumes'], data['weights']
                        xs, ys, zs = data['xs'], data['ys'], data['zs']
                        z_floors, z_centers = data['z_floors'], data['z_centers']
                        majors, minors = data['majors'], data['minors']
                        
                        if len(areas) >= 4:
                            p_low, p_high = 15, 85
                            
                            a_l, a_h = np.percentile(areas, p_low), np.percentile(areas, p_high)
                            h_l, h_h = np.percentile(heights, p_low), np.percentile(heights, p_high)
                            v_l, v_h = np.percentile(volumes, p_low), np.percentile(volumes, p_high)
                            w_l, w_h = np.percentile(weights, p_low), np.percentile(weights, p_high)
                            maj_l, maj_h = np.percentile(majors, p_low), np.percentile(majors, p_high)
                            min_l, min_h = np.percentile(minors, p_low), np.percentile(minors, p_high)
                            x_l, x_h = np.percentile(xs, p_low), np.percentile(xs, p_high)
                            y_l, y_h = np.percentile(ys, p_low), np.percentile(ys, p_high)
                            z_l, z_h = np.percentile(zs, p_low), np.percentile(zs, p_high)
                            zf_l, zf_h = np.percentile(z_floors, p_low), np.percentile(z_floors, p_high)
                            zc_l, zc_h = np.percentile(z_centers, p_low), np.percentile(z_centers, p_high)
                            
                            f_areas = [a for a in areas if a_l <= a <= a_h]
                            f_heights = [h for h in heights if h_l <= h <= h_h]
                            f_volumes = [v for v in volumes if v_l <= v <= v_h]
                            f_weights = [w for w in weights if w_l <= w <= w_h]
                            f_majors = [m for m in majors if maj_l <= m <= maj_h]
                            f_minors = [m for m in minors if min_l <= m <= min_h]
                            f_xs = [x for x in xs if x_l <= x <= x_h]
                            f_ys = [y for y in ys if y_l <= y <= y_h]
                            f_zs = [z for z in zs if z_l <= z <= z_h]
                            f_z_floors = [zf for zf in z_floors if zf_l <= zf <= zf_h]
                            f_z_centers = [zc for zc in z_centers if zc_l <= zc <= zc_h]
                            
                            final_area = np.mean(f_areas) if f_areas else np.mean(areas)
                            final_height = np.mean(f_heights) if f_heights else np.mean(heights)
                            final_volume = np.mean(f_volumes) if f_volumes else np.mean(volumes)
                            final_weight = np.mean(f_weights) if f_weights else np.mean(weights)
                            final_major = np.mean(f_majors) if f_majors else np.mean(majors)
                            final_minor = np.mean(f_minors) if f_minors else np.mean(minors)
                            final_x = np.mean(f_xs) if f_xs else np.mean(xs)
                            final_y = np.mean(f_ys) if f_ys else np.mean(ys)
                            final_z = np.mean(f_zs) if f_zs else np.mean(zs)
                            final_z_floor = np.mean(f_z_floors) if f_z_floors else np.mean(z_floors)
                            final_z_center = np.mean(f_z_centers) if f_z_centers else np.mean(z_centers)
                        else:
                            final_area = np.mean(areas) if areas else 0.0
                            final_height = np.mean(heights) if heights else 0.0
                            final_volume = np.mean(volumes) if volumes else 0.0
                            final_weight = np.mean(weights) if weights else 0.0
                            final_major = np.mean(majors) if majors else 0.0
                            final_minor = np.mean(minors) if minors else 0.0
                            final_x = np.mean(xs) if xs else 0.0
                            final_y = np.mean(ys) if ys else 0.0
                            final_z = np.mean(zs) if zs else 0.0
                            final_z_floor = np.mean(z_floors) if z_floors else 0.0
                            final_z_center = np.mean(z_centers) if z_centers else 0.0

                        if final_area > 0:
                            # 📤 토픽 발행 (Topic Publishing)
                            point_msg = Point()
                            point_msg.x = float(final_x)
                            point_msg.y = float(final_y)
                            point_msg.z = float(final_z)
                            self.pose_pub.publish(point_msg)

                            weight_msg = Float32()
                            weight_msg.data = float(final_weight)
                            self.weight_pub.publish(weight_msg)

                            self.get_logger().info(
                                f"📊 [D435i 1초 평균 데이터] 토마토 #{idx+1} -> "
                                f"면적: {final_area:.1f} mm² | 높이: {final_height:.1f} mm | 🍇 부피: {final_volume:.1f} cm³ | ⚖️ 예상 무게: {final_weight:.1f} g\n"
                                f"   📍 [중심점 3D 좌표] X: {final_x:.1f} mm | Y: {final_y:.1f} mm | Z: {final_z:.1f} mm\n"
                                f"   📐 [형태 분석] 실제 장축(Major): {final_major:.1f} mm | 실제 단축(Minor): {final_minor:.1f} mm\n"
                                f"   🔍 [원시값 검증] Z_floor(바닥): {final_z_floor:.1f} mm | Z_center(꼭대기): {final_z_center:.1f} mm"
                            )
                    self.tomato_buffer.clear()
                else:
                    self.get_logger().info("화면에 토마토가 없습니다... ❌", throttle_duration_sec=1.0)
                self.last_log_time = current_time

            cv2.imshow("Tomato Area Visualizer (D435i)", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 연산 에러: {e}")
            return

def main():
    rclpy.init()
    node = TomatoHeightAndAreaMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()