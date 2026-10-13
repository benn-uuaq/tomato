import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time


class TomatoHeightAndAreaMeasurer(Node):
    def __init__(self):
        super().__init__('tomato_area_d435i_v2')

        # ==============================================================
        # 파라미터
        # ==============================================================
        self.declare_parameter('capture_duration', 1.0)   # 1회 촬영 측정 시간 (초)
        self.declare_parameter('match_dist_px', 60.0)     # 동일 토마토 판정 거리 (픽셀)
        self.declare_parameter('debug_depth', False)      # Z_floor/Z_center 진단 로그
        # 미리보기 창(cv2.imshow) 사용 여부. 'auto' 면 DISPLAY 가 있을 때만 켠다.
        # SSH 로 원격 실행하면 디스플레이가 없어 Qt 가 그대로 죽어버리기 때문에
        # (exit -6), 화면이 없는 환경에서는 창 없이 계산만 하게 한다.
        self.declare_parameter('show_window', 'auto')     # auto | true | false
        self.capture_duration = float(self.get_parameter('capture_duration').value)
        self.match_dist = float(self.get_parameter('match_dist_px').value)
        self.debug_depth = bool(self.get_parameter('debug_depth').value)
        self.show_window = self._resolve_show_window()

        # 모델 경로는 파라미터로 받는다. 워크스페이스를 옮겨도 코드를 고치지
        # 않게 하기 위해서다(기본값은 환경변수 TOMATO_MODEL, 그다음 기존 경로).
        default_model = os.environ.get(
            'TOMATO_MODEL', os.path.expanduser('~/doosan_manipulator/tomato_v1.pt'))
        self.declare_parameter('model_path', default_model)
        model_path = str(self.get_parameter('model_path').value)
        self.get_logger().info(f'YOLO 토마토 segmentation 모델 로드 중... ({model_path})')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.get_logger().info('YOLO 모델 로드 완료!')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # 토마토 밀도 상수 (g/cm3)
        self.TOMATO_DENSITY = 1.086

        # ==============================================================
        # 구독자
        # ==============================================================
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile_sensor_data
        )
        # 촬영 트리거 (로봇 -> 비전)
        self.trigger_sub = self.create_subscription(
            Empty, '/tomato/capture', self.trigger_callback, 10
        )

        # ==============================================================
        # 발행자
        # ==============================================================
        self.pose_pub = self.create_publisher(PoseArray, '/tomato/center_positions', 10)
        self.weight_pub = self.create_publisher(Float32MultiArray, '/tomato/estimated_weights', 10)
        self.height_pub = self.create_publisher(Float32MultiArray, '/tomato/heights', 10)

        # ==============================================================
        # 상태
        # ==============================================================
        self.latest_depth_img = None
        self.frame_id = 'camera_color_optical_frame'

        self.capturing = False          # False = 대기(YOLO 미실행), True = 측정 중
        self.capture_start = 0.0
        self.tracks = {}                # tid -> {'cx','cy','data'}
        self.next_tid = 0

        self.get_logger().info(
            f'=== [D435i v2] 트리거 대기 중 === '
            f'( ros2 topic pub --once /tomato/capture std_msgs/Empty "{{}}" )'
        )

    # ==================================================================
    # 콜백
    # ==================================================================
    def _resolve_show_window(self) -> bool:
        """미리보기 창을 켤지 정한다. 'auto' 면 DISPLAY 가 있을 때만 켠다."""
        mode = str(self.get_parameter('show_window').value).strip().lower()
        if mode in ('false', '0', 'no', 'off'):
            return False
        if mode in ('true', '1', 'yes', 'on'):
            return True
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        if not has_display:
            self.get_logger().info(
                '화면이 없어 미리보기 창 없이 동작합니다(계산·발행은 그대로).')
        return has_display

    def _show(self, window_name, frame):
        """미리보기 창이 켜져 있을 때만 그린다."""
        if not self.show_window:
            return
        cv2.imshow(window_name, frame)
        cv2.waitKey(1)

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"카메라 파라미터 로드 완료 -> fx: {self.fx:.1f}, fy: {self.fy:.1f}, "
                f"cx: {self.cx:.1f}, cy: {self.cy:.1f}")

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"깊이 이미지 변환 실패: {e}")

    def trigger_callback(self, msg):
        if self.capturing:
            self.get_logger().warn('이미 촬영 중입니다. 트리거를 무시합니다.')
            return
        if self.latest_depth_img is None or self.fx is None:
            self.get_logger().error('깊이 영상 또는 카메라 파라미터가 아직 준비되지 않았습니다.')
            return

        self.tracks.clear()
        self.next_tid = 0
        self.capture_start = time.time()
        self.capturing = True
        self.get_logger().info(f'📸 촬영 시작 ({self.capture_duration:.1f}초 측정)')

    # ==================================================================
    # 메인 이미지 콜백
    # ==================================================================
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_image is None or cv_image.size == 0:
                return
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id

            # ---------- 대기 상태: YOLO 미실행, 화면만 출력 ----------
            if not self.capturing:
                cv2.putText(cv_image, "IDLE - waiting for /tomato/capture",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
                self._show("Tomato Area Visualizer (D435i v2)", cv_image)
                return

            # ---------- 측정 상태 ----------
            if self.latest_depth_img is None or self.fx is None:
                return

            annotated_frame = self.measure(cv_image)

            elapsed = time.time() - self.capture_start
            cv2.putText(annotated_frame, f"CAPTURING {elapsed:.1f}/{self.capture_duration:.1f}s",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            self._show("Tomato Area Visualizer (D435i v2)", annotated_frame)

            if elapsed >= self.capture_duration:
                self.capturing = False
                self.finalize_and_publish()

        except Exception as e:
            self.get_logger().error(f"❌ 연산 에러: {e}")
            return

    # ==================================================================
    # 1프레임 측정 및 버퍼 축적
    # ==================================================================
    def measure(self, cv_image):
        results = self.model(cv_image, verbose=False, conf=0.25, retina_masks=True)
        result = results[0]
        annotated_frame = result.plot()
        overlay = annotated_frame.copy()

        if result.masks is None:
            return annotated_frame

        for mask in result.masks.data:
            mask_np = mask.cpu().numpy().astype(np.uint8)
            if mask_np.shape != cv_image.shape[:2]:
                mask_np = cv2.resize(mask_np, (cv_image.shape[1], cv_image.shape[0]),
                                     interpolation=cv2.INTER_NEAREST)

            v_indices, u_indices = np.where(mask_np == 1)
            if len(u_indices) == 0:
                continue

            Z_pixels = self.latest_depth_img[v_indices, u_indices].astype(float)
            valid_mask = Z_pixels > 0
            if not np.any(valid_mask):
                continue
            Z_vals = Z_pixels[valid_mask]

            # --- 3D 물리 공간 좌표 기반 정사영 면적 ---
            contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)

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

            # --- 장축 / 단축 ---
            real_major_axis_mm = 0.0
            real_minor_axis_mm = 0.0
            if len(real_world_points) >= 5:
                _, (d1, d2), _ = cv2.fitEllipse(real_world_points)
                real_major_axis_mm = max(d1, d2)
                real_minor_axis_mm = min(d1, d2)

            # --- 중심점 및 Z_center ---
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx_px = int(M["m10"] / M["m00"])
            cy_px = int(M["m01"] / M["m00"])

            Z_center = float(self.latest_depth_img[cy_px, cx_px])
            if Z_center <= 0:
                Z_center = np.median(Z_vals)

            X_center = (cx_px - self.cx) * Z_center / self.fx
            Y_center = (cy_px - self.cy) * Z_center / self.fy

            # --- 도넛(링) 영역으로 바닥면 추정 ---
            kernel = np.ones((15, 15), np.uint8)
            dilated_mask = cv2.dilate(mask_np, kernel, iterations=1)
            boundary_mask = cv2.bitwise_xor(dilated_mask, mask_np)
            overlay[boundary_mask == 1] = [0, 255, 255]

            floor_depths = self.latest_depth_img[boundary_mask == 1]
            valid_floor_depths = floor_depths[floor_depths > 0]
            if len(valid_floor_depths) == 0:
                continue
            Z_floor = np.median(valid_floor_depths)

            real_height_mm = Z_floor - Z_center
            if real_height_mm < 0:
                real_height_mm = 0.0

            if self.debug_depth:
                self.get_logger().info(
                    f"[진단] px:({cx_px},{cy_px}) Z_floor:{Z_floor:.1f} "
                    f"Z_center:{Z_center:.1f} H:{real_height_mm:.1f}")

            # --- 부피 / 무게 ---
            real_volume_cm3 = (2.0 / 3.0) * real_area_mm2 * real_height_mm / 1000.0
            real_weight_g = real_volume_cm3 * self.TOMATO_DENSITY

            cv2.circle(annotated_frame, (cx_px, cy_px), 5, (0, 0, 255), -1)
            cv2.putText(
                annotated_frame,
                f"A:{real_area_mm2:.1f} H:{real_height_mm:.1f} V:{real_volume_cm3:.1f} cm3",
                (cx_px - 100, cy_px - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 2
            )

            # --- 중심 좌표 기반 개체 매칭 후 버퍼 축적 ---
            tid = self.match_track(cx_px, cy_px)
            d = self.tracks[tid]['data']
            d['areas'].append(real_area_mm2)
            d['heights'].append(real_height_mm)
            d['xs'].append(X_center)
            d['ys'].append(Y_center)
            d['zs'].append(Z_center)
            d['z_floors'].append(Z_floor)
            d['z_centers'].append(Z_center)
            d['volumes'].append(real_volume_cm3)
            d['majors'].append(real_major_axis_mm)
            d['minors'].append(real_minor_axis_mm)
            d['weights'].append(real_weight_g)

        alpha = 0.4
        return cv2.addWeighted(overlay, alpha, annotated_frame, 1 - alpha, 0)

    # ==================================================================
    # 개체 매칭 (정지 대상이므로 중심 좌표 최근접으로 판정)
    # ==================================================================
    def match_track(self, cx_px, cy_px):
        best_tid, best_d = None, self.match_dist
        for tid, tr in self.tracks.items():
            dist = np.hypot(tr['cx'] - cx_px, tr['cy'] - cy_px)
            if dist < best_d:
                best_tid, best_d = tid, dist

        if best_tid is None:
            best_tid = self.next_tid
            self.next_tid += 1
            self.tracks[best_tid] = {
                'cx': cx_px, 'cy': cy_px,
                'data': {
                    'areas': [], 'heights': [], 'xs': [], 'ys': [], 'zs': [],
                    'z_floors': [], 'z_centers': [], 'volumes': [],
                    'majors': [], 'minors': [], 'weights': []
                }
            }
        else:
            self.tracks[best_tid]['cx'] = cx_px
            self.tracks[best_tid]['cy'] = cy_px
        return best_tid

    # ==================================================================
    # 백분위 필터 후 평균
    # ==================================================================
    @staticmethod
    def filtered_mean(vals, p_low=15, p_high=85):
        if not vals:
            return 0.0
        if len(vals) < 4:
            return float(np.mean(vals))
        lo, hi = np.percentile(vals, p_low), np.percentile(vals, p_high)
        f = [v for v in vals if lo <= v <= hi]
        return float(np.mean(f)) if f else float(np.mean(vals))

    # ==================================================================
    # 측정 종료 -> 전체 목록 1회 발행
    # ==================================================================
    def finalize_and_publish(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id

        weight_msg = Float32MultiArray()
        height_msg = Float32MultiArray() 

        summaries = []
        for tid, tr in sorted(self.tracks.items()):
            data = tr['data']
            if not data['areas']:
                continue
            s = {k: self.filtered_mean(v) for k, v in data.items()}
            if s['areas'] <= 0:
                continue
            s['tid'] = tid
            s['n'] = len(data['areas'])
            summaries.append(s)

        for s in summaries:
            p = Pose()
            p.position.x = s['xs'] / 1000.0      # mm -> m
            p.position.y = s['ys'] / 1000.0
            p.position.z = s['zs'] / 1000.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)
            weight_msg.data.append(float(s['weights']))
            height_msg.data.append(float(s['heights']) / 1000.0)

        # 검출 0개여도 빈 목록을 발행해 로봇이 대기 상태에 빠지지 않도록 함
        self.pose_pub.publish(pose_array)
        self.weight_pub.publish(weight_msg)
        self.height_pub.publish(height_msg)

        if not summaries:
            self.get_logger().info('📸 촬영 완료 -> 검출된 토마토 없음 (빈 목록 발행)')
        else:
            self.get_logger().info(f'📸 촬영 완료 -> 토마토 {len(summaries)}개 발행 (frame: {self.frame_id})')
            for i, s in enumerate(summaries):
                self.get_logger().info(
                    f"  [{i}] 면적: {s['areas']:.1f} mm² | 높이: {s['heights']:.1f} mm | "
                    f"부피: {s['volumes']:.1f} cm³ | 무게: {s['weights']:.1f} g  (프레임 {s['n']}개)\n"
                    f"      중심 3D  X: {s['xs']:.1f} / Y: {s['ys']:.1f} / Z: {s['zs']:.1f} mm "
                    f"-> 발행값 ({s['xs']/1000.0:.4f}, {s['ys']/1000.0:.4f}, {s['zs']/1000.0:.4f}) m\n"
                    f"      장축: {s['majors']:.1f} mm | 단축: {s['minors']:.1f} mm | "
                    f"Z_floor: {s['z_floors']:.1f} mm | Z_center: {s['z_centers']:.1f} mm")

        self.tracks.clear()
        self.get_logger().info('=== 트리거 대기 중 ===')


def main():
    rclpy.init()
    node = TomatoHeightAndAreaMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass      # 화면이 없는 환경에서는 창 자체가 없다
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
