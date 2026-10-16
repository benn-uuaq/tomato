#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
토마토 비전 통합 노드 (D435i)

네 가지 기능을 한 프로세스에서 제공한다. 모델 2개를 시작 시 모두 올려두고
어떤 트리거가 왔는지에 따라 골라 쓴다. 카메라 창은 하나만 뜬다.

  트리거                        모드          사용 모델        발행 토픽
  ------------------------------------------------------------------------------
  /tomato/pick_capture        위치(전체)    pick_model      /tomato/pick_positions
                                                            /tomato/pick_weights
  /tomato/nearest_capture     위치(최근접)  pick_model      /tomato/nearest_position
                                                            /tomato/nearest_weight
  /tomato/middle_capture      위치(중앙)    pick_model      /tomato/middle_position
                                                            /tomato/middle_weight
  /tomato/capture             무게 추정     weight_model    /tomato/center_positions
                                                            /tomato/estimated_weights
                                                            /tomato/heights

구성 배경:
  - 위치 인식은 박스에 빽빽하게 담긴 토마토를 분리하는 데 강한 모델(best.pt)을 쓴다.
    꼭지가 마스크에서 제외되므로 면적 기반 무게 추정에는 적합하지 않다.
  - 무게 추정은 꼭지를 포함해 학습한 기존 모델(tomato_v1.pt)을 쓴다.
    새 박스로 옮겨 서로 떨어진 상태에서 측정하는 것을 전제로 한다.
  - 두 모델을 한 프로세스에 두면 CUDA 컨텍스트가 하나만 생성되고
    카메라 구독도 한 번만 이뤄진다.

위치 모드의 보조 무게 추정 (v3 — 면적 x 반높이 타원체):
  바닥이 보이지 않는 박스 안에서는 링 바닥 추정이 불가능해 전체 높이를
  잴 수 없다. 그래서 마스크 자체에서 두 가지를 뽑아 타원체로 본다.

    s     = (rho/f) / sqrt(1 + (rho/f)^2)     sin(각반경)
    offset= Z_rim * s^2 / (1 - s^2)           접선면 -> 중심면 보정
    D     = Z_rim + offset                    중심 깊이
    A_mm2 = area_px * D^2 / (fx * fy)         윤곽선 내부 전체 면적 (적도 단면)
    c     = (Z_rim - Z_center) + offset       적도면에서 꼭대기까지 = 반높이
    V     = (4/3) * A * c                     mm^3 -> cm^3
    W     = V * ellipsoid_gain                기본값 = 밀도 1.086

  Z_rim    : 윤곽선 픽셀 깊이의 중위값. 실루엣 테두리는 적도이므로 이 값이
             중심 깊이이자 면적의 기준 깊이다.
  Z_center : 중심 픽셀 주변 작은 원판의 깊이 중위값 (과육 꼭대기).

  접선면 보정 (tangent_correct):
    실루엣 테두리는 중심면이 아니라 접점이 이루는 원의 평면에서 생기고,
    그 평면은 중심보다 D*sin^2(theta) 앞에 있다. 보정하지 않으면 c 가 그만큼
    작게 나온다. offset 자체는 2~3 mm 지만 c 가 20 mm 안팎이라 비율로는
    15% 를 넘는다. 합성 타원체(장축 55.7 / 단축 49.4 / 높이 38.6, 중심 300mm)
    로 검산하면 보정 전 부피 오차 -19%, 보정 후 -5% 였다.

  타원체 부피는 V = (4/3)*pi*a*b*c 이고 A = pi*a*b 이므로 V = (4/3)*A*c 다.
  전체 높이 h = 2c 를 쓰면 V = (2/3)*A*h 와 같은 식이지만, 여기서 재는 것은
  반높이 c 이므로 (4/3) 계수를 쓴다. 아래쪽 절반은 가려져 보이지 않으므로
  상하 대칭을 가정한다.

  면적은 윤곽선 내부를 전부 센다. RETR_EXTERNAL 로 뽑은 윤곽선이라 꼭지가
  마스크에서 제외되어 생긴 내부 구멍은 메워진 상태다.

  Gain 이 v1/v2 와 다르다. v1/v2 의 0.667 은 밀도 x 형상계수(0.614)였는데,
  형상계수는 구로 근사할 때의 과대평가를 보정하는 값이다. v3 는 실제 면적과
  높이를 재므로 형상이 이미 반영되어 있고, 기본값은 밀도 하나다.
  저울 실측 후 ellipsoid_gain 만 조정하면 전체가 보정된다.

  대체 경로:
    윤곽선 깊이의 유효 비율이 min_rim_valid 미만이거나 c <= 0 이면 타원체
    계산을 신뢰할 수 없으므로 v1/v2 의 구체 근사로 되돌아간다. 그 경우
    weight_gain(0.667) 이 쓰이고 로그에 '구체 대체' 로 표시된다.
    weight_mode 를 'sphere' 로 두면 항상 구체 근사를 쓴다. (v2 와 동일)

  지름 범위 검사는 면적을 원으로 환산한 등가 지름을 쓴다.
    d_eq = 2 * sqrt(A / pi)
  외접원 지름은 마스크가 길쭉할 때 면적과 크게 어긋나기 때문이다.

위치(중앙) 모드:
  화면을 3x3 으로 나눴을 때 가운데 칸 안에 중심이 있는 개체만 후보로 두고,
  그중 카메라에 가장 가까운 하나의 좌표와 무게를 발행한다. 그리퍼가 화면
  중앙을 바라보도록 로봇을 미리 보냈을 때, 시야 주변의 토마토를 잡으러
  멀리 움직이지 않게 하려는 용도다. 항목은 0 개 또는 1 개다.
  가운데 칸의 비율은 center_region_frac 으로 조정한다. (1/3 = 3x3)

대기 상태에서는 어떤 모델도 추론하지 않고 컬러 영상만 표시한다.
촬영 중 다른 트리거가 들어오면 무시한다.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Float32MultiArray
from ultralytics import YOLO

WINDOW_NAME = 'Tomato Vision (D435i)'

MODE_IDLE = 'idle'
MODE_PICK_ALL = 'pick_all'
MODE_PICK_NEAREST = 'pick_nearest'
MODE_PICK_CENTER = 'pick_center'
MODE_WEIGHT = 'weight'


class TomatoVisionNode(Node):
    def __init__(self):
        super().__init__('tomato_vision_d435i')

        # ==============================================================
        # 파라미터
        # ==============================================================
        # 모델 경로
        self.declare_parameter('pick_model_path',
                               '/home/smartfarm/doosan_manipulator/best.pt')
        self.declare_parameter('weight_model_path',
                               '/home/smartfarm/doosan_manipulator/tomato_v1.pt')

        # 촬영 시간 (모드별)
        self.declare_parameter('pick_capture_duration', 3.0)
        self.declare_parameter('weight_capture_duration', 3.0)

        # 공통 / 위치 모드
        self.declare_parameter('pick_conf', 0.25)            # 위치 모델 신뢰도 임계값
        self.declare_parameter('pick_match_dist_px', 40.0)   # 동일 토마토 판정 거리
        self.declare_parameter('min_frames', 3)              # 최소 검출 프레임 수
        self.declare_parameter('min_area_px', 500.0)         # 최소 마스크 면적
        self.declare_parameter('close_px', 0)                # 모폴로지 닫기 커널 (0 = 미사용)
        self.declare_parameter('center_mode', 'circle')      # 'circle' | 'centroid'
        self.declare_parameter('depth_percentile', 10.0)     # Z 산출 백분위
        self.declare_parameter('sort_by_depth', True)        # 전체 목록을 가까운 순으로 정렬
        self.declare_parameter('min_fill', 0.0)
        # 중앙 모드에서 가운데 칸이 차지하는 비율 (1/3 = 3x3 분할의 중앙)
        self.declare_parameter('center_region_frac', 1.0 / 3.0)              # 최근접 모드 최소 원형도 (0 = 미사용)

        # 위치 모드의 보조 무게 추정 (구체 근사)
        self.declare_parameter('sphere_weight', True)        # 구체 근사 무게 추정 사용
        self.declare_parameter('weight_gain', 0.667)         # 밀도 x 형상계수 (g/cm3)
        # 반지름 산출에 쓸 깊이: 'rim' = 윤곽선 깊이, 'top' = 윗면 깊이 + 각반경 보정
        self.declare_parameter('radius_depth_mode', 'rim')
        self.declare_parameter('min_rim_valid', 0.3)         # 윤곽선 깊이 최소 유효 비율
        # 무게 산출 방식: 'ellipsoid' = 면적 x 반높이 (v3), 'sphere' = 외접원만 (v1/v2)
        self.declare_parameter('weight_mode', 'ellipsoid')
        # 타원체 모드 Gain. 면적과 높이를 실제로 재므로 형상계수가 이미 반영되어
        # 있고, 기본값은 밀도(1.086) 하나다. 저울 실측 후 이 값만 조정한다.
        self.declare_parameter('ellipsoid_gain', 1.086)
        # 중심 깊이를 읽을 원판 반지름 (외접원 반지름 대비 비율)
        self.declare_parameter('center_patch_frac', 0.15)
        # 접선면 보정 — 실루엣 평면이 중심면보다 앞에 있는 것을 되돌린다
        self.declare_parameter('tangent_correct', True)
        # 지름 범위 검사 — 박스 벽 등 오검출 제거 (0 = 미사용)
        self.declare_parameter('min_diameter_mm', 35.0)
        self.declare_parameter('max_diameter_mm', 80.0)

        # 무게 모드
        self.declare_parameter('weight_conf', 0.25)          # 무게 모델 신뢰도 임계값
        self.declare_parameter('weight_match_dist_px', 60.0)  # 동일 토마토 판정 거리
        self.declare_parameter('ring_px', 15)                # 바닥 추정 링 커널 크기

        # 표시 / 진단
        self.declare_parameter('enable_viz', True)
        # YOLO 마스크 색칠 오버레이. GPU 에서 (N,H,W) 텐서를 누적곱하므로
        # 개체 수가 많으면 메모리를 크게 먹는다. (25개 / 1280x720 에서 180MB 이상)
        # 젯슨처럼 메모리가 빠듯하면 끄는 것을 권한다. 꺼도 외접원 / 중심점 /
        # 무게 표시는 그대로 그려지므로 검증에 필요한 정보는 유지된다.
        self.declare_parameter('viz_masks', False)
        self.declare_parameter('debug', False)               # 위치 모드 개체별 로그
        self.declare_parameter('debug_depth', False)         # 무게 모드 Z_floor/Z_center 로그

        p = self.get_parameter
        pick_model_path = p('pick_model_path').value
        weight_model_path = p('weight_model_path').value
        self.pick_duration = float(p('pick_capture_duration').value)
        self.weight_duration = float(p('weight_capture_duration').value)
        self.pick_conf = float(p('pick_conf').value)
        self.pick_match_dist = float(p('pick_match_dist_px').value)
        self.min_frames = int(p('min_frames').value)
        self.min_area_px = float(p('min_area_px').value)
        self.close_px = int(p('close_px').value)
        self.center_mode = str(p('center_mode').value)
        self.depth_pct = float(p('depth_percentile').value)
        self.sort_by_depth = bool(p('sort_by_depth').value)
        self.min_fill = float(p('min_fill').value)
        self.center_region_frac = float(p('center_region_frac').value)
        self.sphere_weight = bool(p('sphere_weight').value)
        self.weight_gain = float(p('weight_gain').value)
        self.radius_depth_mode = str(p('radius_depth_mode').value)
        self.min_rim_valid = float(p('min_rim_valid').value)
        self.weight_mode = str(p('weight_mode').value)
        self.ellipsoid_gain = float(p('ellipsoid_gain').value)
        self.center_patch_frac = float(p('center_patch_frac').value)
        self.tangent_correct = bool(p('tangent_correct').value)
        self.min_diam = float(p('min_diameter_mm').value)
        self.max_diam = float(p('max_diameter_mm').value)
        self.weight_conf = float(p('weight_conf').value)
        self.weight_match_dist = float(p('weight_match_dist_px').value)
        self.ring_px = int(p('ring_px').value)
        self.enable_viz = bool(p('enable_viz').value)
        self.viz_masks = bool(p('viz_masks').value)
        self.debug = bool(p('debug').value)
        self.debug_depth = bool(p('debug_depth').value)

        self.close_kernel = (np.ones((self.close_px, self.close_px), np.uint8)
                             if self.close_px > 0 else None)
        self.ring_kernel = np.ones((self.ring_px, self.ring_px), np.uint8)
        self.center_region_frac = min(max(self.center_region_frac, 0.05), 1.0)
        if self.center_mode not in ('circle', 'centroid'):
            self.get_logger().warn(
                f"center_mode '{self.center_mode}' 를 알 수 없어 'circle' 로 진행합니다.")
            self.center_mode = 'circle'
        if self.radius_depth_mode not in ('rim', 'top'):
            self.get_logger().warn(
                f"radius_depth_mode '{self.radius_depth_mode}' 를 알 수 없어 "
                f"'rim' 으로 진행합니다.")
            self.radius_depth_mode = 'rim'
        if self.weight_mode not in ('ellipsoid', 'sphere'):
            self.get_logger().warn(
                f"weight_mode '{self.weight_mode}' 를 알 수 없어 "
                f"'ellipsoid' 로 진행합니다.")
            self.weight_mode = 'ellipsoid'
        self.center_patch_frac = min(max(self.center_patch_frac, 0.02), 0.9)

        # 토마토 밀도 상수 (g/cm3) — 실측값
        self.TOMATO_DENSITY = 1.086

        # ==============================================================
        # 모델 2개 로드
        # ==============================================================
        self.get_logger().info(f'위치 인식 모델 로드 중... ({pick_model_path})')
        self.pick_model = YOLO(pick_model_path)
        self.get_logger().info(f'무게 추정 모델 로드 중... ({weight_model_path})')
        self.weight_model = YOLO(weight_model_path)
        self.bridge = CvBridge()
        self.get_logger().info('모델 2개 로드 완료')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ==============================================================
        # 구독자 (카메라는 한 번만 구독)
        # ==============================================================
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.image_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            self.info_callback, qos_profile_sensor_data)

        # 트리거 3개
        self.create_subscription(
            Empty, '/tomato/pick_capture',
            lambda m: self.start_capture(MODE_PICK_ALL), 10)
        self.create_subscription(
            Empty, '/tomato/nearest_capture',
            lambda m: self.start_capture(MODE_PICK_NEAREST), 10)
        self.create_subscription(
            Empty, '/tomato/middle_capture',
            lambda m: self.start_capture(MODE_PICK_CENTER), 10)
        self.create_subscription(
            Empty, '/tomato/capture',
            lambda m: self.start_capture(MODE_WEIGHT), 10)

        # ==============================================================
        # 발행자
        # ==============================================================
        self.pick_all_pub = self.create_publisher(
            PoseArray, '/tomato/pick_positions', 10)
        self.pick_nearest_pub = self.create_publisher(
            PoseArray, '/tomato/nearest_position', 10)
        # 위치 모드의 보조 무게 (구체 근사) — 면적-높이 방식과 구분해 별도 토픽으로 낸다
        self.pick_weight_pub = self.create_publisher(
            Float32MultiArray, '/tomato/pick_weights', 10)
        self.nearest_weight_pub = self.create_publisher(
            Float32MultiArray, '/tomato/nearest_weight', 10)
        # 중앙 모드 — 화면 3x3 가운데 칸에서 최근접 1개
        self.middle_pub = self.create_publisher(
            PoseArray, '/tomato/middle_position', 10)
        self.middle_weight_pub = self.create_publisher(
            Float32MultiArray, '/tomato/middle_weight', 10)
        self.center_pub = self.create_publisher(
            PoseArray, '/tomato/center_positions', 10)
        self.weight_pub = self.create_publisher(
            Float32MultiArray, '/tomato/estimated_weights', 10)
        self.height_pub = self.create_publisher(
            Float32MultiArray, '/tomato/heights', 10)

        # ==============================================================
        # 상태
        # ==============================================================
        self.latest_depth_img = None
        self.img_w = 0
        self.img_h = 0
        self.frame_id = 'camera_color_optical_frame'

        self.mode = MODE_IDLE
        self.capture_start = 0.0
        self.capture_duration = 0.0
        self.frames_done = 0      # 이번 촬영에서 추론을 끝낸 프레임 수
        self.tracks = {}
        self.next_tid = 0

        self.get_logger().info(
            '=== [통합 비전 노드] 트리거 대기 중 ===\n'
            '  위치(전체)   : ros2 topic pub --once /tomato/pick_capture std_msgs/msg/Empty "{}"\n'
            '  위치(최근접) : ros2 topic pub --once /tomato/nearest_capture std_msgs/msg/Empty "{}"\n'
            '  위치(중앙)   : ros2 topic pub --once /tomato/middle_capture std_msgs/msg/Empty "{}"\n'
            '  무게 추정    : ros2 topic pub --once /tomato/capture std_msgs/msg/Empty "{}"')

    # ==================================================================
    # 콜백
    # ==================================================================
    def info_callback(self, msg):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f'카메라 파라미터 로드 완료 -> fx: {self.fx:.1f}, fy: {self.fy:.1f}, '
                f'cx: {self.cx:.1f}, cy: {self.cy:.1f}')

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'깊이 이미지 변환 실패: {e}')

    # ==================================================================
    # 트리거 (3개 모드 공통 진입점)
    # ==================================================================
    def start_capture(self, mode):
        if self.mode != MODE_IDLE:
            self.get_logger().warn(
                f'이미 촬영 중입니다({self.mode}). {mode} 트리거를 무시합니다.')
            return
        if self.latest_depth_img is None or self.fx is None:
            self.get_logger().error('깊이 영상 또는 카메라 파라미터가 아직 준비되지 않았습니다.')
            return

        self.tracks.clear()
        self.next_tid = 0
        self.frames_done = 0
        self.mode = mode
        self.capture_duration = (self.weight_duration if mode == MODE_WEIGHT
                                 else self.pick_duration)
        self.capture_start = time.time()

        label = {MODE_PICK_ALL: '위치(전체)',
                 MODE_PICK_NEAREST: '위치(최근접)',
                 MODE_PICK_CENTER: '위치(중앙)',
                 MODE_WEIGHT: '무게 추정'}[mode]
        self.get_logger().info(
            f'📸 [{label}] 촬영 시작 ({self.capture_duration:.1f}초 측정)')

    # ==================================================================
    # 메인 이미지 콜백
    # ==================================================================
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_image is None or cv_image.size == 0:
                return
            self.img_h, self.img_w = cv_image.shape[:2]
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id

            # ---------- 대기 상태: 어떤 모델도 추론하지 않음 ----------
            if self.mode == MODE_IDLE:
                if self.enable_viz:
                    cv2.putText(cv_image, 'IDLE - waiting for trigger',
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (200, 200, 200), 2)
                    cv2.putText(cv_image,
                                'pick_capture / nearest_capture / capture',
                                (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (200, 200, 200), 1)
                    cv2.imshow(WINDOW_NAME, cv_image)
                    cv2.waitKey(1)
                return

            # ---------- 측정 상태 ----------
            if self.latest_depth_img is None or self.fx is None:
                return

            if self.mode == MODE_WEIGHT:
                annotated = self.measure_weight(cv_image)
                label = 'WEIGHT'
            else:
                annotated = self.detect_pick(cv_image)
                label = {MODE_PICK_ALL: 'PICK-ALL',
                         MODE_PICK_NEAREST: 'PICK-NEAREST',
                         MODE_PICK_CENTER: 'PICK-CENTER'}[self.mode]
            self.frames_done += 1

            elapsed = time.time() - self.capture_start
            if self.enable_viz:
                cv2.putText(annotated,
                            f'{label}  {elapsed:.1f}/{self.capture_duration:.1f}s',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(WINDOW_NAME, annotated)
                cv2.waitKey(1)

            if elapsed >= self.capture_duration:
                finished_mode = self.mode
                self.mode = MODE_IDLE
                if finished_mode == MODE_WEIGHT:
                    self.finalize_weight()
                else:
                    self.finalize_pick(finished_mode)

        except Exception:
            import traceback
            self.get_logger().error(f'연산 에러:\n{traceback.format_exc()}')
            self.mode = MODE_IDLE

    # ==================================================================
    # 표시용 배경 영상 생성
    # ==================================================================
    def render_base(self, cv_image, result):
        """화면에 쓸 배경 프레임을 만든다.

        result.plot() 은 마스크를 GPU 에서 합성하는데, 개체 N 개에 대해
        (N, H, W) 크기의 float 텐서를 만들고 cumprod 까지 수행한다.
        25개 / 1280x720 이면 180MB 이상이고, 젯슨의 공유 메모리에서는
        NvMap ENOMEM -> CUDA 할당자 오류로 이어진다.

        그래서 세 단계로 비용을 낮춘다.
          - 화면을 안 띄우면(enable_viz=False) 아예 호출하지 않는다
          - viz_masks=False 면 마스크 색칠만 끄고 박스/라벨은 유지한다
          - 그래도 실패하면 원본 영상으로 되돌아간다 (측정은 계속)
        """
        if not self.enable_viz:
            return cv_image
        try:
            return result.plot(masks=self.viz_masks)
        except Exception as e:
            self.get_logger().warn(
                f'오버레이 생성 실패 -> 원본 영상으로 대체합니다 ({e})')
            return cv_image.copy()

    # ==================================================================
    # 위치 모드 : 1프레임 검출 및 버퍼 축적
    # ==================================================================
    def detect_pick(self, cv_image):
        # retina_masks=True -> 원본 해상도 마스크 (letterbox 오정렬 방지)
        result = self.pick_model(cv_image, verbose=False, conf=self.pick_conf,
                                 retina_masks=True)[0]
        annotated = self.render_base(cv_image, result)

        if result.masks is None:
            return annotated

        for mask in result.masks.data:
            # Ultralytics 가 돌려주는 masks.data 는 이미 0/1 로 이진화되어 있다.
            # (retina_masks 여부와 무관) 따라서 임계값을 조절해 마스크를 넓힐 수는 없다.
            mask_np = (mask.cpu().numpy() > 0.5).astype(np.uint8)
            if mask_np.shape != cv_image.shape[:2]:
                mask_np = cv2.resize(mask_np,
                                     (cv_image.shape[1], cv_image.shape[0]),
                                     interpolation=cv2.INTER_NEAREST)
            if self.close_kernel is not None:
                mask_np = cv2.morphologyEx(mask_np, cv2.MORPH_CLOSE, self.close_kernel)

            # ---------- 윤곽선: 가장 큰 조각 ----------
            # RETR_EXTERNAL 이므로 꼭지가 제외되어 생긴 내부 구멍은 메워진다.
            contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            area_px = cv2.contourArea(contour)
            if area_px < self.min_area_px:
                continue

            # ---------- 중심 픽셀 ----------
            # 마스크가 한쪽 잘리면 무게중심은 남은 쪽으로 끌려가므로,
            # 윤곽선을 감싸는 최소 외접원의 중심을 기본으로 쓴다.
            (ux, uy), radius_px = cv2.minEnclosingCircle(contour)
            if self.center_mode == 'centroid':
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue
                ux, uy = M['m10'] / M['m00'], M['m01'] / M['m00']

            cx_px = int(round(ux))
            cy_px = int(round(uy))
            h_img, w_img = mask_np.shape
            cx_px = max(0, min(cx_px, w_img - 1))
            cy_px = max(0, min(cy_px, h_img - 1))

            # 마스크가 원에 얼마나 가까운가 (1.0 에 가까울수록 온전한 원)
            circ_area = np.pi * radius_px ** 2
            fill = area_px / circ_area if circ_area > 0 else 0.0

            # ---------- Z : 마스크 내부 깊이 분포의 가까운 쪽 ----------
            v_idx, u_idx = np.where(mask_np == 1)
            z_pixels = self.latest_depth_img[v_idx, u_idx].astype(float)
            z_valid = z_pixels[z_pixels > 0]
            if z_valid.size < 10:
                continue
            Z = float(np.percentile(z_valid, self.depth_pct))

            X = (cx_px - self.cx) * Z / self.fx
            Y = (cy_px - self.cy) * Z / self.fy

            # ---------- 보조 무게 추정 ----------
            # 박스 안에서는 바닥이 가려져 링 바닥 추정이 안 되므로 전체 높이를
            # 잴 수 없다. 대신 적도면(윤곽선)과 꼭대기(중심)의 깊이 차이가
            # 반높이 c 가 되므로, 타원체 부피 V = (4/3)·A·c 로 구한다.
            z_rim, rim_frac = self.contour_depth(contour)
            z_center = self.center_depth(mask_np, cx_px, cy_px, radius_px)
            (R_mm, area_mm2, c_mm, volume_cm3,
             weight_g, used) = self.estimate_weight(
                area_px, radius_px, Z, z_rim, rim_frac, z_center)

            # 지름 검사용 등가 지름: 면적을 원으로 환산한 지름
            # (타원체 모드에서는 외접원 반지름이 아니라 실제 면적을 기준으로 삼는다)
            diam_eq = (2.0 * np.sqrt(area_mm2 / np.pi)
                       if area_mm2 > 0 else 2.0 * R_mm)

            if self.debug:
                self.get_logger().info(
                    f'[진단] px:({cx_px},{cy_px}) Z_top:{Z:.1f} '
                    f'Z_rim:{z_rim:.1f}({rim_frac*100:.0f}%) '
                    f'Z_cen:{z_center:.1f} c:{c_mm:.1f} [{used}] '
                    f'X:{X:+.1f} Y:{Y:+.1f} mm 원형도:{fill:.2f} '
                    f'A:{area_mm2:.0f} mm² 등가지름:{diam_eq:.1f} mm '
                    f'V:{volume_cm3:.1f} cm³ W:{weight_g:.1f} g '
                    f'(유효픽셀 {z_valid.size})')

            # ---------- 개체 매칭 후 축적 ----------
            tid = self.match_track(cx_px, cy_px, self.pick_match_dist,
                                   ('xs', 'ys', 'zs', 'us', 'vs', 'fills',
                                    'rads', 'radii_mm', 'volumes', 'weights',
                                    'z_rims', 'rim_fracs', 'used_rim',
                                    'areas_mm2', 'c_mms', 'z_cens', 'diams'))
            d = self.tracks[tid]['data']
            d['xs'].append(X)
            d['ys'].append(Y)
            d['zs'].append(Z)
            d['us'].append(cx_px)
            d['vs'].append(cy_px)
            d['fills'].append(fill)
            d['rads'].append(radius_px)
            d['radii_mm'].append(R_mm)
            d['volumes'].append(volume_cm3)
            d['weights'].append(weight_g)
            d['z_rims'].append(z_rim)
            d['rim_fracs'].append(rim_frac)
            # 실제로 어느 방식을 썼는지 (1 = 타원체, 0 = 구체 대체)
            d['used_rim'].append(1.0 if used == 'ellipsoid' else 0.0)
            d['areas_mm2'].append(area_mm2)
            d['c_mms'].append(c_mm)
            d['z_cens'].append(z_center)
            d['diams'].append(diam_eq)

            if self.enable_viz:
                cv2.circle(annotated, (cx_px, cy_px), int(radius_px), (0, 200, 0), 1)
                cv2.circle(annotated, (cx_px, cy_px), 6, (0, 0, 255), -1)
                cv2.circle(annotated, (cx_px, cy_px), 9, (255, 255, 255), 2)
                if self.sphere_weight:
                    cv2.putText(
                        annotated, f'{weight_g:.0f}g',
                        (cx_px - 18, cy_px - int(radius_px) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

        if self.enable_viz and self.mode == MODE_PICK_CENTER:
            self.draw_center_grid(annotated)

        return annotated

    # ==================================================================
    # 마스크 중심부 깊이 (= 과육 꼭대기)
    # ==================================================================
    def center_depth(self, mask_np, cx_px, cy_px, radius_px):
        """중심 픽셀 주변 작은 원판의 깊이 중위값을 돌려준다.

        한 점만 읽으면 꼭지 구멍이나 반사로 튀는 값을 그대로 맞는다.
        그래서 외접원 반지름의 center_patch_frac 비율만큼을 반지름으로 하는
        원판 안에서 중위값을 뽑는다. 마스크 밖과 무효 깊이는 제외한다.

        반환: Z_center_mm  — 유효 점이 없으면 0.0
        """
        h_img, w_img = self.latest_depth_img.shape[:2]
        r = max(2, int(round(radius_px * self.center_patch_frac)))
        u0, u1 = max(0, cx_px - r), min(w_img, cx_px + r + 1)
        v0, v1 = max(0, cy_px - r), min(h_img, cy_px + r + 1)
        if u1 <= u0 or v1 <= v0:
            return 0.0

        patch_z = self.latest_depth_img[v0:v1, u0:u1].astype(float)
        patch_m = mask_np[v0:v1, u0:u1]
        # 원판 모양으로 자른다 (사각형이면 모서리에서 옆 토마토를 읽을 수 있다)
        vv, uu = np.ogrid[v0:v1, u0:u1]
        disc = (uu - cx_px) ** 2 + (vv - cy_px) ** 2 <= r * r

        sel = patch_z[(patch_m == 1) & disc & (patch_z > 0)]
        if sel.size == 0:
            return 0.0
        return float(np.median(sel))

    # ==================================================================
    # 윤곽선 깊이 (= 적도 깊이) 산출
    # ==================================================================
    def contour_depth(self, contour):
        """윤곽선 픽셀의 깊이 중위값과 유효 비율을 돌려준다.

        구의 실루엣 테두리는 적도이므로, 이 깊이가 곧 중심 깊이 D 다.
        (엄밀히는 접점이 이루는 원의 평면이 중심면보다 D*sin^2(theta) 만큼
         앞에 있지만, 각반경이 작아 1~2% 수준이라 무시한다)

        실루엣 경계는 물체와 배경이 섞이는 지점이라 깊이가 0(무효)으로
        나오거나 배경을 읽는 일이 잦다. 그래서 유효 비율도 함께 돌려주어
        호출 측이 신뢰할지 판단하게 한다.

        반환: (Z_rim_mm, 유효비율)  — 유효 점이 없으면 (0.0, 0.0)
        """
        h_img, w_img = self.latest_depth_img.shape[:2]
        pts = contour.reshape(-1, 2)
        u = np.clip(pts[:, 0], 0, w_img - 1)
        v = np.clip(pts[:, 1], 0, h_img - 1)
        z = self.latest_depth_img[v, u].astype(float)
        z_valid = z[z > 0]
        if z_valid.size == 0:
            return 0.0, 0.0
        frac = z_valid.size / float(z.size)
        # 배경(바닥, 뒤쪽 토마토)을 읽은 점이 섞이므로 중위값으로 뽑는다
        return float(np.median(z_valid)), float(frac)

    # ==================================================================
    # 구체 근사 무게 추정
    # ==================================================================
    def sphere_weight_from_circle(self, radius_px, z_top_mm, z_rim_mm, rim_frac):
        """최소외접원 반지름(px)과 깊이로 구체 반지름/부피/무게를 구한다.

        구의 실루엣은 구 표면의 접선에서 생기므로 단순 비례식(R = rho*Z/f)이
        아니라 각반경을 거쳐야 한다. 각반경 theta 에 대해
            tan(theta) = rho / f,  sin(theta) = R / D
        이고, 어떤 깊이를 D 로 볼지가 radius_depth_mode 다.

            'rim' : D = Z_rim (윤곽선 깊이 = 적도 깊이)  ->  R = D * s
            'top' : D = Z_top + R (순환식을 정리)        ->  R = Z_top * s / (1 - s)

        'rim' 을 쓰되 윤곽선 깊이의 유효 비율이 min_rim_valid 미만이면
        'top' 으로 되돌아간다.

        반환: (R_mm, V_cm3, W_g, used)   used = 'rim' | 'top' | ''

        sphere_weight 가 False 여도 계산은 수행한다. 지름 범위 검사(박스 벽 등
        오검출 제거)가 R 값을 쓰기 때문이다. 발행과 화면 표시만 끈다.
        """
        if radius_px <= 0:
            return 0.0, 0.0, 0.0, ''

        f = 0.5 * (self.fx + self.fy)
        t = radius_px / f
        s = t / np.sqrt(1.0 + t * t)
        if s >= 1.0:                      # 이론상 불가하지만 방어
            return 0.0, 0.0, 0.0, ''

        use_rim = (self.radius_depth_mode == 'rim'
                   and z_rim_mm > 0
                   and rim_frac >= self.min_rim_valid)

        if use_rim:
            R_mm = z_rim_mm * s
            used = 'rim'
        else:
            if z_top_mm <= 0:
                return 0.0, 0.0, 0.0, ''
            R_mm = z_top_mm * s / (1.0 - s)
            used = 'top'

        volume_cm3 = (4.0 / 3.0) * np.pi * R_mm ** 3 / 1000.0
        weight_g = volume_cm3 * self.weight_gain
        return float(R_mm), float(volume_cm3), float(weight_g), used

    # ==================================================================
    # 무게 추정 (타원체 우선, 실패 시 구체로 대체)
    # ==================================================================
    def estimate_weight(self, area_px, radius_px, z_top_mm,
                        z_rim_mm, rim_frac, z_center_mm):
        """면적과 반높이로 타원체 부피를 구해 무게를 추정한다. (v3 방식)

        마스크의 정사영 면적 A 는 적도 단면이고, 윤곽선 깊이(적도)와
        중심 깊이(꼭대기)의 차이가 반높이 c 다. 타원체 부피는

            A   = pi * a * b                  (적도 단면)
            V   = (4/3) * pi * a * b * c = (4/3) * A * c

        이다. 전체 높이 h = 2c 를 쓰면 V = (2/3) * A * h 와 같은 식이지만,
        여기서 재는 것은 c 이므로 (4/3) 계수를 쓴다. 바닥이 가려져 아래쪽
        절반을 볼 수 없으므로 상하 대칭을 가정한다.

        면적은 윤곽선 내부 전체를 센다. RETR_EXTERNAL 로 뽑은 윤곽선이므로
        꼭지가 마스크에서 빠져 생긴 내부 구멍은 메워진 상태다.
        픽셀 면적을 적도 깊이에서의 물리 면적으로 환산한다.

            A_mm2 = area_px * Z_rim^2 / (fx * fy)

        윤곽선 깊이가 쓸 수 없거나(유효 비율 미달) c <= 0 이면 구체 근사로
        되돌아간다. 그 경우 면적 기반 값은 0 으로 두고 구체 부피를 쓴다.

        반환: (R_mm, A_mm2, c_mm, V_cm3, W_g, used)
               used = 'ellipsoid' | 'rim' | 'top' | ''
        """
        # 구체 근사는 대체 경로용으로 항상 계산해둔다
        R_mm, sph_vol, sph_w, sph_used = self.sphere_weight_from_circle(
            radius_px, z_top_mm, z_rim_mm, rim_frac)

        if self.weight_mode != 'ellipsoid':
            return R_mm, 0.0, 0.0, sph_vol, sph_w, sph_used

        rim_ok = z_rim_mm > 0 and rim_frac >= self.min_rim_valid
        c_raw = (z_rim_mm - z_center_mm) if (rim_ok and z_center_mm > 0) else 0.0

        if not rim_ok or c_raw <= 0.0 or area_px <= 0:
            # 적도보다 중심이 더 멀게 나오면(c <= 0) 깊이가 뒤집힌 것이므로
            # 타원체 계산을 신뢰할 수 없다. 구체 근사로 넘긴다.
            return R_mm, 0.0, max(c_raw, 0.0), sph_vol, sph_w, sph_used

        # ---------- 접선면 보정 ----------
        # 실루엣 테두리는 중심면이 아니라 접점이 이루는 원의 평면에서 생기고,
        # 그 평면은 중심보다 D*sin^2(theta) 만큼 앞에 있다.
        #     Z_rim = D * cos^2(theta)  ->  D = Z_rim / (1 - s^2)
        #     offset = D - Z_rim = Z_rim * s^2 / (1 - s^2)
        # 보정하지 않으면 반높이 c 가 offset 만큼 작게 나온다. 각도가 작아
        # offset 자체는 2~3 mm 지만, c 가 20 mm 안팎이라 비율로는 15% 를 넘는다.
        # (합성 타원체 검산: 보정 전 부피 -19%, 보정 후 -5%)
        z_ref = z_rim_mm
        c_mm = c_raw
        if self.tangent_correct and radius_px > 0:
            f = 0.5 * (self.fx + self.fy)
            t = radius_px / f
            s = t / np.sqrt(1.0 + t * t)
            s2 = s * s
            if s2 < 0.95:                 # 이론상 불가하지만 방어
                offset = z_rim_mm * s2 / (1.0 - s2)
                z_ref = z_rim_mm + offset   # 중심 깊이 D
                c_mm = c_raw + offset

        # 면적은 중심면(적도) 깊이에서 환산한다
        area_mm2 = area_px * (z_ref * z_ref) / (self.fx * self.fy)
        volume_cm3 = (4.0 / 3.0) * area_mm2 * c_mm / 1000.0
        weight_g = volume_cm3 * self.ellipsoid_gain
        return (R_mm, float(area_mm2), float(c_mm),
                float(volume_cm3), float(weight_g), 'ellipsoid')

    # ==================================================================
    # 무게 모드 : 1프레임 측정 및 버퍼 축적
    # ==================================================================
    def measure_weight(self, cv_image):
        result = self.weight_model(cv_image, verbose=False, conf=self.weight_conf,
                                   retina_masks=True)[0]
        annotated_frame = self.render_base(cv_image, result)
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
            contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
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
            if M['m00'] == 0:
                continue
            cx_px = int(M['m10'] / M['m00'])
            cy_px = int(M['m01'] / M['m00'])

            Z_center = float(self.latest_depth_img[cy_px, cx_px])
            if Z_center <= 0:
                Z_center = np.median(Z_vals)

            X_center = (cx_px - self.cx) * Z_center / self.fx
            Y_center = (cy_px - self.cy) * Z_center / self.fy

            # --- 도넛(링) 영역으로 바닥면 추정 ---
            dilated_mask = cv2.dilate(mask_np, self.ring_kernel, iterations=1)
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
                    f'[진단] px:({cx_px},{cy_px}) Z_floor:{Z_floor:.1f} '
                    f'Z_center:{Z_center:.1f} H:{real_height_mm:.1f}')

            # --- 부피 / 무게 (회전 타원체: V = 2/3 * A * h) ---
            real_volume_cm3 = (2.0 / 3.0) * real_area_mm2 * real_height_mm / 1000.0
            real_weight_g = real_volume_cm3 * self.TOMATO_DENSITY

            cv2.circle(annotated_frame, (cx_px, cy_px), 5, (0, 0, 255), -1)
            cv2.putText(
                annotated_frame,
                f'A:{real_area_mm2:.1f} H:{real_height_mm:.1f} V:{real_volume_cm3:.1f} cm3',
                (cx_px - 100, cy_px - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 2)

            # --- 중심 좌표 기반 개체 매칭 후 버퍼 축적 ---
            tid = self.match_track(
                cx_px, cy_px, self.weight_match_dist,
                ('areas', 'heights', 'xs', 'ys', 'zs', 'z_floors', 'z_centers',
                 'volumes', 'majors', 'minors', 'weights'))
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
    def match_track(self, cx_px, cy_px, match_dist, keys):
        best_tid, best_d = None, match_dist
        for tid, tr in self.tracks.items():
            dist = np.hypot(tr['cx'] - cx_px, tr['cy'] - cy_px)
            if dist < best_d:
                best_tid, best_d = tid, dist

        if best_tid is None:
            best_tid = self.next_tid
            self.next_tid += 1
            self.tracks[best_tid] = {
                'cx': cx_px, 'cy': cy_px,
                'data': {k: [] for k in keys},
            }
        else:
            self.tracks[best_tid]['cx'] = cx_px
            self.tracks[best_tid]['cy'] = cy_px
        return best_tid

    # ==================================================================
    # 실제 사용한 깊이 표기
    # ==================================================================
    @staticmethod
    def used_label(frac):
        """프레임별 타원체 사용 비율(0~1)을 사람이 읽을 라벨로 바꾼다."""
        if frac >= 0.999:
            return '타원체'
        if frac <= 0.001:
            return '구체 대체'
        return f'타원체 {frac*100:.0f}% / 구체 {(1-frac)*100:.0f}%'

    # ==================================================================
    # 처리 프레임 수 / 처리율 로그
    # ==================================================================
    def log_frame_rate(self):
        elapsed = max(time.time() - self.capture_start, 1e-6)
        fps = self.frames_done / elapsed
        warn = ''
        if self.frames_done < self.min_frames:
            warn = (f'  ⚠ min_frames({self.min_frames}) 미달 '
                    f'-> 모든 트랙이 샘플 부족으로 제외됨')
        self.get_logger().info(
            f'  처리 프레임: {self.frames_done}개 / {elapsed:.2f}초 '
            f'({fps:.1f} fps){warn}')

    # ==================================================================
    # 가운데 영역 (3x3 분할의 중앙 칸) 판정과 표시
    # ==================================================================
    def in_center_cell(self, u, v):
        """중심 픽셀이 화면 중앙 칸 안에 있는지 판정한다.

        center_region_frac 이 가로/세로에서 중앙 칸이 차지하는 비율이다.
        1/3 이면 화면을 3x3 으로 나눴을 때의 가운데 한 칸이 된다.
        영상 크기를 아직 모르면(첫 프레임 전) 참으로 둬서 걸러내지 않는다.
        """
        if self.img_w <= 0 or self.img_h <= 0:
            return True
        f = self.center_region_frac
        u0 = self.img_w * (1.0 - f) * 0.5
        v0 = self.img_h * (1.0 - f) * 0.5
        return (u0 <= u <= self.img_w - u0) and (v0 <= v <= self.img_h - v0)

    def draw_center_grid(self, img):
        """3x3 격자선과 중앙 칸을 그린다. (중앙 모드 확인용)"""
        h, w = img.shape[:2]
        f = self.center_region_frac
        u0 = int(round(w * (1.0 - f) * 0.5))
        v0 = int(round(h * (1.0 - f) * 0.5))
        u1, v1 = w - u0, h - v0
        for x in (u0, u1):
            cv2.line(img, (x, 0), (x, h), (120, 120, 120), 1)
        for y in (v0, v1):
            cv2.line(img, (0, y), (w, y), (120, 120, 120), 1)
        cv2.rectangle(img, (u0, v0), (u1, v1), (0, 255, 255), 2)

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
    # 위치 모드 종료 -> 발행
    # ==================================================================
    def finalize_pick(self, mode):
        center_only = (mode == MODE_PICK_CENTER)
        # 중앙 모드도 1개만 발행하므로 최근접 경로를 그대로 쓴다
        nearest_only = (mode == MODE_PICK_NEAREST) or center_only

        summaries = []
        dropped_frames = 0
        dropped_fill = 0
        dropped_diam = 0
        dropped_region = 0
        for tid, tr in self.tracks.items():
            data = tr['data']
            n = len(data['zs'])
            if n < self.min_frames:
                dropped_frames += 1
                continue
            s = {k: self.filtered_mean(v) for k, v in data.items()}
            # 0/1 플래그는 절삭평균을 쓰면 비율이 왜곡되므로 단순 평균으로 덮어쓴다
            s['used_rim'] = float(np.mean(data['used_rim'])) if data['used_rim'] else 0.0

            # ---------- 가운데 영역 검사 (중앙 모드) ----------
            # 화면을 3x3 으로 나눴을 때 중앙 칸 안에 중심이 있는 개체만 남긴다.
            if center_only and not self.in_center_cell(s['us'], s['vs']):
                dropped_region += 1
                continue

            # ---------- 지름 범위 검사 (박스 벽 등 오검출 제거) ----------
            # 박스 벽은 마스크가 길게 잡혀 면적이 비정상적으로 커지고,
            # 벽 모서리 조각만 잡히면 반대로 작아진다. 토마토가 가질 수 없는
            # 지름이면 좌표도 무게도 발행하지 않는다. (파지 사고 방지)
            # 타원체 모드에서는 면적을 원으로 환산한 등가 지름을 쓴다.
            # 외접원 지름은 마스크가 길쭉할 때 면적과 크게 어긋나기 때문이다.
            if s['diams'] > 0:
                diam = s['diams']
                diam_src = '등가'
            else:
                diam = 2.0 * s['radii_mm']
                diam_src = '외접원'
            too_small = self.min_diam > 0.0 and diam < self.min_diam
            too_big = self.max_diam > 0.0 and diam > self.max_diam
            if too_small or too_big:
                dropped_diam += 1
                reason = '작음' if too_small else '큼'
                self.get_logger().warn(
                    f"  [제외] px:({s['us']:.0f},{s['vs']:.0f}) "
                    f"{diam_src}지름 {diam:.1f} mm ({reason}) | "
                    f"Z_rim: {s['z_rims']:.1f} / Z_cen: {s['z_cens']:.1f} mm, "
                    f"c: {s['c_mms']:.1f} mm, A: {s['areas_mm2']:.0f} mm², "
                    f"원형도 {s['fills']:.2f}, 프레임 {n}개 "
                    f"-> 허용 범위 {self.min_diam:.0f}~{self.max_diam:.0f} mm")
                continue

            # 최근접 모드에서만 원형도 필터를 적용한다
            # (전체 목록 모드는 수신 측이 직접 판단할 수 있도록 모두 넘긴다)
            if nearest_only and self.min_fill > 0.0 and s['fills'] < self.min_fill:
                dropped_fill += 1
                continue
            s['n'] = n
            summaries.append(s)

        # 가까운(= 위에 쌓인) 순으로 정렬해 로봇이 위에서부터 집을 수 있게 한다
        if nearest_only or self.sort_by_depth:
            summaries.sort(key=lambda s: s['zs'])

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id

        weight_msg = Float32MultiArray()

        publish_list = summaries[:1] if nearest_only else summaries
        for s in publish_list:
            p = Pose()
            p.position.x = s['xs'] / 1000.0      # mm -> m
            p.position.y = s['ys'] / 1000.0
            p.position.z = s['zs'] / 1000.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)
            # 좌표와 같은 순서 / 같은 개수로 발행해 인덱스로 짝지을 수 있게 한다
            weight_msg.data.append(float(s['weights']))

        # 검출 0개여도 빈 목록을 발행해 로봇이 대기 상태에 빠지지 않도록 함
        if center_only:
            pub, w_pub = self.middle_pub, self.middle_weight_pub
        elif nearest_only:
            pub, w_pub = self.pick_nearest_pub, self.nearest_weight_pub
        else:
            pub, w_pub = self.pick_all_pub, self.pick_weight_pub
        pub.publish(pose_array)
        if self.sphere_weight:
            w_pub.publish(weight_msg)

        excl = f'샘플 부족 {dropped_frames}개, 지름 범위 밖 {dropped_diam}개'
        if center_only:
            excl += f', 중앙 칸 밖 {dropped_region}개'
        if nearest_only and self.min_fill > 0.0:
            excl += f', 원형도 미달 {dropped_fill}개'

        # 처리 프레임 수를 항상 남긴다. 발행이 0개일 때 원인 판별의 출발점이 된다
        # (프레임이 min_frames 보다 적으면 모든 트랙이 샘플 부족으로 빠진다)
        self.log_frame_rate()

        if not summaries:
            self.get_logger().info(
                f'📸 촬영 완료 -> 대상 없음 (빈 목록 발행, 제외: {excl})')
            self.tracks.clear()
            self.get_logger().info('=== 트리거 대기 중 ===')
            return

        if nearest_only:
            target = summaries[0]
            flag = '  ⚠ 마스크 불완전' if target['fills'] < 0.6 else ''
            self.get_logger().info(
                f'📸 촬영 완료 -> '
                f'{"중앙 칸 " if center_only else ""}후보 {len(summaries)}개 중 '
                f'최근접 1개 발행 (frame: {self.frame_id}, 제외: {excl})')
            self.get_logger().info(
                f"  [대상] px:({target['us']:.0f},{target['vs']:.0f}) | "
                f"X: {target['xs']:+7.1f} / Y: {target['ys']:+7.1f} / "
                f"Z: {target['zs']:7.1f} mm "
                f"-> 발행값 ({target['xs']/1000.0:+.4f}, {target['ys']/1000.0:+.4f}, "
                f"{target['zs']/1000.0:.4f}) m  "
                f"(프레임 {target['n']}개, 원형도 {target['fills']:.2f}){flag}")
            if self.sphere_weight:
                gain = (self.ellipsoid_gain if target['used_rim'] > 0.5
                        else self.weight_gain)
                self.get_logger().info(
                    f"         무게  A: {target['areas_mm2']:.0f} mm² | "
                    f"c: {target['c_mms']:.1f} mm | "
                    f"등가지름: {target['diams']:.1f} mm | "
                    f"V: {target['volumes']:.1f} cm³ | "
                    f"W: {target['weights']:.1f} g  (gain {gain:.3f})\n"
                    f"         깊이  Z_rim: {target['z_rims']:.1f} / "
                    f"Z_cen: {target['z_cens']:.1f} / Z_top: {target['zs']:.1f} mm "
                    f"(윤곽선 유효 {target['rim_fracs']*100:.0f}%) | "
                    f"사용: {self.used_label(target['used_rim'])} "
                    f"[설정 {self.weight_mode}]")

            # 참고용: 발행하지 않은 나머지 후보 (Z 가 비슷하면 선택이 흔들릴 수 있음)
            for i, s in enumerate(summaries[1:4], start=1):
                gap = s['zs'] - target['zs']
                self.get_logger().info(
                    f"  (참고 {i}) px:({s['us']:.0f},{s['vs']:.0f}) | "
                    f"X: {s['xs']:+7.1f} / Y: {s['ys']:+7.1f} / "
                    f"Z: {s['zs']:7.1f} mm  (대상보다 {gap:+.1f} mm 뒤, "
                    f"프레임 {s['n']}개, 원형도 {s['fills']:.2f})")
        else:
            order = '가까운 순' if self.sort_by_depth else '검출 순'
            self.get_logger().info(
                f'📸 촬영 완료 -> 토마토 {len(summaries)}개 발행 '
                f'({order}, frame: {self.frame_id}, 제외: {excl})')
            for i, s in enumerate(summaries):
                flag = '  ⚠ 마스크 불완전' if s['fills'] < 0.6 else ''
                extra = ''
                if self.sphere_weight:
                    extra = (f"\n       무게  A: {s['areas_mm2']:.0f} mm² | "
                             f"c: {s['c_mms']:.1f} mm | "
                             f"등가지름: {s['diams']:.1f} mm | "
                             f"V: {s['volumes']:.1f} cm³ | W: {s['weights']:.1f} g | "
                             f"Z_rim: {s['z_rims']:.1f} / Z_cen: {s['z_cens']:.1f} mm "
                             f"(유효 {s['rim_fracs']*100:.0f}%, "
                             f"{self.used_label(s['used_rim'])})")
                self.get_logger().info(
                    f"  [{i}] px:({s['us']:.0f},{s['vs']:.0f}) | "
                    f"X: {s['xs']:+7.1f} / Y: {s['ys']:+7.1f} / Z: {s['zs']:7.1f} mm "
                    f"-> 발행값 ({s['xs']/1000.0:+.4f}, {s['ys']/1000.0:+.4f}, "
                    f"{s['zs']/1000.0:.4f}) m  "
                    f"(프레임 {s['n']}개, 원형도 {s['fills']:.2f}){flag}{extra}")

        self.tracks.clear()
        self.get_logger().info('=== 트리거 대기 중 ===')

    # ==================================================================
    # 무게 모드 종료 -> 발행
    # ==================================================================
    def finalize_weight(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id

        weight_msg = Float32MultiArray()
        height_msg = Float32MultiArray()

        self.log_frame_rate()

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
        self.center_pub.publish(pose_array)
        self.weight_pub.publish(weight_msg)
        self.height_pub.publish(height_msg)

        if not summaries:
            self.get_logger().info('📸 촬영 완료 -> 검출된 토마토 없음 (빈 목록 발행)')
        else:
            self.get_logger().info(
                f'📸 촬영 완료 -> 토마토 {len(summaries)}개 발행 (frame: {self.frame_id})')
            for i, s in enumerate(summaries):
                self.get_logger().info(
                    f"  [{i}] 면적: {s['areas']:.1f} mm² | 높이: {s['heights']:.1f} mm | "
                    f"부피: {s['volumes']:.1f} cm³ | 무게: {s['weights']:.1f} g  "
                    f"(프레임 {s['n']}개)\n"
                    f"      중심 3D  X: {s['xs']:.1f} / Y: {s['ys']:.1f} / "
                    f"Z: {s['zs']:.1f} mm -> 발행값 "
                    f"({s['xs']/1000.0:.4f}, {s['ys']/1000.0:.4f}, {s['zs']/1000.0:.4f}) m\n"
                    f"      장축: {s['majors']:.1f} mm | 단축: {s['minors']:.1f} mm | "
                    f"Z_floor: {s['z_floors']:.1f} mm | Z_center: {s['z_centers']:.1f} mm")

        self.tracks.clear()
        self.get_logger().info('=== 트리거 대기 중 ===')


def main():
    rclpy.init()
    node = TomatoVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
