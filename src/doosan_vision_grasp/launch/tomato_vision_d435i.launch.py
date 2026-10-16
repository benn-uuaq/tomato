import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1. 리얼센스 패키지 launch 경로 찾기
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    # 2. D435i 카메라 구동 정의 (RGB-Depth 정렬 활성화) — 카메라는 한 번만 띄운다
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'align_depth.enable': 'true',   # RGB-Depth 픽셀 정렬
            'pointcloud.enable': 'false',   # 부하 최소화
            'enable_color': 'true',
            # 해상도 고정 — 메모리가 부족하거나 처리 속도가 느릴 때 주석을 해제한다
            # (D405 기본 848x480 / D435i 기본 1280x720 이라 기종에 따라 달라진다)
            # 'rgb_camera.color_profile': '848x480x30',
            # 'depth_module.depth_profile': '848x480x30',
        }.items(),
    )

    # 3. 가상환경 파이썬 · 노드 파일 · 모델 경로
    #    (tomato_ws 수정) 원래는 ~/doosan_manipulator 로 고정이었다. 이 워크스페이스
    #    (tomato_ws) 에서 띄울 때와 한양대 워크스페이스에서 띄울 때 모두 맞도록,
    #    환경변수 -> 이 패키지 share -> ~/doosan_manipulator 순으로 찾는다.
    home_dir = os.path.expanduser('~')
    hanyang = os.path.join(home_dir, 'doosan_manipulator')
    ws = os.environ.get('TOMATO_WS', '')

    def first_existing(*paths):
        for path in paths:
            if path and os.path.exists(path):
                return path
        return paths[-1]

    python_executable = first_existing(
        os.environ.get('TOMATO_VENV_PY', ''),
        os.path.join(hanyang, 'venv_yolo', 'bin', 'python3'),
        '/usr/bin/python3')
    node_path = first_existing(
        os.path.join(get_package_share_directory('doosan_vision_grasp'), 'scripts', 'tomato_vision_d435i.py'),
        os.path.join(hanyang, 'doosan_vision_grasp', 'doosan_vision_grasp', 'tomato_vision_d435i.py'))
    pick_model_path = first_existing(
        os.environ.get('TOMATO_PICK_MODEL', ''),
        os.path.join(ws, 'models', 'best.pt') if ws else '',
        os.path.join(hanyang, 'best.pt'))
    weight_model_path = first_existing(
        os.environ.get('TOMATO_MODEL', ''),
        os.path.join(ws, 'models', 'tomato_v1.pt') if ws else '',
        os.path.join(hanyang, 'tomato_v1.pt'))
    # 카메라 창: 화면이 있을 때만 띄운다. SSH 로 띄우면 화면이 없어 cv2 가 죽는다.
    # VISION_VIZ=1 / 0 으로 강제할 수 있다.
    viz_env = os.environ.get('VISION_VIZ', '')
    enable_viz = (viz_env == '1') if viz_env in ('0', '1') else bool(
        os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))

    # 4. 통합 비전 노드 실행 정의 (위치 인식 + 무게 추정, 모델 2개 상주)
    vision_node = Node(
        package='doosan_vision_grasp',
        executable=python_executable,
        arguments=[node_path],
        name='tomato_vision_d435i',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # ---------- 모델 경로 ----------
            # 위치 인식: 뭉친 토마토 분리에 강한 모델 (꼭지 제외)
            'pick_model_path': pick_model_path,
            # 무게 추정: 꼭지를 포함해 학습한 기존 모델
            'weight_model_path': weight_model_path,

            # ---------- 촬영 시간 (초) ----------
            'pick_capture_duration': 1.5,
            'weight_capture_duration': 1.5,

            # ---------- 위치 인식 모드 ----------
            'pick_conf': 0.25,            # YOLO 신뢰도 임계값
            'pick_match_dist_px': 40.0,   # 동일 토마토 판정 거리 (픽셀)
            'min_frames': 1,              # 최소 검출 프레임 수 (일시적 오검출 제거)
            'min_area_px': 500.0,         # 최소 마스크 면적 (픽셀)
            'close_px': 0,                # 모폴로지 닫기 커널 (0 = 미사용, 예: 9)
            'center_mode': 'circle',      # 'circle' = 최소외접원 중심, 'centroid' = 무게중심
            'depth_percentile': 10.0,     # Z 산출 백분위
            'sort_by_depth': True,        # 전체 목록을 가까운 순으로 정렬
            'min_fill': 0.0,
            # 중앙 모드에서 가운데 칸이 차지하는 비율
            # 0.3333 = 화면을 3x3 으로 나눈 중앙 한 칸
            'center_region_frac': 0.3333,              # 최근접 모드 최소 원형도 (0 = 미사용, 예: 0.6)

            # ---------- 위치 모드의 보조 무게 추정 (구체 근사) ----------
            'sphere_weight': True,        # 외접원 -> 구체 부피 -> 무게 추정 사용
            # Gain = 밀도(1.086) x 형상계수(0.614) — 저울 실측 후 이 값만 조정
            'weight_gain': 0.667,
            # 지름 범위 검사 — 박스 벽 등 오검출을 목록에서 제외 (0 = 미사용)
            # 범위를 벗어나면 좌표도 무게도 발행하지 않고 경고 로그만 남긴다
            'min_diameter_mm': 35.0,
            'max_diameter_mm': 120.0,

            # ---------- 무게 추정 모드 ----------
            'weight_conf': 0.25,          # YOLO 신뢰도 임계값
            'weight_match_dist_px': 60.0,  # 동일 토마토 판정 거리 (픽셀)
            'ring_px': 15,                # 바닥 추정 링 커널 크기

            # ---------- 표시 / 진단 ----------
            'enable_viz': enable_viz,
            # YOLO 마스크 색칠 오버레이 (GPU 메모리를 크게 먹음 — 젯슨은 False 권장)
            'viz_masks': False,
            'debug': False,               # 위치 모드 개체별 로그
            'debug_depth': False,         # 무게 모드 Z_floor / Z_center 로그
        }]
    )

    return LaunchDescription([
        realsense_camera,
        vision_node
    ])
