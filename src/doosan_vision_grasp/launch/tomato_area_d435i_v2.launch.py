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

    # 2. D435i 카메라 구동 정의 (RGB-Depth 정렬 활성화)
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'align_depth.enable': 'true',  # RGB-Depth 픽셀 정렬
            'pointcloud.enable': 'false',  # 부하 최소화
            'enable_color': 'true'
        }.items(),
    )

    # 3. 가상환경 파이썬 및 D435i 전용 노드 파일 경로 지정
    home_dir = os.path.expanduser('~')
    python_executable = os.path.join(home_dir, 'doosan_manipulator', 'venv_yolo', 'bin', 'python3')
    node_path = os.path.join(
        home_dir,
        'doosan_manipulator',
        'doosan_vision_grasp',
        'doosan_vision_grasp',
        'tomato_area_d435i_v2.py'
    )

    # 4. 토마토 연산 노드 실행 정의
    yolo_node = Node(
        package='doosan_vision_grasp',
        executable=python_executable,
        arguments=[node_path],
        name='tomato_area_calculator_d435i_v2',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'capture_duration': 1.0,   # 1회 촬영 측정 시간 (초)
            'match_dist_px': 60.0,     # 동일 토마토 판정 거리 (픽셀)
            'debug_depth': False,      # Z_floor / Z_center 진단 로그
        }]
    )

    return LaunchDescription([
        realsense_camera,
        yolo_node
    ])
