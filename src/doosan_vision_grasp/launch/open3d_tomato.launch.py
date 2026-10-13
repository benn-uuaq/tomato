import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 리얼센스 카메라 패키지 런치 연결 및 정렬 처리
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')
    
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'align_depth.enable': 'true',       # 컬러와 뎁스를 일치시키는 필수 옵션
            'pointcloud.enable': 'true',       # 부하가 큰 ROS 3D 연산은 off
            'depth_module.depth_profile': '848x480x15', # 최적화용 대역폭 제한 (15 FPS)
            'enable_color': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true'

            #'depth_module.enable_auto_white_balance': 'false'
            #'white_balance': '4600 
        }.items(),
    )

    # 2. 가상환경 세션 및 실행할 노드 절대 경로 지정
    home_dir = os.path.expanduser('~')
    python_executable = os.path.join(home_dir, 'doosan_manipulator', 'venv_yolo', 'bin', 'python3')
    node_path = os.path.join(
        home_dir, 
        'doosan_manipulator', 
        'doosan_vision_grasp', 
        'doosan_vision_grasp', 
        'open3d_tomato_viewer.py'
    )

    # 3. 가상환경 내부 파이썬 환경의 노드 실행 정의
    yolo_open3d_node = ExecuteProcess(
        cmd=[python_executable, node_path],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        realsense_camera,
        yolo_open3d_node
    ])