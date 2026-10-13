import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 리얼센스 카메라 패키지의 Launch 파일 경로 찾기
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')
    
    # 2. 카메라 실행 정의 (align_depth 옵션 포함)
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={'align_depth.enable': 'true'}.items(),
    )

    # 3. 가상환경 파이썬 및 YOLO 노드 파일 경로 설정
    home_dir = os.path.expanduser('~')
    python_executable = os.path.join(home_dir, 'doosan_manipulator', 'venv_yolo', 'bin', 'python3')
    node_path = os.path.join(
        home_dir, 
        'doosan_manipulator/doosan_vision_grasp/doosan_vision_grasp/yolo_pose_node_d405.py'
    )

    # 4. YOLO 노드 실행 정의
    yolo_node = Node(
        package='doosan_vision_grasp',
        executable=python_executable,
        arguments=[node_path],
        name='yolo_pose_node_d405',
        output='screen',
        emulate_tty=True
    )

    # 5. 카메라와 YOLO를 함께 실행
    return LaunchDescription([
        realsense_camera,
        yolo_node
    ])