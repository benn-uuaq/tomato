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
    
    # 2. 카메라 구동 정의 (3D 점구름 비활성화로 부하 최소화)
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false', # 2D 면적 계산이므로 3D는 Off!
            #'depth_module.depth_profile': '848x480x15',
            'enable_color': 'true'
        }.items(),
    )

    # 3. 가상환경 파이썬 및 새 파일(tomato_area.py) 경로 지정
    home_dir = os.path.expanduser('~')
    python_executable = os.path.join(home_dir, 'doosan_manipulator', 'venv_yolo', 'bin', 'python3')
    node_path = os.path.join(
        home_dir, 
        'doosan_manipulator',
        'doosan_vision_grasp',
        'doosan_vision_grasp',
        'tomato_area.py'
    )

    # 4. 토마토 면적 계산 노드 실행 정의
    yolo_node = Node(
        package='doosan_vision_grasp',
        executable=python_executable,
        arguments=[node_path],
        name='tomato_area_calculator',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        realsense_camera,
        yolo_node
    ])