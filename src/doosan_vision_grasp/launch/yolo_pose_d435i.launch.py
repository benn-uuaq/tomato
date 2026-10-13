import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------------- Launch 인자 ----------------
    scale_x_arg = DeclareLaunchArgument('scale_x', default_value='1.0')
    scale_y_arg = DeclareLaunchArgument('scale_y', default_value='1.0')
    depth_profile_arg = DeclareLaunchArgument('depth_profile', default_value='848x480x30')
    color_profile_arg = DeclareLaunchArgument('color_profile', default_value='848x480x30')
    min_depth_arg = DeclareLaunchArgument('min_valid_depth', default_value='0.10')

    # ---------------- 리얼센스 카메라 ----------------
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'align_depth.enable': 'true',

            # [D435i] 컬러/뎁스 동기화 설정
            'enable_sync': 'true',

            # D435i 프로파일 매핑
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),

            # 적외선 프로젝터 ON (패턴 투사로 텍스처 부족한 토마토/딸기 인식률 향상)
            'depth_module.emitter_enabled': '1',

            # 오토 화이트 밸런스 비활성화 및 색온도 수동 고정 (D435i는 rgb_camera 하위)
            'rgb_camera.enable_auto_white_balance': 'false',
            'rgb_camera.white_balance': '4600',

            # IMU 미사용 (대역폭 절약)
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'unite_imu_method': '0',

            'pointcloud.enable': 'false',
            'initial_reset': 'true',
        }.items(),
    )

    # ---------------- YOLO 노드 ----------------
    home_dir = os.path.expanduser('~')
    python_executable = os.path.join(home_dir, 'doosan_manipulator', 'venv_yolo', 'bin', 'python3')
    node_path = os.path.join(
        home_dir,
        'doosan_manipulator/doosan_vision_grasp/doosan_vision_grasp/yolo_pose_node_d435i.py'
    )

    yolo_node = Node(
        package='doosan_vision_grasp',
        executable=python_executable,
        arguments=[node_path],
        name='yolo_pose_node_d435i',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'model_path': os.path.join(home_dir, 'doosan_manipulator', 'rot_v6.pt'),
            'scale_x': LaunchConfiguration('scale_x'),
            'scale_y': LaunchConfiguration('scale_y'),
            'depth_scale': 0.001,
            'min_valid_depth': LaunchConfiguration('min_valid_depth'),
            'max_valid_depth': 1.5,
            'depth_patch_k': 5,
            'depth_min_valid_px': 5,
            'max_sync_gap': 0.15,
            'kp_conf_threshold': 0.4,
            'yolo_conf': 0.25,
            'show_window': True,
        }],
    )

    return LaunchDescription([
        scale_x_arg,
        scale_y_arg,
        depth_profile_arg,
        color_profile_arg,
        min_depth_arg,
        realsense_camera,
        yolo_node,
    ])