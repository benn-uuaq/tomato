import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------------- Launch 인자 ----------------
    depth_profile_arg = DeclareLaunchArgument('depth_profile', default_value='848x480x30')
    color_profile_arg = DeclareLaunchArgument('color_profile', default_value='848x480x30')
    conf_arg = DeclareLaunchArgument('yolo_conf', default_value='0.4')

    # ---------------- 리얼센스 카메라 ----------------
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',

            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),

            'depth_module.emitter_enabled': '1',

            # 오토 화이트 밸런스 비활성화 및 색온도 수동 고정
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

    # ---------------- Segmentation 노드 ----------------
    home_dir = os.path.expanduser('~')
    python_executable = os.path.join(home_dir, 'doosan_manipulator', 'venv_yolo', 'bin', 'python3')
    node_path = os.path.join(
        home_dir,
        'doosan_manipulator/doosan_vision_grasp/doosan_vision_grasp/yolo_seg_node_d435i.py'
    )
    seg_model_path = os.path.join(
        home_dir,
        'doosan_manipulator/Dataset/StrawDI_Db1/yolo_dataset/runs/segment/'
        'strawberry_seg/strawdi_v1-2/weights/best.pt'
    )

    seg_node = Node(
        package='doosan_vision_grasp',
        executable=python_executable,
        arguments=[node_path],
        name='yolo_seg_node_d435i',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'model_path': seg_model_path,
            'yolo_conf': LaunchConfiguration('yolo_conf'),
            'show_window': True,
            'mask_alpha': 0.45,
        }],
    )

    return LaunchDescription([
        depth_profile_arg,
        color_profile_arg,
        conf_arg,
        realsense_camera,
        seg_node,
    ])
