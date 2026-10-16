import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'doosan_vision_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # 토마토 측정 노드는 YOLO 가상환경의 파이썬으로 따로 띄운다(ROS 의
        # 엔트리포인트로 띄우면 시스템 파이썬이 잡혀 ultralytics 가 없다).
        # 그래서 소스 파일 자체를 share 에 설치해, launch 가 워크스페이스
        # 경로를 몰라도 찾을 수 있게 한다.
        (os.path.join('share', package_name, 'scripts'),
            ['doosan_vision_grasp/tomato_area_d435i_v2.py',
             # 2026-10 한양대 통합 비전 노드 (위치·최근접·중앙·무게 트리거)
             'doosan_vision_grasp/tomato_vision_d435i.py',
             'doosan_vision_grasp/tomato_vision_d435i_v2.py',
             'doosan_vision_grasp/tomato_vision_d435i_v3.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartfarm',
    maintainer_email='smartfarm@todo.todo',
    description='토마토 검출·무게 추정 비전 노드 (D435i + YOLO seg)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_pose_node_d405 = doosan_vision_grasp.yolo_pose_node_d405:main',
            'yolo_pose_node_d435i = doosan_vision_grasp.yolo_pose_node_d435i:main',
            'motor_command_node = doosan_vision_grasp.motor_command_node:main',
            'pick_place_motion = doosan_vision_grasp.doosan_delivery_node:main',
            'camera_to_base = doosan_vision_grasp.camera_to_base:main',
            'dyn_test = doosan_vision_grasp.dxl_test:main',
            'tomato_area = doosan_vision_grasp.tomato_area:main',
            'tomato_area_d435i = doosan_vision_grasp.tomato_area_d435i:main',
            'tomato_area_d435i_v2 = doosan_vision_grasp.tomato_area_d435i_v2:main',
            'open3d_tomato_viewer = doosan_vision_grasp.open3d_tomato_viewer:main',
            'tomato_volume_estimator = doosan_vision_grasp.tomato_volume_estimator:main',
            'yolo_seg_node_d435i = doosan_vision_grasp.yolo_seg_node_d435i:main',
            'tomato_area_d435i_test = doosan_vision_grasp.tomato_area_d435i_test:main',
            'tomato_align_test = doosan_vision_grasp.tomato_align_test_node:main',
            'tomato_align_test_v2 = doosan_vision_grasp.tomato_align_test_node_v2:main',
            'tomato_tcp_bridge = doosan_vision_grasp.tomato_tcp_bridge:main',
            
        ],
    },
)
