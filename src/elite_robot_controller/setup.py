import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'elite_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # 웹 UI 화면은 파이썬 패키지 안에 함께 설치한다(ui_node 가 옆에서 읽는다).
    package_data={'elite_robot_controller.tomato': ['ui/*.html']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dusan_ws',
    maintainer_email='jing20c@gmail.com',
    description='Elite CS616 협동로봇 제어 및 토마토 픽앤플레이스 ROS 2 노드',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_control_node = elite_robot_controller.robot.robot_control_node:main',
            'pick_sequence_node = elite_robot_controller.tomato.pick_sequence_node:main',
            'pick_ui_node = elite_robot_controller.tomato.ui_node:main',
            'gripper_dynamixel_node = elite_robot_controller.tomato.gripper_dynamixel_node:main',
            'vision_bridge_client = elite_robot_controller.tomato.vision_bridge_client:main',
            'vision_panel_node = elite_robot_controller.tomato.vision_panel:main',
        ],
    },
)
