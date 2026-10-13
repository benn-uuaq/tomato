"""토마토 픽앤플레이스 일괄 실행 (Elite CS616).

로봇 제어 노드 + 시퀀스 노드 + 웹 UI 를 한 번에 띄운다.

    ros2 launch elite_robot_controller tomato_pick_cs616.launch.py \
        robot_ip:=192.168.1.200

설정값은 config/tomato_pick.yaml 에서 읽는다. 다른 파일을 쓰려면
`pick_config:=/경로/내설정.yaml` 로 바꿔 준다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'elite_robot_controller'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    default_config = os.path.join(share, 'config', 'tomato_pick.yaml')

    robot_ip = LaunchConfiguration('robot_ip')
    modbus_port = LaunchConfiguration('modbus_port')
    pick_config = LaunchConfiguration('pick_config')
    ui_port = LaunchConfiguration('ui_port')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.200',
                              description='Elite CS616 컨트롤러 IP'),
        DeclareLaunchArgument('modbus_port', default_value='502',
                              description='Modbus TCP 포트(시뮬레이터는 5502)'),
        DeclareLaunchArgument('pick_config', default_value=default_config,
                              description='픽앤플레이스 설정 yaml'),
        DeclareLaunchArgument('ui_port', default_value='8080',
                              description='웹 UI 포트'),

        Node(
            package=PACKAGE, executable='robot_control_node',
            name='robot_control_node', output='screen',
            parameters=[{'robot_ip': robot_ip, 'modbus_port': modbus_port}],
        ),
        Node(
            package=PACKAGE, executable='pick_sequence_node',
            name='tomato_pick_sequence', output='screen',
            parameters=[pick_config],
        ),
        Node(
            package=PACKAGE, executable='pick_ui_node',
            name='tomato_pick_ui', output='screen',
            parameters=[pick_config, {'ui_port': ui_port}],
        ),
    ])
