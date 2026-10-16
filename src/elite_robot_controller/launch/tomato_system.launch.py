"""토마토 수확 시스템 실행 — 켤 부품을 골라 한 번에 띄운다.

    ros2 launch elite_robot_controller tomato_system.launch.py

부품:

1. RealSense D435i (`realsense2_camera`, depth-color 정렬 켬)        camera
2. 토마토 측정 노드 (`doosan_vision_grasp`, **YOLO 가상환경 파이썬**)  vision
3. 다이나믹셀 그리퍼 노드 (`/set_pos`)                                gripper
4. Elite 로봇 제어 노드 + 픽앤플레이스 시퀀스                           robot
5. 웹 UI (기본 8080)                                                 ui
6. 비전 PC <-> 로봇 PC TCP 브리지                                     bridge

분리 구성(비전은 Orin, 로봇은 PC)에서는 `run.sh` 가 장비에 맞춰 골라 준다.

    Orin:  camera vision gripper  + bridge:=server   (robot:=false ui:=false)
    PC  :  robot ui               + bridge:=client   (camera/vision/gripper:=false)

**꺼 둔 부품은 찾지도 않는다.** 로봇 PC 에는 카메라 패키지도 YOLO 가상환경도
없는 게 정상이라, 그걸 찾다가 "없다" 고 알리면 헷갈리기만 한다. 그래서 실제
노드 목록은 인자를 다 읽은 뒤(OpaqueFunction) 켠 것만으로 만든다.

가상환경 파이썬과 모델 경로는 `scripts/env.sh` 가 만들어 주는 환경변수
(`TOMATO_VENV_PY`, `TOMATO_MODEL`)를 기본값으로 쓴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'elite_robot_controller'
VISION_PACKAGE = 'doosan_vision_grasp'


def _default(env_name, fallback):
    return os.environ.get(env_name, fallback)


def _share(package):
    """패키지 share 경로. 없으면 None."""
    try:
        return get_package_share_directory(package)
    except Exception:
        return None


def _launch_setup(context):
    def value(name):
        return context.perform_substitution(LaunchConfiguration(name))

    def enabled(name):
        return value(name).strip().lower() in ('true', '1', 'yes', 'on')

    pick_config = value('pick_config')
    bridge = value('bridge').strip().lower()
    auto_connect = enabled('auto_connect')
    bridge_port = int(value('bridge_port'))
    actions = []

    # ---- 1. RealSense D435i ------------------------------------------
    if enabled('camera'):
        share = _share('realsense2_camera')
        if share:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rs_launch.py')),
                launch_arguments={
                    'align_depth.enable': 'true',   # 깊이를 컬러 픽셀에 맞춘다
                    'pointcloud.enable': 'false',   # 부하 최소화
                    'enable_color': 'true',
                }.items(),
            ))
        else:
            print('[tomato_system] realsense2_camera 를 찾지 못해 카메라는 건너뜁니다.')

    # ---- 2. 토마토 측정 노드 (가상환경 파이썬) -------------------------
    if enabled('vision'):
        share = _share(VISION_PACKAGE)
        python = value('venv_python')
        if not os.path.exists(python):
            # 가상환경이 없으면 시스템 파이썬으로 돌려 본다(ultralytics 가
            # 없으면 비전 노드만 죽는다).
            print(f'[tomato_system] 가상환경 파이썬이 없어({python}) '
                  '시스템 python3 로 비전 노드를 띄웁니다.')
            python = 'python3'
        if share:
            actions.append(Node(
                package=VISION_PACKAGE,
                executable=python,
                arguments=[os.path.join(share, 'scripts', 'tomato_area_d435i_v2.py')],
                name='tomato_area_calculator_d435i_v2',
                output='screen', emulate_tty=True,
                parameters=[pick_config, {'model_path': value('model_path')}],
            ))
        else:
            print(f'[tomato_system] {VISION_PACKAGE} 를 찾지 못해 비전 노드는 건너뜁니다.')

    # ---- 3. 그리퍼 ----------------------------------------------------
    if enabled('gripper'):
        actions.append(Node(
            package=PACKAGE, executable='gripper_dynamixel_node',
            name='gripper_dynamixel', output='screen',
            parameters=[pick_config],
        ))

    # ---- 4. 로봇 제어 + 시퀀스 -----------------------------------------
    if enabled('robot'):
        actions.append(Node(
            package=PACKAGE, executable='robot_control_node',
            name='robot_control_node', output='screen',
            parameters=[{'robot_ip': value('robot_ip'),
                         'modbus_port': int(value('modbus_port')),
                         'auto_connect': auto_connect}],
        ))
        actions.append(Node(
            package=PACKAGE, executable='pick_sequence_node',
            name='tomato_pick_sequence', output='screen',
            parameters=[pick_config],
        ))

    # ---- 5. UI --------------------------------------------------------
    if enabled('ui'):
        actions.append(Node(
            package=PACKAGE, executable='pick_ui_node',
            name='tomato_pick_ui', output='screen',
            # 화면에 처음 채울 주소(기억해 둔 주소가 있으면 그걸 쓴다).
            parameters=[pick_config, {'ui_port': int(value('ui_port')),
                                      'robot_ip': value('robot_ip'),
                                      'vision_host': value('vision_host'),
                                      'connect_on_start': enabled('connect_on_start')}],
        ))

    # ---- 6. 비전 PC <-> 로봇 PC TCP 브리지 ------------------------------
    if bridge == 'server':
        if _share(VISION_PACKAGE):
            actions.append(Node(
                package=VISION_PACKAGE, executable='tomato_tcp_bridge',
                name='tomato_tcp_bridge', output='screen',
                parameters=[{'host': '0.0.0.0', 'port': bridge_port}],
            ))
        else:
            print(f'[tomato_system] {VISION_PACKAGE} 가 없어 브리지 서버를 띄우지 못했습니다.')
    elif bridge == 'client':
        actions.append(Node(
            package=PACKAGE, executable='vision_bridge_client',
            name='vision_bridge_client', output='screen',
            parameters=[{'host': value('vision_host'), 'port': bridge_port,
                         'auto_connect': auto_connect}],
        ))
    elif bridge != 'none':
        print(f'[tomato_system] bridge 는 none / server / client 중 하나여야 합니다: {bridge}')

    return actions


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    workspace = _default('TOMATO_WS', os.path.expanduser('~/tomato_ws'))

    arguments = [
        DeclareLaunchArgument('robot_ip', default_value=_default('ROBOT_IP', '192.168.1.200'),
                              description='Elite CS616 컨트롤러 IP'),
        DeclareLaunchArgument('modbus_port', default_value='502',
                              description='Modbus TCP 포트(시뮬레이터는 5502)'),
        DeclareLaunchArgument('pick_config',
                              default_value=os.path.join(share, 'config', 'tomato_pick.yaml'),
                              description='픽앤플레이스 설정 yaml'),
        DeclareLaunchArgument('ui_port', default_value='8080', description='웹 UI 포트'),
        DeclareLaunchArgument('venv_python',
                              default_value=_default('TOMATO_VENV_PY', os.path.join(
                                  workspace, 'venv', 'bin', 'python3')),
                              description='YOLO 가상환경의 python3'),
        DeclareLaunchArgument('model_path',
                              default_value=_default('TOMATO_MODEL', os.path.join(
                                  workspace, 'models', 'tomato_v1.pt')),
                              description='YOLO 토마토 모델(.pt)'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense 실행 여부'),
        DeclareLaunchArgument('vision', default_value='true', description='토마토 측정 노드 실행 여부'),
        DeclareLaunchArgument('gripper', default_value='true', description='그리퍼 노드 실행 여부'),
        DeclareLaunchArgument('robot', default_value='true', description='로봇·시퀀스 노드 실행 여부'),
        DeclareLaunchArgument('ui', default_value='true', description='웹 UI 실행 여부'),
        # 분리 구성: 비전 PC(Orin) 는 server, 로봇 PC 는 client. 한 대에서
        # 다 돌릴 때는 none (같은 ROS 그래프라 브리지가 필요 없다).
        DeclareLaunchArgument('bridge', default_value='none',
                              description='TCP 브리지 역할: none | server | client'),
        DeclareLaunchArgument('vision_host', default_value=_default('VISION_HOST', '192.168.1.10'),
                              description='client 일 때 접속할 비전 PC(Orin) 주소'),
        DeclareLaunchArgument('bridge_port', default_value=_default('BRIDGE_PORT', '9000'),
                              description='브리지 TCP 포트'),
        # false 면 로봇·Orin 에 바로 붙지 않고 UI 에서 IP 를 넣고 [연결] 을 누를 때 붙는다.
        DeclareLaunchArgument('auto_connect', default_value='true',
                              description='기동하자마자 로봇·비전 PC 에 연결할지'),
        # auto_connect:=false 인 로봇 PC 에서, UI 가 기억해 둔 주소로 [연결] 을 대신 누른다.
        DeclareLaunchArgument('connect_on_start', default_value='false',
                              description='UI 가 뜨자마자 기억한 주소로 연결할지 (robot_run.sh --connect)'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
