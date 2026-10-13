# elite_robot_controller

Elite CS612 협동로봇을 ROS 2에서 제어하기 위한 패키지입니다. AMR 위에 올라가는 매니퓰레이터의 제어부에 해당합니다.

`~/ws_elt`의 `robot_controller` 패키지에서 **드라이버, 제어 노드, launch 파일만** 이식했습니다. 원본의 `ui/`, `tasks/`(팔레타이징·언로딩·픽앤플레이스)는 ws_argo 프로젝트 잔재로 `argo_project_interfaces`에 의존하고 벽면 검사와 무관하여 가져오지 않았습니다.

## 통신 구조

로봇 컨트롤러와 세 개의 채널로 동시에 통신합니다. 세 채널이 모두 연결되어야 노드가 기동합니다.

| 포트 | 클래스 | 역할 |
| --- | --- | --- |
| 29999 | `Robot_29999` | Dashboard. 텍스트 명령으로 전원, 브레이크, 재생/일시정지/정지 제어 |
| 30001 | `Robot_30001` | Primary. 바이너리 패킷 스트림에서 로봇 상태와 알람 수집 |
| 502 | `Robot_modbus` | Modbus TCP. 레지스터 단위 상태 조회 및 설정 |

## 노드

### `robot_control_node`

10 Hz(`0.1초`) 주기로 Modbus 레지스터를 읽고 30001 알람을 수집하여 발행합니다.

**파라미터**

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_ip` | `192.168.227.134` | Elite 컨트롤러 IP |
| `modbus_port` | `502` | Modbus TCP 포트. 리눅스에서 502는 권한이 필요해 시뮬레이터 시험 시 바꿀 수 있게 뺐다 |
| `jog_joint_speed_max` 등 | — | 조그·홈 이동 관련 파라미터. 아래 "조그와 홈 이동" 절 참고 |

**발행 토픽**

| 토픽 | 타입 | 출처 |
| --- | --- | --- |
| `robot/status/robot_mode` | `std_msgs/Int32` | Modbus 66 |
| `robot/status/control_method` | `std_msgs/Int32` | Modbus 71 |
| `robot/status/operation_mode` | `std_msgs/Int32` | Modbus 72 |
| `robot/status/tcp_pose` | `std_msgs/Float32MultiArray` | Modbus 384~389 (현재 TCP, 기본 프레임) |
| `robot/status/tcp_pose_zero` | `std_msgs/Float32MultiArray` | Modbus 280~285 (원점 기준 상대 pose) |
| `robot/status/scan_state` | `std_msgs/Int32MultiArray` | Modbus 290~298 (로봇 태스크의 스캔 진행 상태) |
| `robot/status/task_state` | `std_msgs/Int32` | Modbus 500 (1 실행 중, 2 일시 중지, 3 중지됨) |
| `robot/status/speed_scale` | `std_msgs/Int32` | Modbus 17 (로봇 동작 속도 비율 %, 2~100) |
| `robot/status/joint_position` | `std_msgs/Float32MultiArray` | Modbus 73~78 (관절 각도) |
| `robot/status/alarms` | `std_msgs/String` | 30001 알람 |
| `robot/status/connected` | `std_msgs/Bool` | 세 채널 연결 여부 |

## Modbus 레지스터 맵

**주소는 코드에 두지 않고 [`config/modbus_registers.json`](config/modbus_registers.json) 한 곳에서만 관리한다.** 주소가 바뀌거나 새로 확인되면 이 파일만 고치면 되고 코드는 건드리지 않는다. 운영 UI도 같은 파일을 읽으므로 정의가 갈라지지 않는다.

`register_map` 파라미터로 다른 경로를 지정할 수 있다.

```bash
ros2 run elite_robot_controller robot_control_node --ros-args -p register_map:=/경로/my_registers.json
```

### 현재 내용

| 항목 | 주소 | 내용 |
| --- | --- | --- |
| `read.robot_mode` | 66 | 로봇 모드 |
| `read.control_method` | 71 | 제어 방식 |
| `read.operation_mode` | 72 | 운전 모드 |
| `read.joint_position` | 73~78 | 관절 각도 (베이스, 어깨, 엘보, 손목 1~3) |
| `read.tcp_absolute` | 384~389 | 현재 TCP (기본 프레임) |
| `read.tcp_zero_relative` | 280~285 | 원점 기준 상대 pose |
| `read.scan_state` | 290~298 | 스캔 진행 상태. `state, row_idx, rows, alive, zero_ok, finished, pitch, ...` 순서 |
| `read.task_state` | 500 | 태스크 상태(1 실행 중, 2 일시 중지, 3 중지됨). holding 이 아니라 **input register** 라 드라이버 폴백으로 읽힌다 |
| `read.speed_scale` | 17 | 로봇 동작 속도 비율[%] 2~100. **읽기 전용으로 쓴다** — 설정은 29999 `speed -v N` 으로 보낸다 |
| `write.linear_speed` | 306 | 작업 속도 [mm/s]. 태스크의 `movel` 속도 |
| `write.speed_ratio` | 307 | 로봇 자체 속도 비율 [%] 2~100 |
| `write.pose_src` | 308 | 1이면 태스크가 310~321의 기준 위치를 쓴다 |
| `write.home_joint` | 310~315 | 홈 관절값 [mrad] |
| `write.start_pose` | 316~321 | 시작 포즈 (길이 0.1 mm, 회전 mrad) |
| `write.work_area` | 256~259 | 작업 영역 [너비, 높이, 스캐너높이, 겹침] mm. **정수 mm 그대로** 저장한다(`kind: "raw"`, 0.1mm 아님) |
| `write.param_src` | 266 | 1이면 태스크가 256~259의 작업 영역을 쓴다 |

쓰기 주소는 로봇 태스크가 읽는 범용 레지스터 대역(256~383)에 맞췄습니다. 태스크가 이미 쓰는 256~305는 피했습니다.

`address`가 `null`인 항목은 주소 미확정을 뜻한다. **노드는 해당 요청을 거부하고 Modbus 접근 자체를 하지 않는다.** 엉뚱한 레지스터에 쓰면 로봇이 예기치 않게 움직이기 때문이다. 운영 UI도 같은 파일을 읽어 해당 버튼을 비활성화한다.

### 자세 값 환산

자세는 축마다 레지스터 1개씩 `[X, Y, Z, Rx, Ry, Rz]` 순서로 6개가 연속 배치된다. `Robot_modbus.get_all_registers()`가 부호 있는 16비트로 변환해 주므로 노드에서는 단위 환산만 한다.

항목의 `kind`가 환산 방법을 정한다. `pose`는 앞 3개를 위치, 뒤 3개를 회전으로 보고, `angle`은 6개 모두 회전으로 본다. 관절 각도는 6축 전부 mrad이므로 `angle`이다.

| 성분 | 설정 항목 | 환산 | 발행 단위 |
| --- | --- | --- | --- |
| X, Y, Z (`pose`) | `scale.position_per_count` | 레지스터 × 0.1 | mm |
| Rx, Ry, Rz (`pose`), 관절 6축 (`angle`) | `scale.rotation_per_count` | 레지스터 × 1.0 | mrad |

레지스터를 6개 모두 읽지 못하면 잘못된 자세를 내보내지 않도록 발행을 건너뛴다.

> **확인 필요:** 위치 환산 계수 0.1은 이식 전 코드의 값을 그대로 이어받은 것으로, 실장비에서 검증하지 않았다. 레지스터가 0.1 mm가 아니라 1 mm 단위라면 설정 파일의 `position_per_count`만 1.0으로 바꾸면 된다.

## 명령 인터페이스

| 종류 | 이름 | 쓰기 항목 |
| --- | --- | --- |
| 서비스 (`std_srvs/Trigger`) | `robot/command/save_home_pose` | `save_home_pose` |
| 서비스 (`std_srvs/Trigger`) | `robot/command/save_start_pose` | `save_start_pose` |
| 서비스 (`std_srvs/Trigger`) | `robot/command/move_home` | `move_home` |
| 토픽 (`std_msgs/Int32`) | `robot/command/linear_speed` | `linear_speed` |
| 토픽 (`std_msgs/Int32`) | `robot/command/speed_ratio` | `speed_ratio` |
| 토픽 (`std_msgs/Float32MultiArray`) | `robot/command/home_joint` | `home_joint` |
| 토픽 (`std_msgs/Float32MultiArray`) | `robot/command/start_pose` | `start_pose` |
| 토픽 (`std_msgs/Float32MultiArray`) | `robot/command/work_area` | `work_area` (성공 시 `param_src`도 1로 세운다) |
| 토픽 (`std_msgs/Int32`) | `robot/command/jog_joint` | 없음 (30001 스크립트) |
| 토픽 (`std_msgs/Int32`) | `robot/command/jog_tcp` | 없음 (30001 스크립트) |

값이 없는 한 번짜리 명령은 성공 여부를 돌려받아야 하므로 서비스로, 값이 있는 명령은 토픽으로 받습니다.

### 조그와 홈 이동

조그와 홈 이동은 레지스터가 아니라 **30001 포트로 스크립트를 보내** 처리합니다.

- 조그 값은 **부호가 방향, 절댓값이 축 번호(1~6)** 이며 `0`은 정지입니다.
- 움직일 때는 `speedj` / `speedl`을 보내고, **멈출 때는 29999의 `stop`** 을 씁니다. 감속 없이 즉시 서기 위해서입니다.
- **조그 속도는 레지스터 307(속도 비율 %)을 그대로 따릅니다.** 단위가 달라 아래 파라미터를 100 % 기준으로 두고 비율만큼 줄입니다.

| 파라미터 | 기본값 | 단위 | 대상 |
| --- | --- | --- | --- |
| `jog_joint_speed_max` | 0.50 | rad/s | `speedj` 의 `qd` |
| `jog_tcp_speed_max` | 0.10 | m/s | `speedl` 의 `xd` 앞 3개 |
| `jog_tcp_rot_speed_max` | 0.50 | rad/s | `speedl` 의 `xd` 뒤 3개 |
| `jog_accel_max` | 1.00 | rad/s² · m/s² | `a` |
| `jog_hold_time` | 0.5 | s | `t` |

예를 들어 비율이 20 %면 관절 조그는 0.1 rad/s, 가속도는 0.2가 됩니다.

`t`(`jog_hold_time`)는 **짧게 둡니다.** 매뉴얼상 로봇은 `t` 동안 계속 움직이므로, 길게 주면 정지 명령이 실패했을 때 그 시간만큼 멈추지 않습니다. 대신 운영 UI가 버튼을 누르고 있는 동안 명령을 되풀이해 보내고(150 ms 간격), 화면이 멈추거나 통신이 끊기면 되풀이가 끊겨 로봇도 `t` 안에 섭니다.
- `robot/command/move_home`은 태스크의 `move_home` 노드와 같은 기준을 씁니다: 레지스터 308(`pose_src`)이 1일 때만 310~315 override 관절값을 쓰고, 평소(0)에는 `Home_joint`/`Home_pose`를 씁니다. `Home_pose[2]`까지 `movel`로 올린 뒤 `Home_joint`(또는 override 관절값)로 `movej` 합니다. 예전엔 항상 310~315를 읽어, 운영자가 '위치 저장'을 한 번도 안 했으면 그 레지스터가 0으로 남아 있어 `movej([0,0,0,0,0,0], ...)`처럼 엉뚱한 관절로 가버리는 문제가 있었습니다.
  - 30001 소켓(실시간 명령 채널)에 `if`/`else`를 최상위로 그냥 보내면 한 줄씩 별개 명령으로 읽혀 실패합니다(`No 'if' command`). `def move_home_now(): ... end`로 감싼 **하나의 프로그램**으로 보내야 하고, 보낸 즉시 파싱이 끝나는 대로 실행되므로 따로 호출하면 안 됩니다(`move_home_now` 정의 안 됨 오류).
  - 이 프로그램은 태스크 preamble 밖에서 도는 별개 컨텍스트라 태스크가 저장해 둔 `Home_joint`/`Home_pose` 전역을 그냥 참조할 수 없습니다(`정의되지 않았습니다`). 대신 29999의 `variable -get Home_joint`/`variable -get Home_pose`로 로봇에 저장된 실제 값을 그때그때 읽어와(`Robot_29999.get_variable`), `global` 선언 뒤 그 자리에서 바로 대입해 이 스크립트만의 전역으로 만들어 씁니다. 값을 하드코딩하지 않으므로 태스크 쪽 `Home_joint`/`Home_pose`가 바뀌어도 자동으로 맞습니다. 읽기 자체가 실패했을 때만 `RobotControlNode._HOME_JOINT_FALLBACK`/`_HOME_POSE_FALLBACK` 고정값으로 대신합니다.
  - 변수 이름 `p`, `j`는 pose/joint 리터럴(`p[..]`, `j[..]`)과 충돌하니 피합니다.
- `robot/command/work_area`는 원통이 커서 AMR 원주 구역 + Cobot 세로 격자로 나눠 스캔할 때, MQTT `job_cmd`의 `grid` 블록에서 온 이번 격자의 너비/높이/스캐너높이/겹침을 전달한다. `work_area` 항목은 `kind: "raw"`라 다른 자세 항목과 달리 값을 그대로 정수 mm로 저장한다(0.1mm 단위 아님).

> `speedj`/`speedl`은 컨트롤러의 속도 백분율 설정에 영향을 받습니다(스크립트 매뉴얼 3.1.26/3.1.27). 100 %가 아니면 실제 속도가 그만큼 줄어듭니다.

**서비스** (모두 `std_srvs/Trigger`)

`robot/dashboard/` 아래:

| 서비스 | 동작 |
| --- | --- |
| `connect` / `disconnect` | 세 채널을 연결하거나 해제한다. Modbus 레지스터를 쓰지 않는다 |
| `robot_mode`, `status` | 29999 Dashboard 조회 명령 |
| `power_on`, `power_off`, `brake_release` | 전원과 브레이크 |
| `play`, `pause`, `stop` | 프로그램 제어 |

Dashboard 명령은 29999 소켓으로 나가므로 레지스터 주소와 무관하게 바로 쓸 수 있습니다. 반면 `robot/command/*`는 Modbus 레지스터에 쓰므로 주소가 정해져야 동작합니다.

## 실행

```bash
ros2 run elite_robot_controller robot_control_node --ros-args -p robot_ip:=192.168.227.134
```

### 로봇 없이 시험 (시뮬레이터)

실제 로봇 없이 UI와 노드를 함께 시험하려면 [`tools/elite_robot_simulator.py`](tools/elite_robot_simulator.py)를 먼저 띄운다. 29999/30001/Modbus 세 채널을 흉내 내고, 조그 명령을 받으면 관절·TCP 레지스터가 실제로 움직인다.

```bash
# 터미널 1: 가짜 로봇
python3 src/elite_robot_controller/tools/elite_robot_simulator.py

# 터미널 2: 노드 (502 대신 시뮬레이터의 5502로 접속)
ros2 run elite_robot_controller robot_control_node --ros-args \
    -p robot_ip:=127.0.0.1 -p modbus_port:=5502

# 터미널 3: 운영 UI
cd operator-ui && python -m smr_operator_ui
```

세 채널 모두 정상 연결되고, Cobot 조그 화면에서 버튼을 누르면 관절·TCP 값이 실시간으로 움직인다. 시뮬레이터는 레지스터 주소를 `config/modbus_registers.json`에서 그대로 읽으므로 노드와 다른 주소를 쓰는 일이 없다.

```bash
ros2 launch elite_robot_controller elite_cs612.launch.py robot_ip:=192.168.227.134
```

**로봇이 연결되어 있지 않아도 노드는 종료하지 않습니다.** 기동 시 한 번 연결을 시도하고, 실패하면 경고만 남긴 뒤 대기합니다. 운영 UI의 `연결` 버튼이나 `robot/dashboard/connect` 서비스로 다시 시도할 수 있습니다. 연결 전에는 Modbus 읽기를 건너뛰고 `robot/status/connected`로 상태만 알립니다.

## 의존성

`package.xml`의 `rclpy`, `std_msgs`, `std_srvs` 외에 **`pyModbusTCP`가 pip 의존**으로 필요합니다. rosdep 키가 없어 `package.xml`에 선언하지 않았습니다.

```bash
pip install pyModbusTCP
```

## 테스트

```bash
colcon test --packages-select elite_robot_controller && colcon test-result --verbose
```

`test_pose_publishing.py`는 Modbus 클라이언트를 대체해 로봇 없이 자세 레지스터 주소와 단위 환산을 검증한다.

**워크스페이스 가상환경의 `pytest`로 직접 실행하지 말 것.** ROS Humble의 `launch_testing` 플러그인은 pytest 6 API를 사용하는데 `operator-ui`가 pytest 8 이상을 요구해 같은 가상환경에서 충돌한다. `colcon test`는 정상 동작한다.

`flake8`과 `pep257` 검사는 현재 실패한다. `robot_driver.py`와 `robot_control_node.py`를 `ws_elt`에서 로직 수정 없이 이식했고 원본이 이미 스타일 규칙을 지키지 않았기 때문이다. 일괄 정리는 실장비 검증 이후로 미룬다.

## 주의

`robot_driver.py`는 Elite 컨트롤러의 바이너리 패킷 구조체 포맷(`FMT_*` 상수)에 강하게 결합되어 있습니다. 컨트롤러 펌웨어 버전이 바뀌면 파싱이 깨질 수 있으므로 포맷 문자열을 함부로 수정하지 마십시오.

---

# 토마토 픽앤플레이스 (Elite CS616)

외부 비전 토픽을 읽어 토마토를 집어 옮기는 시퀀스를 위 제어 노드 위에 올렸다.
`elite_robot_controller/tomato/` 아래가 새로 추가된 부분이고, 기존
`robot_control_node` 는 **스크립트 중계 토픽 하나**(`robot/command/script`)만
늘었다.

워크스페이스 전체 구성과 Orin nano 설치·실행은 [워크스페이스 README](../../README.md)
를 본다. 여기서는 시퀀스 노드 자체를 설명한다.

## 구성

| 노드 | 실행 파일 | 하는 일 |
| --- | --- | --- |
| 로봇 제어 | `robot_control_node` | 29999/30001/Modbus 세 채널을 **혼자** 들고 상태를 발행한다 |
| 시퀀스 | `pick_sequence_node` | 작업 순서를 상태기계로 진행한다. 움직임은 스크립트로 만들어 중계 토픽으로 넘긴다 |
| 그리퍼 | `gripper_dynamixel_node` | `/set_pos` (Int32) 를 받아 다이나믹셀 XD540 에 목표 위치를 쓴다 |
| UI | `pick_ui_node` | 브라우저로 여는 조작 화면(기본 8080) |
| 비전 브리지 | `vision_bridge_client` | 분리 구성에서 Orin 의 `tomato_tcp_bridge`(:9000) 에 접속해 촬영·그리퍼 명령을 보내고, 결과를 원래 토픽 3 개로 다시 내보낸다. 연결 상태는 `vision/bridge/connected` |

소켓을 한 노드가 독점하는 이유는 같은 포트에 둘이 붙으면 명령이 섞이기
때문이다. 시퀀스 노드는 `robot/command/script` 로 문자열만 보내고, 이동
완료는 `robot/status/tcp_pose` 로 들어오는 실제 TCP 값이 목표에
`settle_time_s` 동안 머물렀는지로 판정한다.

```bash
ros2 launch elite_robot_controller tomato_system.launch.py   # 전체
# 또는 워크스페이스 루트에서:  ./run.sh
```

## 작업 순서

17 단계다. 단계표·하강 거리 계산·현장 확인 항목은
[워크스페이스 README 의 "작업 순서"](../../README.md#작업-순서) 에 있다. 코드는

| 파일 | 하는 일 |
| --- | --- |
| `tomato/pick_sequence_node.py` | 상태기계. 상태 이름은 `STEP_ORDER` (UI 가 이 순서로 그린다) |
| `tomato/plan.py` | 목표 자세 계산 — 중심 맞추기·파지점·하강은 **TCP 기준**, 놓기는 베이스 Z |
| `tomato/motion.py` | Elite 스크립트 — `movel`, 경유점을 거치는 `joint_path_program` |
| `tomato/geometry.py` | RPY / 회전 벡터, DH 정기구학 |

픽 쪽 이동은 TCP 기준이라 **로봇이 보고하는 회전 표기**가 맞아야 한다
(`orientation_format`, 기본 `rpy`). `tools/check_pose_convention.py` 로 로봇을
움직이지 않고 확인한다.

이동 완료 판정은 직선 이동이면 실제 TCP, 관절 이동이면 실제 관절값으로 한다
(`pose_tolerance_mm`, `angle_tolerance_deg`, `joint_tolerance_deg`).

## 명령과 상태

| 종류 | 이름 | 내용 |
| --- | --- | --- |
| 토픽 (`std_msgs/Empty`) | `pick/trigger` | 한 사이클 시작 |
| 토픽 (`std_msgs/String`) | `pick/command` | `start` / `loop` / `stop` / `pause` / `resume` |
| 서비스 (`std_srvs/Trigger`) | `pick/start` | 한 번만 작업 |
| 서비스 | `pick/start_loop` | 정지할 때까지 반복 |
| 서비스 | `pick/pause` / `pick/resume` | 29999 `stop` 으로 멈추고, 재개하면 **그 단계를 처음부터** 다시 실행한다 |
| 서비스 | `pick/stop` | 정지하고 대기 상태로 |
| 토픽 (`std_msgs/String`) | `pick/status` | 상태·단계·대상 토마토·중심 오차·비어 있는 설정을 JSON 으로 (5 Hz) |
| 토픽 (`std_msgs/String`) | `pick/result` | 사이클 종료 결과(성공 여부·무게·높이) |
| 토픽 (`std_msgs/Bool`) | `pick/done` | 사이클 종료 신호 |

설정이 하나라도 비어 있으면 `pick/start` 는 **거절한다.** 무엇이 비었는지는
`pick/status` 의 `missing` 과 UI 아래쪽 카드에 나온다.

## 비전 인터페이스

`doosan_vision_grasp` 의 `tomato_area_d435i_v2` 노드에 맞춰 두었다.

| 방향 | 토픽 | 타입 | 단위 |
| --- | --- | --- | --- |
| 로봇 → 비전 | `/tomato/capture` | `std_msgs/Empty` | 트리거. 약 1 초간 측정 |
| 비전 → 로봇 | `/tomato/center_positions` | `geometry_msgs/PoseArray` | **m**, 카메라 optical frame |
| 비전 → 로봇 | `/tomato/estimated_weights` | `std_msgs/Float32MultiArray` | **g** |
| 비전 → 로봇 | `/tomato/heights` | `std_msgs/Float32MultiArray` | **m** |

세 토픽은 같은 순서로 한 번씩 오고, `VisionClient` 가 인덱스로 합쳐
`Detection` 하나로 만든다. 위치가 먼저 오고 무게가 조금 늦게 올 수 있어
0.2 초 기다렸다가 합친다.

좌표 해석 방식은 `vision.detection_frame` 이 정한다.

- **`camera_delta`** (기본): 카메라 X·Y 를 **지금 자리에서 움직일 양**으로 바로
  쓴다. `camera_axis_map` 이 축 대응이고 기본값 `["-y", "-x"]` 가 README 의
  "(x, y) → (-y, -x)" 다. 장착 방향이 다르면 **이 두 값만** 바꾸면 된다.
  Z 는 건드리지 않는다.
- `camera` / `tool` / `base`: 핸드인아이 변환(`hand_eye_xyz_mm`,
  `hand_eye_rpy_deg`)을 제대로 태우는 일반 방식. 카메라가 기울어 달렸거나
  툴 자세가 자주 바뀔 때 쓴다.

메시지 구조가 바뀌면 **`vision_interface.parse_detections` 한 곳만** 고치면
된다(`PoseArray`, `Float32MultiArray`, `Detection3DArray` 를 알아본다).

## 그리퍼

다이나믹셀 **XD540-T270** 이고 `/set_pos` (Int32) 로 목표 위치를 받는다.
README 기준 **3185 파지 / 0 열기**. `gripper_dynamixel_node` 가 이 토픽을
받아 모터에 쓴다(포트·ID·보드레이트는 파라미터).

`gripper.backend` 로 다른 방식으로 바꿀 수 있다.

- `int_topic`(기본): Int32 목표 위치
- `topic`: `std_msgs/Bool` (true 닫기)
- `serial`: Orin 에서 직접 시리얼 문자열 전송(`serial_port`, `close_command`, `open_command`)
- `none`: 명령을 내보내지 않고 로그만 — 로봇 동작만 먼저 볼 때

피드백 신호가 없으므로 동작 완료는 `action_wait_s` 시간으로 본다.

## 위치 가르치기 (펜던트 → 저장)

홈·대기·놓는 자리·경유점은 **로봇을 그 자리에 세워 두고 버튼 한 번**으로
저장한다. 이 명령들은 **지금 TCP·관절값을 읽기만 하고 로봇을 움직이지 않는다.**
작업 중에는 거절한다.

| 서비스 (`std_srvs/Trigger`) | 저장되는 값 |
| --- | --- |
| `pick/teach/home_pose` | 관절 6 축 |
| `pick/teach/wait_pose` | TCP 6 축 + 관절 6 축 |
| `pick/teach/place_pose` | TCP 6 축 + 관절 6 축 |
| `pick/teach/waypoint_wait_add` / `_undo` / `_clear` | ★ 홈↔대기 경유점 (관절) |
| `pick/teach/waypoint_place_add` / `_undo` / `_clear` | ★ 홈↔놓는 자리 경유점 (관절) |
| `pick/teach/clear` | 모두 지우고 설정 파일 값으로 |

놓기 접근 높이·여는 높이는 시퀀스 노드의 파라미터 `place_approach_mm`,
`place_release_gap_mm` 를 바꾸면(UI 의 [놓기 높이 저장]) 검사한 뒤 파일에도 남는다.

저장 위치는 `~/.tomato_pick/taught_poses.yaml` (사람이 읽을 수 있는 yaml, 관절·회전
deg, 위치 mm). **우선순위는 `tomato_pick.yaml` < 가르친 파일** 이다.

## 설정 — [`config/tomato_pick.yaml`](config/tomato_pick.yaml)

사람이 쓰는 단위는 **mm 와 deg** 이고, 스크립트 단위(m·rad)로는 노드가
한 번만 환산한다. pose 의 회전은 회전 벡터라 펜던트의 TCP 값을 그대로
넣으면 된다.

현장에서 채울 값은 ★ 로 표시해 두었다 — 대기 자세, 놓는 자리, 하강 높이,
그리고 축 대응·그리퍼 오프셋 확인이다. 비전·그리퍼 토픽 이름은 실제
인터페이스로 이미 채워져 있다.

## 로봇 없이 시험

시뮬레이터가 `movel`/`movej(p[...])` 를 받아 TCP 레지스터를 **설정 속도로**
움직이도록 확장했다. 순간이동시키면 도착 판정을 시험할 수 없기 때문이다.

```bash
python3 tools/elite_robot_simulator.py
# 워크스페이스 루트에서
./run.sh camera:=false vision:=false gripper:=false \
         robot_ip:=127.0.0.1 modbus_port:=5502
```

계산만 보는 시험은 로봇도 ROS 도 없이 돈다.

```bash
python3 -m pytest test/test_tomato_pick.py
```

## Elite 스크립트 문법 (실장비에서 확인)

30001 로 보내는 스크립트는 **URScript 와 미묘하게 다르다.** 틀리면 컨트롤러가
스크립트를 조용히 버리고 **아무 오류 없이 로봇만 안 움직인다** — 30001 의
메시지 패킷(type 20)을 읽어야 사유가 보인다.

| 하면 안 되는 것 | 해야 하는 것 | 컨트롤러가 내는 말 |
| --- | --- | --- |
| `movel(p[...], ...)` | `movel([...], ...)` | `type_error_arg_not_subscriptable: builtin_function_or_method` |
| `movej(pose, ...)` | `movej(get_inverse_kin([...]), ...)` | (관절값을 받는 함수다) |

`p` 는 Elite 에서 **내장 함수 이름**이라 `p[...]` 로 쓰면 "함수에 첨자를
붙였다"는 오류가 난다. pose 는 그냥 리스트 `[x, y, z, rx, ry, rz]` 다
(위치 m, 회전 rotvec rad).

움직이지 않을 때 확인할 순서:

1. `remoteControl -status` 가 `true` 인가 (29999)
2. `task -s` 가 `Task is stopped` 인가 — 태스크가 돌면 스크립트와 섞인다
3. 30001 메시지 패킷에 오류가 오는가 — 위 표의 문법 오류가 여기 나온다
4. `robotMode: RUNNING`, `SafetyMode: NORMAL` 인가

## 현장에서 확인할 것

- ~~TCP 레지스터 환산 계수 0.1~~ — **CS616 실장비에서 확인 완료**(2026-09-21).
  30001 스트림의 `0.400 / 0.000 / 0.525 m` 와 Modbus 384~389 원시값
  `4000 / 0 / 5250` 이 정확히 `×0.1 mm` 로 일치했고, 회전도 mrad 그대로였다.
  관절(73~78)도 mrad 이다.
- **레지스터는 16 비트**라 위치는 ±3276.7 mm 까지만 읽힌다. 작업 영역이
  그 밖이면 상태 읽기를 30001 실시간 스트림으로 바꿔야 한다.
- **`camera_axis_map` 과 `gripper.tcp_offset_mm`** 은 두산 설치 기준 값이다.
  CS616 에 카메라·그리퍼를 다시 달았다면 `align_iterations` 를 0 으로 두고
  한 번 찍어 보며 부호부터 확인한다.
- 하강은 이제 **TCP +z 로 카메라 깊이만큼**이다(베이스 Z 하강 한계 없음).
  `descend.grasp_depth_mm`(30) 은 저속으로 집어 보며 맞춘다.
- **30001 소켓이 끊기면** 모든 이동 명령이 "전송 실패" 로 흘러간다. 상태
  읽기는 조용히 실패만 하므로 아무도 눈치채지 못했다 — 이제 `send_script`
  가 끊긴 소켓을 다시 붙여 한 번 더 보낸다(2026-09-21 수정).
- 3 단계는 **검출 목록의 첫 번째**를 집는다. 정렬 기준(거리순·크기순)이
  따로 있으면 `run_wait_detect` 에서 고르는 부분만 바꾸면 된다.
