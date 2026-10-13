# 토마토 수확 워크스페이스 (Elite CS616 + D435i + 다이나믹셀 그리퍼)

비전과 로봇을 **두 장비로 나눠** 돌린다. 두 장비는 랜선으로만 이어지고,
같은 폴더를 양쪽에 두고 각자 `./run.sh` 하나만 치면 장비가 자기 몫을 띄운다.

```
  ┌──────── 비전 PC (Orin nano) ────────┐        ┌────── 로봇 PC (Windows + WSL) ──────┐
  │ RealSense D435i ─► YOLO 토마토 검출  │        │  시퀀스 ─► 로봇 제어 ─► Elite CS616 │
  │ 그리퍼 (USB, /set_pos)              │  TCP   │   ▲  │      (29999/30001/502)      │
  │ tomato_tcp_bridge  (서버 :9000)  ◄──┼────────┼── vision_bridge_client           │
  └─────────────────────────────────────┘ 9000   │  웹 UI  http://localhost:8080      │
                                                 └─────────────────────────────────────┘
```

| 장비 | `./run.sh` 가 띄우는 것 | 화면 |
| --- | --- | --- |
| 비전 PC (Orin) | **비전 패널** — 카메라 · YOLO · 그리퍼 · 외부 통신을 버튼으로 켜고 끈다 | `http://<Orin IP>:8081` |
| 로봇 PC (WSL) | 로봇 제어 · 작업 시퀀스 · 작업 UI · 브리지 클라이언트 | `http://localhost:8080` |

**작업 순서**

1. Orin: `./run.sh` → 패널에서 **[전체 시작]** → 네 부품이 모두 "준비됨"
2. 로봇 PC: `./run.sh` → 작업 UI 에서 로봇 IP · Orin IP 넣고 **[연결]**
3. 로봇 PC 작업 UI 에서 **[작업 시작]**

역할은 장비를 보고 정한다(Jetson → 비전 PC, 그 외 → 로봇 PC). 한 대에서
전부 돌리려면 `./run.sh --role all`.

## 두 장비가 주고받는 것

TCP 브리지는 비전 팀이 만들어 둔 `tomato_tcp_bridge` 를 그대로 쓴다(JSON Lines,
포트 9000). 그리퍼 USB 가 Orin 에 꽂혀 있어 그리퍼 명령만 하나 더 붙였다.

| 방향 | 내용 |
| --- | --- |
| 로봇 PC → Orin | `{"cmd":"capture"}` 촬영 요청 · `{"cmd":"gripper","position":3185}` 그리퍼 · `{"cmd":"ping"}` |
| Orin → 로봇 PC | `{"type":"result", "tomatoes":[{position(m, 카메라 좌표), weight_g, height_m}...]}` |

**카메라 좌표 → 로봇 좌표 변환은 로봇 PC 에서 한다.** 로봇 PC 가 지금 TCP 를
알고 있기 때문이다. Orin 은 카메라 좌표계 결과만 던진다.

받은 결과는 로봇 PC 에서 원래 비전 노드와 **같은 토픽 3 개**로 다시 나오므로,
시퀀스는 비전이 어디에 있든 똑같이 동작한다.

**연결이 끊기면 안전하게 멈춘다.** 작업 중에 Orin 이 끊기면 시퀀스가 로봇을
세우고("비전 PC(Orin) 연결이 끊겼습니다"), 끊긴 동안은 작업 시작을 거절한다.
Orin 이 돌아오면 2 초 안에 저절로 다시 붙는다. UI 에 "비전 PC 연결됨/끊김"
배지가 보인다.

## 네트워크

**로봇 PC 가 모든 연결을 먼저 거는 쪽**이다(로봇에도, Orin 에도). WSL2 는 기본
NAT 네트워크에서 바깥으로 나가는 연결은 그대로 되므로 **WSL 쪽 설정이 필요
없다** — mirrored 모드도, 포트포워딩도, Windows 방화벽 규칙도 안 건드린다.
Windows 의 유선 랜에 주소만 있으면 된다.

권장 배선 — **스위치 허브 하나에 셋 다** 꽂고 같은 대역으로:

| 장비 | 주소 | 설정 위치 |
| --- | --- | --- |
| Elite CS616 | 192.168.1.200 | 펜던트 |
| Orin (`enP8p1s0`) | 192.168.1.10 | `robot-lan` 프로필 (만들어 둠, 아래) |
| 로봇 PC (Windows 유선 랜) | 192.168.1.20 | Windows 네트워크 설정 |

Windows 쪽은 관리자 PowerShell 에서 (어댑터 이름은 `Get-NetAdapter` 로 확인):

```powershell
New-NetIPAddress -InterfaceAlias "이더넷" -IPAddress 192.168.1.20 -PrefixLength 24
```

배선이 다르면 `scripts/env.sh` 의 세 줄만 고친다.

```bash
export ROBOT_IP="${ROBOT_IP:-192.168.1.200}"      # 로봇
export VISION_HOST="${VISION_HOST:-192.168.1.10}" # Orin — 로봇 PC 가 여기 접속
export BRIDGE_PORT="${BRIDGE_PORT:-9000}"
```

예) 로봇만 PC 에 직결하고 Orin 은 Wi-Fi 공유기로 붙인다면 `VISION_HOST` 를
Orin 의 Wi-Fi 주소(지금 192.168.0.103)로.

두 장비의 ROS 는 서로를 보지 않는다(`ROS_LOCALHOST_ONLY=1`). 잇는 건 TCP
브리지 하나뿐이다 — 같은 토픽 이름이 양쪽에 있어 서로 보이면 명령이 메아리처럼
되돌아오기 때문이다.

Orin 유선 주소는 아래 프로필로 고정해 두었다(재부팅해도 유지). `ip addr add`
로 붙인 주소는 NetworkManager 가 지워 버리므로 프로필이어야 한다.

```bash
sudo nmcli con add type ethernet ifname enP8p1s0 con-name robot-lan \
    ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.never-default yes \
    ipv6.method ignore connection.autoconnect yes
```

## 준비 (장비마다 한 번)

```bash
cd <워크스페이스> && bash scripts/setup.sh
```

같은 스크립트가 장비를 보고 다르게 동작한다.

| 단계 | 비전 PC (Orin) | 로봇 PC (WSL) |
| --- | --- | --- |
| ROS 노드용 의존 (`requirements/ros.txt`, 시스템 파이썬) | O | O |
| 가상환경 + YOLO 의존 | O — **torch 는 건드리지 않는다**(JetPack 휠) | **안 한다** (torch 불필요) |
| `colcon build --symlink-install` | O | O |
| 폴더에 들어오면 자동 환경 | direnv 있으면 `.envrc` 승인, 없으면 `~/.bashrc` 훅 | 〃 |
| 주변 확인 | 카메라 패키지·시리얼 권한·모델 파일 | 로봇·Orin 브리지 도달 여부 |

`direnv: error ... .envrc is blocked` 가 뜨면 `setup.sh` 를 한 번 돌리거나
`direnv allow` 하면 된다.

ROS 노드용 의존을 시스템 파이썬에 까는 이유: colcon 이 만든 ROS 실행 파일의
shebang 이 `/usr/bin/python3` 라서, 가상환경에만 깔면 `pyModbusTCP` 를 못 찾는다.

Orin 으로 폴더를 옮길 때는 빌드 결과를 빼고 보낸다(이 PC 의 절대 경로가 박혀
있다). 실수로 같이 보내도 `setup.sh` 가 알아보고 지운 뒤 다시 빌드한다.

```bash
rsync -av --exclude build --exclude install --exclude log --exclude venv \
      --exclude '*.zip' --exclude '*.odt' --exclude '*Zone.Identifier' \
      ~/ws_hy/ nano@<ORIN_IP>:~/tomato_ws/
```

## 실행

양쪽 모두 같은 명령이다.

```bash
./run.sh                 # 자기 역할만 띄운다
./run.sh --restart       # 돌고 있던 것을 내리고 다시
./stop.sh                # 내리기만
./run.sh --role all      # 한 대에서 전부
```

조작은 로봇 PC 브라우저에서 **`http://localhost:8080`**.

**로봇 PC 는 UI 부터 뜨고, 연결은 화면에서 한다.** `./run.sh` 직후에는 로봇에도
Orin 에도 접속하지 않는다. 맨 위 "연결" 카드에

1. **로봇 IP** (예: 192.168.1.200)
2. **Orin IP** (예: 192.168.1.10)

를 넣고 **[연결]** 을 누르면 그때 붙는다. 상태는 카드 아래에 "로봇 연결됨 /
Orin 연결됨" 으로 나온다.

- 마지막에 연결한 주소는 `~/.tomato_pick/connection.yaml` 에 남아 다음에 미리
  채워진다.
- 주소 형식이 틀리면(예: `192.168.1.2000`) 접속을 시도하지 않고 바로 알려 준다.
- 상대가 아직 안 켜져 있어도 [연결] 을 누른 뒤에는 2 초마다 다시 시도한다.
  멈추려면 **[연결 해제]**.
- 연결되기 전이나 [연결 해제] 뒤에는 작업 시작을 거절한다.
- 주소를 바꿔 다시 [연결] 하면 이전 연결을 끊고 새 주소로 옮긴다.

Orin 쪽은 그냥 `./run.sh` 로 띄워 두면 된다(브리지 서버가 접속을 기다린다).
한 대 구성(`--role all`)과 Orin 은 예전처럼 기동하자마자 연결한다.

Orin 화면에서 실행하면 YOLO 미리보기 창이 뜬다(SSH 로 실행하면 창 없이 계산만).

## 비전 패널 (Orin)

README.odt 의 수동 절차(터미널 여러 개에서 launch / run / topic pub)를 버튼으로
옮긴 화면이다. Orin 에서 `./run.sh` 하면 뜨고, Orin 화면에서 실행했다면 창이
자동으로 열린다. 로봇 PC 브라우저에서도 `http://192.168.1.10:8081` 로 열린다.

| 부품 | 예전 명령 | 패널 | "준비됨" 으로 보는 조건 |
| --- | --- | --- | --- |
| 카메라 | `tomato_area_d435i_v2.launch.py` 의 앞부분 | 시작 / 중지 / 재시작, 미리보기 | 카메라 영상(camera_info)이 실제로 들어옴 |
| YOLO | 같은 launch 의 뒷부분 | 시작 / 중지, **촬영 시험** + 결과 표 | 모델을 다 불러와 촬영 요청을 받을 수 있음 |
| 그리퍼 | `dyn_test` + `topic pub /set_pos` | **열기 / 닫기 / 위치 직접 입력** | 모터가 현재 위치를 답함 |
| 외부 통신 | TCP 브리지 노드 | 시작 / 중지 | 포트가 열림 — 로봇 PC 가 붙으면 그 주소 |

- **켜졌다 ≠ 준비됐다.** 프로세스가 떠 있어도 카메라 USB 가 빠졌거나 모터
  전원이 없으면 "문제 있음" 과 이유(예: "카메라를 찾지 못했습니다 — USB 연결
  확인", "모터가 응답하지 않습니다 — 전원·ID·보드레이트 확인")를 보여 준다.
- 각 부품의 로그는 카드의 [로그] 를 펼치면 보인다.
- 카메라 영상이 없으면 [촬영 시험] 은 거절한다 — YOLO 가 촬영 요청을 조용히
  무시해 결과가 영영 안 오기 때문이다.
- [시작] 할 때 패널 밖에서 같은 장치를 잡고 있는 프로세스(예전 `dyn_test`,
  남은 카메라 노드)가 있으면 먼저 정리한다. 같은 카메라·시리얼 포트를 둘이
  잡으면 둘 다 망가진다.
- 미리보기는 체크했을 때만 영상을 받는다(1280×720 을 늘 받으면 Orin 이 바쁘다).
- 로봇 PC 가 작업하는 동안에는 촬영 시험·그리퍼 수동 조작을 하지 않는다.

```bash
./run.sh               # 패널만 — [전체 시작] 은 화면에서
./run.sh --autostart   # 패널을 띄우면서 네 부품을 바로 켬 (사람 없이 켜 둘 때)
./run.sh --no-window   # Orin 화면에 창을 띄우지 않음
```

2026-10-07 실제 장비에서 확인: 그리퍼 모터 응답(위치 118, 열림)·명령 전달,
로봇 PC [연결] → 로봇 192.168.1.200 · Orin 브리지 연결, 패널에 "로봇 PC
연결됨 (192.168.1.100)" 표시, 카메라 USB 가 빠진 상태를 "카메라를 찾지
못했습니다" 로 표시, 카메라 없이 촬영 시험 거절.

빌드 주의: Orin 의 `~/.local` 에 setuptools 64+ 가 있으면 `--symlink-install`
이 조용히 복사 설치로 바뀐다. `setup.sh` / `run.sh` 는 빌드할 때
`PYTHONNOUSERSITE=1` 로 시스템 setuptools 를 쓰게 해 두었다.

## 위치 가르치기

펜던트로 로봇을 원하는 자리에 세워 두고 로봇 PC UI 버튼을 누르면 **지금
자세·관절값을 읽어 저장**한다(로봇은 움직이지 않는다). 저장 위치는 **로봇 PC 의**
`~/.tomato_pick/taught_poses.yaml` 이며 설정 파일보다 우선한다. 작업 중에는
가르칠 수 없다(정지한 뒤에).

| UI 버튼 | 서비스 | 저장 |
| --- | --- | --- |
| 현재 위치 → 홈 자세 | `pick/teach/home_pose` | 관절 6 축 (1·10·15·17 단계) |
| 현재 위치 → 대기 자세 | `pick/teach/wait_pose` | TCP 6 축 **+ 관절 6 축** (2·16 단계는 관절값으로 movej) |
| 현재 위치 → 놓는 자리 | `pick/teach/place_pose` | TCP 6 축 + 관절 6 축 |
| ★ 홈 ↔ 대기 경유점 추가 / 마지막 지우기 / 모두 지우기 | `pick/teach/waypoint_wait_{add,undo,clear}` | 관절 6 축을 목록 끝에 |
| ★ 홈 ↔ 놓는 자리 경유점 추가 / … | `pick/teach/waypoint_place_{add,undo,clear}` | 〃 |
| 놓기 높이 [저장] | (파라미터) | 접근 높이·여는 높이 mm |
| 가르친 값 모두 지우기 | `pick/teach/clear` | 설정 파일 값으로 복귀 |

**경유점**은 홈에서 나가는 순서로 쌓는다. 홈 → 대기 길에는 그 순서로, 대기
쪽 → 홈 길에는 거꾸로 거친다(놓는 자리도 같다). 장애물을 피해 가는 자세를
펜던트로 하나씩 만들어 [경유점 추가] 를 누르면 되고 코드는 고칠 필요가 없다.
코드에서 구간을 고르는 곳은 `pick_sequence_node.py` 의 `★ 경유점` 표시다.

예전 파일(대기·놓는 자리 자세만 있는 것)도 그대로 읽는다. 다만 **홈은 새로
가르쳐야** 시작할 수 있고, 대기 자세 관절값이 없으면 로봇이 자세로 역기구학을
풀어 간다 — 대기 자세를 한 번 다시 가르치면 관절값까지 저장된다. 예전의
"하강 한계 Z" 는 없어졌다(읽을 때 버린다).

## 외부 시스템과 작업 트리거 주고받기 (로봇 PC)

작업 시작·상태는 **로봇 PC 의 ROS** 에서 주고받는다. 비전 쪽 토픽
(`/tomato/*`, `/set_pos`)은 브리지가 로봇 PC 안에 그대로 다시 만들어 주므로
아래 표는 한 대 구성과 같다.

| 방향 | 이름 | 타입 | 내용 |
| --- | --- | --- | --- |
| 외부 → 로봇 | `pick/trigger` | `std_msgs/Empty` | 한 사이클 시작 |
| 외부 → 로봇 | `pick/command` | `std_msgs/String` | `start` / `loop` / `stop` / `pause` / `resume` |
| 외부 → 로봇 | `pick/start`, `pick/start_loop`, `pick/stop`, `pick/pause`, `pick/resume` | `std_srvs/Trigger` | 같은 명령을 서비스로 (성공 여부를 돌려받는다) |
| 로봇 → 외부 | `pick/status` | `std_msgs/String` | 현재 단계·좌표·무게·오차·미설정 항목 (JSON, 5 Hz) |
| 로봇 → 외부 | `pick/result` | `std_msgs/String` | 사이클 종료 결과 (성공 여부·무게·높이) |
| 로봇 → 외부 | `pick/done` | `std_msgs/Bool` | 사이클 종료 신호 하나만 필요할 때 |
| 로봇 → 비전 | `/tomato/capture` | `std_msgs/Empty` | 촬영 트리거 (약 1 초 측정) |
| 비전 → 로봇 | `/tomato/center_positions` | `geometry_msgs/PoseArray` | 토마토 위치 **m**, 카메라 optical frame |
| 비전 → 로봇 | `/tomato/estimated_weights` | `std_msgs/Float32MultiArray` | 무게 **g** |
| 비전 → 로봇 | `/tomato/heights` | `std_msgs/Float32MultiArray` | 높이 **m** |
| 로봇 → 그리퍼 | `/set_pos` | `std_msgs/Int32` | 다이나믹셀 목표 위치 (3185 파지 / 0 열기) |

세 개의 비전 토픽은 **같은 순서**로 한 번씩 발행된다(i 번째 pose 와 i 번째
무게가 같은 토마토다). 검출이 0 개여도 빈 목록이 오므로 로봇이 매달리지
않는다.

명령줄에서 걸어 보려면:

```bash
ros2 topic pub --once /pick/trigger std_msgs/msg/Empty "{}"
ros2 topic pub --once /pick/command std_msgs/msg/String "{data: 'loop'}"
ros2 topic echo /pick/result
```

## 작업 순서

현장에서는 토마토를 **벽면 쪽으로** 따야 해서 TCP 가 아래가 아니라 벽을 본다.
그래서 픽 쪽 이동은 모두 **TCP 기준**이고, 장애물이 많은 홈 ↔ 대기 / 홈 ↔ 놓는
자리 구간은 **관절 이동 + 경유점**으로 간다.

| 단계 | 이동 | 기준 |
| --- | --- | --- |
| 1 홈 | movej | 가르친 관절값 |
| 2 대기 자세 | movej, ★경유점 순서대로 | 가르친 관절값 (없으면 자세 → 역기구학) |
| 3 촬영 → 좌표 수신 | — | 검출 목록의 첫 번째 |
| 4 카메라 중심을 토마토에 맞춤 → 다시 촬영 | movel | **TCP x·y 평면** (벽까지 거리 그대로) |
| 5 크기·무게 확인 | — | 중심에서 다시 찍은 값 |
| 6 그리퍼 파지점 정렬 | movel | **TCP 기준** 카메라→그리퍼 오프셋 |
| 7 하강 | movel | **TCP +z**, 거리는 카메라 깊이로 계산 (고정 Z 한계 없음) |
| 8 그리퍼 오므리기 | — | `/set_pos` 3185 |
| 9 상승 | movel | **TCP -z**, 6 단계 자세로 = 대기 자세와 같은 선상 |
| 10 홈 | movej, ★경유점 거꾸로 | |
| 11 놓을 자리 위로 | movej, ★경유점 순서대로 | 놓는 자리 + **접근 높이(UI 에서 지정)**, 베이스 Z |
| 12 감속 하강 | movel | 여는 높이까지 (베이스 Z, 상자 위) |
| 13 그리퍼 열기 | — | `/set_pos` 0 |
| 14 상승 | movel | 접근 높이로 |
| 15 홈 | movej, ★경유점 거꾸로 | |
| 16 대기 자세 | movej, ★경유점 순서대로 | **연속 운전이면 여기서 3 으로** |
| 17 작업 끝 — 홈 | movej, ★경유점 거꾸로 | ★ 완료 신호 자리(추후 지정) |

- **토마토가 없으면** 오류가 아니라 작업 끝이다 — 17 단계로 홈에 가서 "토마토
  N 개를 옮기고 더 없어 마쳤습니다" 로 끝난다(연속 운전의 정상 종료).
- **하강 거리** = 토마토 깊이 + 33 (플랜지→카메라 앞면) − 172 (플랜지→핑거 끝)
  + 30 (더 들어갈 양). 예) 깊이 300 mm → 191 mm. 계산한 거리가 250 mm 를
  넘으면(깊이를 잘못 쟀을 때) 들어가지 않고 멈춘다. 모두 `descend.*` 설정.
- **목표는 절대 자세로 한 번만 계산**한다. 하강 도중 일시정지했다 재개해도 두
  배로 들어가지 않는다(시뮬레이터로 확인).
- 놓기(11~14)는 상자 위에서 내려놓으므로 **베이스 Z** 기준이다.

## 현장에서 확인할 것 (벽을 보는 자세로 바꾼 뒤)

| 항목 | 왜 | 확인 방법 |
| --- | --- | --- |
| **회전 표기** `orientation_format` (기본 `rpy`) | TCP 기준 이동은 로봇이 보고한 회전으로 방향을 계산한다. 틀리면 엉뚱한 방향을 목표로 잡는다(도착 판정에 걸려 멈추긴 한다) | `python3 src/elite_robot_controller/tools/check_pose_convention.py 192.168.1.200` — **로봇을 움직이지 않고** DH 정기구학으로 비교 |
| **카메라 → TCP 축 대응** `vision.camera_axis_map` | 기본 `["-y","-x"]` 는 두산·아래 보기·베이스 기준 측정값이다 | 틀리면 두 번째 촬영에서 오차가 줄지 않아 **시퀀스가 스스로 멈춘다**("오차가 72 → 72 mm"). 그때 부호·축을 바꾼다 |
| **카메라 → 그리퍼 오프셋** `gripper.tcp_offset_mm` (TCP 기준) | 기본 `[100, 32.5, 0]` 도 두산·베이스 기준 | 중심 맞춘 뒤 6 단계에서 그리퍼가 토마토 앞에 서는지 저속으로 본다 |
| **더 들어갈 양** `descend.grasp_depth_mm` (30) | 핑거가 토마토를 감싸는 깊이 | 저속으로 집어 보며 조정 |
| 11 단계 놓을 자리 위 | 관절값이 아니라 자세로 가서 로봇이 역기구학을 푼다(지금 관절값에 가까운 해) | 자세가 이상하게 뒤집히면 놓는 쪽 경유점을 놓는 자리 가까이에 하나 더 |

안전장치: 카메라 중심 이동이 100 mm 를 넘으면 거절(`align_max_step_mm`), 두 번째
촬영 오차가 첫 오차의 70 % 를 넘으면 멈춤(`axis_check_ratio`), 하강 250 mm 초과
거절(`descend.max_travel_mm`), 경유점마다 섬(`waypoint_blend_m: 0`).

참고(README.odt): 끝단 중심에서 **카메라**는 정면 100.045 mm / 하단 8 mm,
카메라 두께 25 mm → 33 mm, 화면 중심은 오른쪽으로 17.5 mm. **그리퍼**는
하단으로 102 mm(핑거 제외) / 172 mm(핑거 전개 후 끝단).

## 로봇·Orin 없이 시험

PC 한 대에서 두 장비를 흉내 낼 수 있다. ROS 도메인을 나눠 둘이 서로를 못
보게 하고 TCP 브리지로만 잇는다 — 실제 분리 구성과 같은 조건이다.

```bash
# "Orin" 쪽 (도메인 31): 브리지 서버 + 가짜 YOLO/그리퍼
ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 ros2 run doosan_vision_grasp tomato_tcp_bridge &
ROS_DOMAIN_ID=31 ROS_LOCALHOST_ONLY=1 python3 <가짜 비전 노드> &

# 로봇 시뮬레이터
python3 src/elite_robot_controller/tools/elite_robot_simulator.py &

# "로봇 PC" 쪽 (도메인 32): 실제 run.sh 그대로
ROS_DOMAIN_ID=32 TOMATO_ROLE=pc VISION_HOST=127.0.0.1 ROBOT_IP=127.0.0.1 \
    ./run.sh modbus_port:=5502
```

2026-10-07 이 방식으로 확인한 것: 촬영 2 회·결과 2 회·그리퍼 닫기/열기가
브리지를 건너 한 사이클 완료, 작업 중 Orin 끊김 → 정지, 끊긴 동안 시작 거절,
Orin 복귀 → 자동 재접속.

2026-10-13 17 단계 시퀀스 확인(시뮬레이터 — 관절 이동은 정기구학으로 TCP 를
계산, 회전은 RPY): UI 버튼으로 홈·경유점·대기(**TCP 가 수평 = 벽을 보는 자세**)·
놓는 자리를 가르친 뒤

- 1 → 17 단계 순서대로 완료, 그리퍼 닫기·열기가 Orin 쪽에 도착
- 4 단계 중심 맞추기 72.1 mm 이동 중 TCP z 성분 0 (벽까지 거리 유지)
- 7 단계 하강 191.0 mm 모두 TCP +z, 베이스 Z 변화 0 — **하강 도중 일시정지·재개**
  했는데도 191.0 mm (두 배로 안 들어감)
- 9 단계 상승 191.0 mm TCP -z, 6 단계 자세로 오차 0 mm 복귀
- 연속 운전: 2 개 옮긴 뒤 토마토 없음 → 17 단계 홈 → "토마토 2개를 옮기고 더 없어 마쳤습니다"
- 카메라 축 대응이 틀린 상황: 두 번째 촬영 직후(하강 전) 멈추고 `camera_axis_map` 확인 안내

계산만 보는 시험은 ROS 도 필요 없다.

```bash
python3 -m pytest src/elite_robot_controller/test/test_tomato_pick.py
```
