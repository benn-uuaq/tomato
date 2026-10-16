#!/usr/bin/env bash
# 워크스페이스 환경을 한 번에 잡는다. `source scripts/env.sh` 로 쓰고,
# direnv(.envrc) 나 ~/.bashrc 훅도 이 파일을 부른다.
#
# **장비마다 역할이 다르다** (TOMATO_ROLE)
#   orin : 비전 PC — 카메라·YOLO·그리퍼 + TCP 브리지 서버
#   pc   : 로봇 PC — 로봇·시퀀스·UI + TCP 브리지 클라이언트 (WSL 포함)
#   all  : 한 대에서 전부 (브리지 없음)
# 정하지 않으면 Jetson 이면 orin, 아니면 pc 로 본다. 바꾸려면:
#   TOMATO_ROLE=all ./run.sh

# 이 파일의 위치로 워크스페이스를 찾는다(어디서 source 해도 맞게).
_TOMATO_ENV_SRC="${BASH_SOURCE[0]:-$0}"
export TOMATO_WS="$(cd "$(dirname "$_TOMATO_ENV_SRC")/.." && pwd)"

# ---------------------------------------------------------------- 역할
if [ -z "${TOMATO_ROLE:-}" ]; then
    if [ -f /etc/nv_tegra_release ] || grep -qi tegra /proc/version 2>/dev/null; then
        TOMATO_ROLE=orin
    else
        TOMATO_ROLE=pc
    fi
fi
export TOMATO_ROLE

# ---------------------------------------------------------------- 주소
# 여기 세 줄만 고치면 런치 기본값이 따라간다.
export ROBOT_IP="${ROBOT_IP:-192.168.1.200}"     # Elite CS616 컨트롤러
export VISION_HOST="${VISION_HOST:-192.168.1.10}" # 비전 PC(Orin) — 로봇 PC 가 여기 접속한다
export BRIDGE_PORT="${BRIDGE_PORT:-9000}"         # 비전 팀 tomato_tcp_bridge 기본 포트

# ROS(DDS) 설정 — 장비마다 다르다.
#   Orin : **새 터미널과 같은 기본값**(도메인 0, 모든 인터페이스). 한양대 쪽이
#          자기 터미널에서 `ros2 launch doosan_vision_grasp ...` 로 켠 비전 노드나
#          dyn_test 가 이 워크스페이스의 패널·브리지에 보여야 하기 때문이다.
#   PC   : 다른 장비의 ROS 를 보지 않게 막는다. 둘을 잇는 건 TCP 브리지 하나뿐이다
#          — 같은 토픽 이름이 양쪽에 있어 서로 보이면 명령이 메아리처럼 되돌아온다.
#          (WSL2 의 NAT 네트워크는 어차피 DDS 탐색 패킷이 바깥과 오가지 못한다.)
if [ "$TOMATO_ROLE" = "orin" ]; then
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
else
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
fi

export TOMATO_VENV="$TOMATO_WS/venv"
export TOMATO_VENV_PY="$TOMATO_VENV/bin/python3"
export TOMATO_MODEL="${TOMATO_MODEL:-$TOMATO_WS/models/tomato_v1.pt}"       # 무게 추정 모델
export TOMATO_PICK_MODEL="${TOMATO_PICK_MODEL:-$TOMATO_WS/models/best.pt}"     # 위치 인식 모델 (2026-10)

# ---------------------------------------------------------------- ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[env] /opt/ros/humble 이 없습니다 — ROS 2 Humble 을 먼저 설치하세요."
fi

# ---------------------------------------------------------------- 가상환경
# YOLO 를 돌리는 장비에서만 필요하다. 로봇 PC 에서는 만들지도 켜지도 않는다
# — torch 를 받지 않게 하기 위해서다.
if [ "$TOMATO_ROLE" != "pc" ]; then
    if [ ! -x "$TOMATO_VENV_PY" ]; then
        if [ "${TOMATO_NO_AUTOSETUP:-0}" = "1" ]; then
            echo "[env] 가상환경이 없습니다. scripts/setup.sh 를 실행하세요."
        else
            echo "[env] 가상환경이 없어 처음 한 번 준비합니다 (scripts/setup.sh)..."
            bash "$TOMATO_WS/scripts/setup.sh" || echo "[env] 준비 중 문제가 있었습니다 — 위 로그를 보세요."
        fi
    fi
    if [ -f "$TOMATO_VENV/bin/activate" ]; then
        source "$TOMATO_VENV/bin/activate"
    fi
fi

# ---------------------------------------------------------------- 빌드 결과
if [ -f "$TOMATO_WS/install/setup.bash" ]; then
    source "$TOMATO_WS/install/setup.bash"
fi
