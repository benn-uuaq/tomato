#!/usr/bin/env bash
# 워크스페이스 한 번 준비하기.
#
#   bash scripts/setup.sh            # 한 대(Orin)에서 전부 — 기본
#   TOMATO_ROLE=pc bash scripts/setup.sh    # (두 대로 나눌 때) 로봇 PC
#
# 역할 (scripts/env.sh 와 같은 규칙)
#   all    : 기본. 로봇·UI + 카메라·YOLO·그리퍼 — 가상환경과 YOLO 의존을 깐다
#   vision : 비전 쪽만 (all 과 같이 준비)
#   orin   : (두 대로 나눌 때) 비전 PC (all 과 같이 준비)
#   pc     : (두 대로 나눌 때) 로봇 PC — **가상환경도 torch 도 깔지 않는다**
#
# 하는 일
#   0. 다른 장비에서 빌드된 install/ 정리
#   1. ROS 노드용 의존 → 시스템 파이썬 (pip --user)
#   2. (orin/all) 가상환경 + YOLO 의존
#   3. colcon build
#   4. 폴더에 들어오면 환경이 자동으로 잡히게 (direnv 가 있으면 direnv, 없으면 ~/.bashrc 훅)
#   5. 주변 확인 (로봇 도달, 카메라·그리퍼·모델 / pc: 로봇·비전 PC 도달)
#
# 이미 되어 있는 것은 건너뛰므로 몇 번 돌려도 안전하다.
# set -u 는 쓰지 않는다 — ROS 의 setup.bash 가 선언되지 않은 변수를 참조해 죽는다.

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$WS/venv"

is_jetson() { [ -f /etc/nv_tegra_release ] || grep -qi tegra /proc/version 2>/dev/null; }
TOMATO_ROLE="${TOMATO_ROLE:-all}"
case "$TOMATO_ROLE" in
    all|vision|orin|pc) ;;
    *) echo "TOMATO_ROLE 은 all / vision / orin / pc 중 하나여야 합니다: $TOMATO_ROLE"; exit 1 ;;
esac

echo "=== 토마토 수확 워크스페이스 준비 ==="
echo "    위치: $WS"
echo "    역할: $TOMATO_ROLE"

# --------------------------------------------------- 0. 남의 장비 빌드 결과 정리
# colcon 의 install/ 안에는 **빌드한 장비의 절대 경로**가 박혀 있다. 폴더째
# 복사해 왔다면 그 경로가 여기와 달라서, source 는 되는데 노드는 못 찾는
# 이상한 상태가 된다(--symlink-install 이면 심볼릭 링크까지 깨진다).
if [ -f "$WS/install/setup.sh" ] && ! grep -q "$WS" "$WS/install/setup.sh" 2>/dev/null; then
    echo "[0/5] 다른 장비에서 빌드된 install/ 을 발견했습니다 — 지우고 새로 빌드합니다."
    rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

# ------------------------------------------------------ 1. ROS 노드용 의존
# colcon 이 만든 실행 파일의 shebang 이 /usr/bin/python3 라서, 가상환경에만
# 깔면 노드가 못 찾는다(ModuleNotFoundError: pyModbusTCP).
echo "[1/5] ROS 노드용 의존 (시스템 파이썬)"
/usr/bin/python3 -m pip install --user -q -r "$WS/requirements/ros.txt" || {
    echo "    시스템 파이썬 의존 설치 실패 — 위 로그를 확인하세요."
    exit 1
}

# ------------------------------------------------ 2. 가상환경 + YOLO 의존
if [ "$TOMATO_ROLE" = "pc" ]; then
    echo "[2/5] 로봇 PC — 가상환경·YOLO 의존은 필요 없어 건너뜁니다."
else
    if [ ! -x "$VENV/bin/python3" ]; then
        echo "[2/5] 가상환경 생성 (--system-site-packages)"
        # rclpy, cv_bridge 는 시스템(ROS) 것을 그대로 봐야 한다.
        python3 -m venv --system-site-packages "$VENV" || {
            echo "    venv 생성 실패 — sudo apt install python3-venv"
            exit 1
        }
    else
        echo "[2/5] 가상환경 이미 있음"
    fi
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
    python3 -m pip install --upgrade -q pip setuptools wheel
    STAMP="$VENV/.tomato_deps_done"
    if [ ! -f "$STAMP" ]; then
        if is_jetson; then
            echo "      Jetson — torch 는 건드리지 않고 나머지만 설치합니다."
            python3 -m pip install -r "$WS/requirements/jetson.txt" || exit 1
            # ultralytics 가 torch 를 끌어오지 않게 --no-deps 로 넣는다.
            python3 -m pip install --no-deps ultralytics==8.4.67 || exit 1
            if ! python3 -c 'import torch' 2>/dev/null; then
                echo ""
                echo "    ※ torch 가 아직 없습니다. JetPack 버전에 맞는 NVIDIA 휠을 설치하세요:"
                echo "      https://developer.download.nvidia.com/compute/redist/jp/"
                echo ""
            fi
        else
            echo "      일반 PC — torch 포함 설치 (수 GB)"
            python3 -m pip install -r "$WS/requirements/desktop.txt" || exit 1
        fi
        touch "$STAMP"
    else
        echo "      YOLO 의존 이미 설치됨 (다시 하려면 $STAMP 삭제)"
    fi
    deactivate 2>/dev/null || true
fi

# --------------------------------------------------------------- 3. 빌드
echo "[3/5] colcon build"
if [ -f /opt/ros/humble/setup.bash ]; then
    # 빌드는 시스템 파이썬 기준으로 돈다. 가상환경이 켜진 채로 돌리면 꼬인다.
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # PYTHONNOUSERSITE=1: ~/.local 에 새 setuptools(64+) 가 깔려 있으면
    # --symlink-install 이 조용히 복사 설치로 바뀐다(소스를 고쳐도 다시 빌드
    # 하기 전엔 반영 안 됨). 빌드할 때만 시스템 setuptools 를 쓰게 한다.
    (cd "$WS" && PYTHONNOUSERSITE=1 colcon build --symlink-install 2>&1 | tail -3) || {
        echo "    빌드 실패 — 위 로그를 확인하세요."
        exit 1
    }
else
    echo "    /opt/ros/humble 이 없어 빌드를 건너뜁니다."
fi

# ------------------------------------------------------- 4. 자동 환경 설정
# direnv 가 있으면 그걸 쓴다(.envrc). 둘 다 걸면 env.sh 가 두 번 불린다.
if command -v direnv >/dev/null 2>&1; then
    echo "[4/5] direnv 로 자동 환경 설정 (.envrc 승인)"
    (cd "$WS" && direnv allow . )
else
    HOOK_MARK="# >>> tomato_ws auto env >>>"
    if ! grep -q "$HOOK_MARK" "$HOME/.bashrc" 2>/dev/null; then
        echo "[4/5] ~/.bashrc 에 자동 환경 설정 추가"
        cat >> "$HOME/.bashrc" <<'HOOK'

# >>> tomato_ws auto env >>>
# 워크스페이스 폴더(.tomato_ws 표시 파일이 있는 곳)에 들어가면 ROS·가상환경을
# 자동으로 잡는다. 빠져나오면 가상환경만 해제한다.
_tomato_auto_env() {
    local dir="$PWD"
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/.tomato_ws" ]; then
            if [ "${TOMATO_WS:-}" != "$dir" ]; then
                source "$dir/scripts/env.sh"
                echo "[tomato_ws] 환경 준비 완료 ($TOMATO_ROLE) — 실행: ./run.sh"
            fi
            return
        fi
        dir="$(dirname "$dir")"
    done
    if [ -n "${TOMATO_WS:-}" ]; then
        [ -n "${VIRTUAL_ENV:-}" ] && deactivate 2>/dev/null
        unset TOMATO_WS
    fi
}
PROMPT_COMMAND="_tomato_auto_env${PROMPT_COMMAND:+;$PROMPT_COMMAND}"
# <<< tomato_ws auto env <<<
HOOK
    else
        echo "[4/5] ~/.bashrc 훅 이미 있음"
    fi
fi

# ------------------------------------------------------------ 5. 주변 확인
echo "[5/5] 주변 확인"
# shellcheck disable=SC1091
TOMATO_NO_AUTOSETUP=1 source "$WS/scripts/env.sh" >/dev/null 2>&1
if [ "$TOMATO_ROLE" = "pc" ]; then
    if ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
        echo "    로봇 $ROBOT_IP — 응답 있음"
    else
        echo "    ※ 로봇 $ROBOT_IP 에 닿지 않습니다. 이 PC(Windows)의 유선 랜에"
        echo "      같은 대역 주소(예: 192.168.1.20/24)를 주고 랜선을 확인하세요."
    fi
    if timeout 3 bash -c "cat < /dev/null > /dev/tcp/$VISION_HOST/$BRIDGE_PORT" 2>/dev/null; then
        echo "    비전 PC $VISION_HOST:$BRIDGE_PORT — 브리지 응답 있음"
    else
        echo "    ※ 비전 PC $VISION_HOST:$BRIDGE_PORT 브리지가 아직 안 보입니다"
        echo "      (Orin 에서 ./run.sh 를 띄운 뒤에 열립니다)."
    fi
else
    if [ "$TOMATO_ROLE" = "all" ]; then
        if ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
            echo "    로봇 $ROBOT_IP — 응답 있음"
        else
            echo "    ※ 로봇 $ROBOT_IP 에 닿지 않습니다 — 랜선과 유선 주소(192.168.1.x/24)를 확인하세요."
        fi
    fi
    if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
        echo "    ※ realsense2_camera 가 없습니다: sudo apt install ros-humble-realsense2-camera"
    fi
    if ! id -nG "$USER" | grep -qw dialout; then
        echo "    ※ 그리퍼 시리얼 권한이 없습니다: sudo usermod -aG dialout $USER (다시 로그인)"
    fi
    if [ ! -f "$WS/models/tomato_v1.pt" ]; then
        echo "    ※ models/tomato_v1.pt 가 없습니다 — 비전 노드가 뜨지 않습니다."
    fi
fi

echo ""
echo "=== 준비 끝 ($TOMATO_ROLE) ==="
echo "    cd $WS && ./run.sh"
