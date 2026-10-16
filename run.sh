#!/usr/bin/env bash
# 실행 — **한 대(Orin)에서 전부** 띄운다. 보통은 ./robot_run.sh 로 부른다.
#
#   ./run.sh                  # 로봇 제어 + 시퀀스 + 작업 UI(8080) + 비전 패널(8081)
#                             #   비전·그리퍼는 작업 UI 의 [비전 시작] 으로 켠다
#   ./run.sh --autostart      # 비전·그리퍼도 바로 켬 ([비전 시작] 을 누른 것처럼)
#   ./run.sh --connect        # UI 가 뜨자마자 기억한 로봇 주소로 [연결]
#   ./run.sh --restart        # 돌고 있던 것을 내리고 다시
#   ./run.sh --stop           # 내리기만 (./stop.sh 와 같음)
#   ./run.sh --no-window      # 화면에 UI 창을 띄우지 않음
#   ./run.sh robot_ip:=...    # 런치 인자는 그대로 넘어간다
#
# 역할 (--role, 기본 all)
#   all     한 대에서 전부 (기본)
#   vision  비전 패널만 — 카메라·YOLO·그리퍼 (vision_run.sh, 비전 시험용)
#   orin    (두 대로 나눌 때) 비전 PC — 패널 + TCP 브리지 서버
#   pc      (두 대로 나눌 때) 로봇 PC — 로봇·UI + 브리지 클라이언트
#
# 역할은 **이 명령의 --role 로만** 정한다. 터미널에 남아 있는 TOMATO_ROLE 은 보지
# 않는다 — 폴더에 들어올 때 자동으로 잡힌 예전 값(orin 등)이 끼어들지 않게.
#
# set -u 는 쓰지 않는다 — ROS 의 setup.bash 가 선언되지 않은 변수를 참조해 죽는다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RESTART=0
NO_WINDOW=0
AUTOSTART=0
CONNECT=0
ROLE=all
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --restart)   RESTART=1 ;;
        --stop)      exec bash "$WS/scripts/stop_all.sh" ;;
        --no-window) NO_WINDOW=1 ;;
        --autostart) AUTOSTART=1 ;;
        --connect)   CONNECT=1 ;;
        --role)      shift; ROLE="$1" ;;
        --role=*)    ROLE="${1#--role=}" ;;
        *)           ARGS+=("$1") ;;
    esac
    shift
done

case "$ROLE" in
    all|vision|orin|pc) ;;
    *) echo "[run] 역할은 all / vision / orin / pc 중 하나여야 합니다: $ROLE"; exit 1 ;;
esac
export TOMATO_ROLE="$ROLE"
# 예전 역할(pc)이 남긴 DDS 설정도 버린다 — env.sh 가 역할에 맞춰 다시 정한다.
unset ROS_LOCALHOST_ONLY
# shellcheck disable=SC1091
source "$WS/scripts/env.sh"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[run] 빌드 결과가 없어 먼저 빌드합니다."
    (cd "$WS" && PYTHONNOUSERSITE=1 colcon build --symlink-install) || exit 1
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi

# 이미 떠 있으면 조용히 충돌하는 대신(카메라 busy / 포트 점유로 일부만 죽는다)
# 알려 준다. **자기 역할의 노드만** 본다 — 다른 역할의 노드가 같은 장비에
# 있어도(시험할 때 등) 자기 몫과는 부딪히지 않는다.
# 한양대 쪽에서 터미널로 켜 둔 비전·그리퍼 노드는 패널이 알아보고 그대로 쓰므로
# 여기서 막지 않는다.
case "$TOMATO_ROLE" in
    vision|orin) RUN_PATTERN="vision_panel_node" ;;
    pc)   RUN_PATTERN="robot_control_node|pick_sequence_node|pick_ui_node|vision_bridge_client" ;;
    *)    RUN_PATTERN="tomato_system.launch|pick_ui_node|robot_control_node|pick_sequence_node|vision_panel_node" ;;
esac
RUNNING=$(pgrep -f "$RUN_PATTERN" 2>/dev/null | wc -l)
if [ "$RUNNING" -gt 0 ]; then
    if [ "$RESTART" = "1" ]; then
        echo "[run] 이미 떠 있는 노드를 내리고 다시 시작합니다."
        bash "$WS/scripts/stop_all.sh"
    else
        echo ""
        echo "[run] 이미 돌고 있는 노드가 있습니다 ($RUNNING 개)."
        pgrep -af "$RUN_PATTERN" | sed "s|^|      |" | cut -c1-110
        echo ""
        echo "      ./run.sh --restart   (내리고 다시 시작)"
        echo "      ./stop.sh            (내리기만)"
        echo ""
        exit 1
    fi
fi

# 유선·무선 주소를 다 보여 준다(도커 172.17.x 는 뺀다) — 어느 쪽으로 접속할지 고를 수 있게.
ALL_IPS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9.]+$' | grep -v '^172\.17\.' | tr '\n' ' ')

case "$TOMATO_ROLE" in
    vision|orin)
        # 비전 패널(웹 UI)만 띄운다. 카메라·YOLO·그리퍼(orin 이면 외부 통신도)는
        # 패널에서 켜고 끈다(--autostart 면 바로 다 켠다).
        PANEL_PORT=8081
        BRIDGE=$([ "$TOMATO_ROLE" = "orin" ] && echo true || echo false)
        echo "[run] 비전 패널 — 포트 $PANEL_PORT"
        for ip in $ALL_IPS; do echo "      http://$ip:$PANEL_PORT"; done
        export VISION_LAUNCH="${VISION_LAUNCH:-tomato_vision_d435i_v2.launch.py}"
        if [ "$AUTOSTART" = "1" ]; then
            echo "      비전($VISION_LAUNCH)·그리퍼를 켭니다 — 이미 떠 있는 것은 그대로 씁니다."
        else
            echo "      화면에서 [전체 시작] 을 누르세요."
        fi
        echo "      끝내려면 Ctrl+C (이 스크립트가 켠 것만 내린다)."
        if [ "$NO_WINDOW" = "0" ]; then
            TOMATO_UI_PORT=$PANEL_PORT setsid nohup bash "$WS/scripts/open_ui.sh" \
                > /tmp/tomato_open_ui.log 2>&1 < /dev/null &
        fi
        CONFIG="$WS/install/elite_robot_controller/share/elite_robot_controller/config/tomato_pick.yaml"
        exec ros2 run elite_robot_controller vision_panel_node --ros-args \
            -r __node:=tomato_vision_panel --params-file "$CONFIG" \
            -p autostart:=$([ "$AUTOSTART" = "1" ] && echo true || echo false) \
            -p bridge:=$BRIDGE -p bridge_port:=$BRIDGE_PORT "${ARGS[@]}"
        ;;
    pc)
        # 로봇 PC: 카메라·YOLO·그리퍼는 Orin 에 꽂혀 있다. UI 부터 띄우고,
        # 로봇·Orin 연결은 화면에서 IP 를 넣고 [연결] 을 눌러야 시작한다.
        ROLE_ARGS=(camera:=false vision:=false gripper:=false bridge:=client auto_connect:=false)
        echo "[run] 로봇 PC — UI: http://localhost:8080"
        if [ "$CONNECT" = "1" ]; then
            ROLE_ARGS+=(connect_on_start:=true)
            echo "      지난번 주소로 바로 연결합니다 (UI 의 연결 카드에서 확인)."
        else
            echo "      화면에서 로봇 IP · Orin IP 를 넣고 [연결] 을 누르세요."
        fi
        ;;
    all)
        # 한 대에서 전부. 비전·그리퍼는 패널이 한양대 런치로 띄운다(예전 camera/vision/
        # gripper 인자는 쓰지 않는다). 로봇은 화면에서 IP 를 넣고 [연결] 할 때 붙는다.
        export VISION_LAUNCH="${VISION_LAUNCH:-tomato_vision_d435i_v2.launch.py}"
        ROLE_ARGS=(camera:=false vision:=false gripper:=false bridge:=none panel:=true
                   auto_connect:=false vision_launch:=$VISION_LAUNCH)
        echo "[run] 토마토 픽앤플레이스 — 작업 UI 포트 8080, 비전 패널 8081"
        for ip in $ALL_IPS; do echo "      http://$ip:8080"; done
        if [ "$AUTOSTART" = "1" ]; then
            ROLE_ARGS+=(vision_autostart:=true)
            echo "      비전($VISION_LAUNCH)·그리퍼를 바로 켭니다."
        else
            echo "      [비전 시작] 으로 카메라·YOLO·그리퍼를 켜세요."
        fi
        if [ "$CONNECT" = "1" ]; then
            ROLE_ARGS+=(connect_on_start:=true)
            echo "      지난번 로봇 주소로 바로 연결합니다."
        else
            echo "      로봇 IP 를 넣고 [연결] 을 누르세요."
        fi
        echo "      끝내려면 Ctrl+C."
        # 화면에서 실행했다면 UI 창도 띄운다(SSH 면 열지 않는다).
        if [ "$NO_WINDOW" = "0" ]; then
            setsid nohup bash "$WS/scripts/open_ui.sh" > /tmp/tomato_open_ui.log 2>&1 < /dev/null &
        fi
        ;;
esac

exec ros2 launch elite_robot_controller tomato_system.launch.py "${ROLE_ARGS[@]}" "${ARGS[@]}"
