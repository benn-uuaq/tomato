#!/usr/bin/env bash
# 비전 PC(Orin) 실행 — 카메라 + YOLO(한양대 통합 비전 노드) + 그리퍼 + 외부 통신 브리지
#
#   ./vision_run.sh              # 빠진 것만 켠다. 한양대 쪽에서 이미 켠 노드는 그대로 쓴다
#   ./vision_run.sh --v1 | --v3  # 비전 런치를 v1 / v3 으로 (기본 v2, README.odt 권장)
#   ./vision_run.sh --no-window  # Orin 화면에 패널 창을 띄우지 않음
#   ./vision_run.sh --restart    # 이 워크스페이스가 띄운 것을 내리고 다시
#
# 준비가 끝나면 터미널에 "[상태] 준비 완료 — 로봇 PC(robot_run.sh) 의 [연결] 을
# 기다리는 중" 이 찍힌다. 그다음 로봇 PC 에서 ./robot_run.sh.
# 브라우저 패널(http://<Orin IP>:8081)에서 상태 확인·촬영 시험·그리퍼 수동 조작도 된다.
#
# 한양대 방식으로 직접 켜도 된다(다른 터미널):
#   cd ~/doosan_manipulator && source install/setup.bash
#   ros2 launch doosan_vision_grasp tomato_vision_d435i_v2.launch.py
# 그 상태에서 ./vision_run.sh 를 하면 비전은 그대로 두고 그리퍼·브리지만 켠다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PASS=()
for arg in "$@"; do
    case "$arg" in
        --v1) export VISION_LAUNCH=tomato_vision_d435i.launch.py ;;
        --v2) export VISION_LAUNCH=tomato_vision_d435i_v2.launch.py ;;
        --v3) export VISION_LAUNCH=tomato_vision_d435i_v3.launch.py ;;
        *)    PASS+=("$arg") ;;
    esac
done
exec bash "$WS/run.sh" --role orin --autostart "${PASS[@]}"
