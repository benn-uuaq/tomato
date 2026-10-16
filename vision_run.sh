#!/usr/bin/env bash
# 비전만 실행 — 카메라 + YOLO(한양대 통합 비전 노드) + 그리퍼. 로봇은 띄우지 않는다.
# 로봇 작업은 ./robot_run.sh 하나로 하면 되고(비전도 같이 뜬다), 이건 비전만 시험할 때 쓴다.
#
#   ./vision_run.sh              # 빠진 것만 켠다. 한양대 쪽에서 이미 켠 노드는 그대로 쓴다
#   ./vision_run.sh --v1 | --v3  # 비전 런치를 v1 / v3 으로 (기본 v2, README.odt 권장)
#   ./vision_run.sh --no-window  # 화면에 패널 창을 띄우지 않음
#   ./vision_run.sh --restart    # 이 워크스페이스가 띄운 것을 내리고 다시
#
# 브라우저 패널(http://<Orin IP>:8081)에서 상태 확인·촬영 시험·그리퍼 수동 조작을 한다.
#
# 한양대 방식으로 직접 켜도 된다(다른 터미널):
#   cd ~/doosan_manipulator && source install/setup.bash
#   ros2 launch doosan_vision_grasp tomato_vision_d435i_v2.launch.py
# 그 상태에서 ./vision_run.sh(또는 ./robot_run.sh 의 [비전 시작])를 하면 비전은 그대로 두고
# 그리퍼만 켠다.
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
exec bash "$WS/run.sh" --role vision --autostart "${PASS[@]}"
