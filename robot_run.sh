#!/usr/bin/env bash
# 실행 — 이것 하나로 전부 뜬다 (한 대에서: 로봇 제어 + 시퀀스 + 작업 UI + 비전 패널)
#
#   ./robot_run.sh              # 작업 UI(http://localhost:8080) 가 뜬다
#   ./robot_run.sh --connect    # 지난번 로봇 주소(~/.tomato_pick/connection.yaml)로 바로 연결
#   ./robot_run.sh --autostart  # 비전·그리퍼도 바로 켬 ([비전 시작] 을 누른 것처럼)
#   ./robot_run.sh --v1 | --v3  # 비전 런치를 v1 / v3 으로 (기본 v2, README.odt 권장)
#   ./robot_run.sh --restart    # 돌던 것을 내리고 다시
#
# 작업 UI 에서
#   1. 로봇 IP 넣고 [연결]
#   2. [비전 시작] -> "준비 완료" (카메라·YOLO·그리퍼, 20~60 초)
#   3. [작업 시작] — 홈 -> 대기 자세 -> 최근접 촬영 -> 카메라 중심 맞춤 -> 중앙 재측정
#      -> 파지점 정렬 -> TCP +z 하강 -> 집기(4095) -> 상승 -> 홈 -> 놓을 자리 -> 놓기(0)
#      -> 홈 -> 대기 -> 홈 순서로 한 개를 옮긴다([연속 시작] 은 토마토가 없을 때까지).
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
exec bash "$WS/run.sh" --role all "${PASS[@]}"
