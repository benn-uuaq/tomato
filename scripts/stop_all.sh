#!/usr/bin/env bash
# 이 워크스페이스가 띄운 노드를 모두 내린다.
#
#   ./stop.sh          또는   bash scripts/stop_all.sh
#
# 카메라(RealSense)는 프로세스가 완전히 죽어야 장치를 놓는다. 남아 있으면
# 다음 실행이 "Device or resource busy" 로 실패하므로 확인까지 한다.

PATTERNS=(
    "vision_panel_node"          # Orin 패널 — 먼저 내리면 자기가 띄운 것을 정리한다
    "tomato_system.launch"
    "realsense2_camera_node"
    "tomato_area_d435i"
    "pick_sequence_node"
    "pick_ui_node"
    "robot_control_node"
    "gripper_dynamixel_node"
    "tomato_tcp_bridge"
    "vision_bridge_client"
    "user-data-dir=$HOME/.tomato_pick/chromium"   # UI 창
)

found=0
for pat in "${PATTERNS[@]}"; do
    pids=$(pgrep -f "$pat" 2>/dev/null)
    [ -z "$pids" ] && continue
    found=1
    echo "[stop] $pat -> $(echo "$pids" | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null
done

[ "$found" = "0" ] && { echo "[stop] 돌고 있는 노드가 없습니다."; exit 0; }

# 얌전히 안 죽으면 확실히 내린다(카메라 장치를 놓게).
sleep 3
for pat in "${PATTERNS[@]}"; do
    pids=$(pgrep -f "$pat" 2>/dev/null)
    # shellcheck disable=SC2086
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
done
sleep 1
echo "[stop] 정리 완료."
