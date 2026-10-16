#!/usr/bin/env bash
# 로봇 PC(WSL) 실행 — 로봇 제어 + 픽앤플레이스 시퀀스 + 작업 UI + 비전 브리지 클라이언트
#
#   ./robot_run.sh             # UI(http://localhost:8080) 를 띄운다. 화면에서 IP 넣고 [연결]
#   ./robot_run.sh --connect   # 지난번 주소(~/.tomato_pick/connection.yaml)로 바로 연결
#   ./robot_run.sh --restart   # 이 PC 에서 돌던 것을 내리고 다시
#
# Orin 에서 ./vision_run.sh 가 "준비 완료" 를 찍은 뒤 [작업 시작] 을 누르면
#   홈 -> 대기 자세 -> 최근접 촬영 -> 카메라 중심 맞춤 -> 중앙 재측정 -> 파지점 정렬
#   -> TCP +z 하강 -> 집기(4095) -> 상승 -> 홈 -> 놓을 자리 -> 놓기(0) -> 홈 -> 대기 -> 홈
# 순서로 한 개를 옮긴다([연속 시작] 은 토마토가 없을 때까지).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WS/run.sh" --role pc "$@"
