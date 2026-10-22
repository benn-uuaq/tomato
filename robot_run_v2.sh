#!/usr/bin/env bash
# 실행 — 시퀀스 v2. ./robot_run.sh 와 옵션·화면이 모두 같고 하나만 다르다:
#
#   v1 (./robot_run.sh)    7 그리퍼 파지점은 카메라 중심을 맞춘 자세 기준. 4 단계의 재측정
#                          (4 번 트리거 /tomato/middle_capture)은 검사·깊이·무게에만 쓴다.
#   v2 (./robot_run_v2.sh) 재측정이 내는 /tomato/middle_position(카메라 중심 기준 토마토 위치)
#                          만큼 7 파지점을 보정한다 — 로봇을 한 번 더 움직이지 않고 목표에
#                          더한다. 깊이(z)·무게(/tomato/middle_weight)도 그 재측정 값.
#                          3 번 무게 추정 트리거(/tomato/capture)는 쓰지 않는다.
#
#   ./robot_run_v2.sh              # 작업 UI(http://localhost:8080) — 머리에 "시퀀스 v2" 표시
#   ./robot_run_v2.sh --connect    # 그 밖의 옵션은 robot_run.sh 와 같다
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WS/robot_run.sh" "$@" sequence_version:=2
