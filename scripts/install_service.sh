#!/usr/bin/env bash
# 부팅하면 자동으로 뜨게 한다 (선택 사항).
#
#   bash scripts/install_service.sh          # 설치 + 지금 시작
#   sudo systemctl status  tomato-pick       # 상태
#   sudo journalctl -u tomato-pick -f        # 로그 보기
#   sudo systemctl stop    tomato-pick       # 내리기
#   sudo systemctl disable tomato-pick       # 자동 시작 끄기
#
# 자동 시작을 켜두면 사람이 로그인하지 않아도 카메라·로봇·UI 가 올라온다.
# 손으로 ./run.sh 를 쓰려면 먼저 이 서비스를 멈춰야 한다(포트·카메라 충돌).

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="$(id -un)"
UNIT=/etc/systemd/system/tomato-pick.service

echo "[service] $UNIT 생성 (작업 폴더: $WS, 사용자: $USER_NAME)"
sudo tee "$UNIT" > /dev/null <<UNITFILE
[Unit]
Description=토마토 수확 시스템 (카메라 + 비전 + 로봇 + UI)
# 네트워크(로봇 LAN)와 USB 장치가 준비된 뒤에 뜬다.
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$WS
# run.sh 가 ROS·가상환경·환경변수를 스스로 잡는다.
ExecStart=/bin/bash $WS/run.sh
ExecStop=/bin/bash $WS/scripts/stop_all.sh
Restart=on-failure
RestartSec=5
# 카메라가 USB 로 다시 잡히는 데 시간이 걸릴 수 있다.
TimeoutStartSec=120

[Install]
WantedBy=multi-user.target
UNITFILE

sudo systemctl daemon-reload
sudo systemctl enable tomato-pick
echo "[service] 설치 완료. 지금 시작하려면: sudo systemctl start tomato-pick"
