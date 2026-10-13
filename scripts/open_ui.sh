#!/usr/bin/env bash
# Orin 화면에 조작 UI 창을 띄운다.
#
# `run.sh` 가 배경으로 이걸 부른다. UI 서버가 응답할 때까지 기다렸다가
# 브라우저를 여는데, 카메라와 YOLO 가 올라오는 데 20 초쯤 걸리기 때문이다.
#
#   bash scripts/open_ui.sh            # 화면이 있으면 연다
#   TOMATO_UI_KIOSK=1 ...              # 전체화면(키오스크)으로
#
# **SSH 로 실행할 때는 열지 않는다.** 화면이 없는 세션에서 창을 띄우려 하면
# 실패하거나, 옆에서 보고 있지도 않은 Orin 모니터에 창이 튀어나온다.

PORT="${TOMATO_UI_PORT:-8080}"
URL="http://localhost:$PORT"
PROFILE="$HOME/.tomato_pick/chromium"

# --- 화면 찾기 ---------------------------------------------------------
if [ -z "$DISPLAY" ]; then
    if [ "${TOMATO_UI_FORCE:-0}" = "1" ] && [ -e /tmp/.X11-unix/X0 ]; then
        export DISPLAY=:0     # systemd 등에서 일부러 띄울 때
    else
        echo "[ui] 화면이 없어 창은 열지 않습니다 ($URL 로 접속하세요)."
        exit 0
    fi
fi

BROWSER=$(command -v chromium-browser || command -v chromium || command -v firefox)
if [ -z "$BROWSER" ]; then
    echo "[ui] 브라우저를 찾지 못했습니다. $URL 로 직접 접속하세요."
    exit 1
fi

# --- 이미 열려 있으면 두 번 열지 않는다 --------------------------------
# 명령줄만 보고 판단하면 안 된다. user-data-dir 문자열이 들어간 다른
# 명령줄(이 검사를 부른 셸 등)까지 걸리고, 반대로 크로미움은 실행 파일
# 경로가 /usr/bin 이 아니라 /usr/lib/... 으로 바뀌어 있어 경로로 묶어도
# 빗나간다. 그래서 **찾은 PID 가 정말 브라우저인지** 를 확인한다.
window_open() {
    local pid comm
    for pid in $(pgrep -f "user-data-dir=$PROFILE" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        comm=$(cat "/proc/$pid/comm" 2>/dev/null)
        case "$comm" in
            chrom*|firefox*) return 0 ;;
        esac
    done
    return 1
}

if window_open; then
    echo "[ui] 창이 이미 열려 있습니다."
    exit 0
fi

# --- UI 서버가 답할 때까지 기다린다 ------------------------------------
echo "[ui] UI 준비를 기다립니다..."
for _ in $(seq 1 90); do
    if curl -s -o /dev/null --max-time 2 "$URL/status"; then
        break
    fi
    sleep 1
done
if ! curl -s -o /dev/null --max-time 2 "$URL/status"; then
    echo "[ui] UI 가 응답하지 않아 창을 열지 않습니다."
    exit 1
fi

MODE="--app=$URL --start-maximized"
[ "${TOMATO_UI_KIOSK:-0}" = "1" ] && MODE="--kiosk $URL"

echo "[ui] 창을 엽니다: $URL"
# 별도 프로필을 쓴다 — 쓰던 크로미움 창이 있어도 서로 방해하지 않는다.
# shellcheck disable=SC2086
setsid nohup "$BROWSER" --user-data-dir="$PROFILE" \
    --no-first-run --no-default-browser-check --disable-translate \
    $MODE > /tmp/tomato_ui_window.log 2>&1 < /dev/null &
