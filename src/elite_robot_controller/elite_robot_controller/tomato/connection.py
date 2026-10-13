"""UI 에서 넣은 로봇·Orin 주소를 검사하고 기억해 두는 곳.

다음에 UI 를 열면 마지막에 쓴 주소가 미리 채워져 있게 한다. 사람이 읽고
고칠 수 있게 yaml 한 장으로 둔다.

    ~/.tomato_pick/connection.yaml
"""

from __future__ import annotations

import ipaddress
import os
import re

import yaml

DEFAULT_PATH = "~/.tomato_pick/connection.yaml"
FIELDS = ("robot_ip", "vision_host")

#: 호스트 이름(예: nano-desktop.local) 으로도 넣을 수 있게 한다.
_HOSTNAME = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9\-]{0,61}[A-Za-z0-9])?"
                       r"(?:\.[A-Za-z0-9](?:[A-Za-z0-9\-]{0,61}[A-Za-z0-9])?)*$")


def validate_host(text) -> tuple:
    """(맞는지, 다듬은 값, 사유) 를 돌려준다.

    숫자와 점만 있으면 IPv4 로 보고 엄격히 검사한다 — "192.168.1.2000" 같은
    오타가 호스트 이름으로 통과해 엉뚱한 곳에 접속을 시도하지 않게.
    """
    value = str(text or "").strip()
    if not value:
        return False, value, "주소가 비어 있습니다."
    if re.fullmatch(r"[0-9.]+", value):
        try:
            ipaddress.IPv4Address(value)
        except ValueError:
            return False, value, f"IPv4 주소 형식이 아닙니다: {value}"
        return True, value, ""
    if len(value) > 253 or not _HOSTNAME.match(value):
        return False, value, f"주소 형식이 아닙니다: {value}"
    return True, value, ""


def resolve(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path or DEFAULT_PATH))


def load(path: str) -> dict:
    """기억해 둔 주소를 읽는다. 없거나 깨졌으면 빈 dict."""
    full = resolve(path)
    if not os.path.isfile(full):
        return {}
    try:
        with open(full, encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception:
        return {}
    return {key: str(data[key]) for key in FIELDS if data.get(key)}


def save(path: str, values: dict) -> str:
    full = resolve(path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    body = {key: values[key] for key in FIELDS if values.get(key)}
    with open(full, "w", encoding="utf-8") as handle:
        handle.write("# UI 에서 마지막으로 연결한 주소. 다음에 UI 를 열면 미리 채워진다.\n")
        yaml.safe_dump(body, handle, allow_unicode=True, sort_keys=True,
                       default_flow_style=False)
    return full
