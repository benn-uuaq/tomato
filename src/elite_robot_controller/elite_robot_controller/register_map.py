"""Modbus 레지스터 맵을 읽는다.

주소를 코드에 두지 않고 config/modbus_registers.json 한 곳에서 관리한다.
주소가 아직 정해지지 않은 항목은 address가 null이며, 이 경우 해당 기능을
거부해 잘못된 레지스터에 쓰는 일을 막는다.
"""

from __future__ import annotations

import json
import os


DEFAULT_FILENAME = "modbus_registers.json"


class RegisterEntry:
    """레지스터 한 항목. 주소가 없으면 사용할 수 없다."""

    __slots__ = ("name", "address", "count", "kind")

    def __init__(self, name: str, address: int | None, count: int, kind: str = "pose") -> None:
        self.name = name
        self.address = address
        self.count = count
        # pose는 앞 3개가 위치, 뒤 3개가 회전이다. angle은 6개 모두 회전이다.
        self.kind = kind

    @property
    def available(self) -> bool:
        """주소가 정해져 실제로 읽고 쓸 수 있는지 알려준다."""
        return self.address is not None

    def __repr__(self) -> str:  # pragma: no cover - 디버깅 편의용이다.
        return f"RegisterEntry({self.name!r}, address={self.address}, count={self.count})"


class RegisterMap:
    """읽기/쓰기 레지스터 항목과 단위 환산 계수를 담는다."""

    def __init__(self, data: dict) -> None:
        scale = data.get("scale", {})
        self.position_scale = float(scale.get("position_per_count", 0.1))
        self.rotation_scale = float(scale.get("rotation_per_count", 1.0))
        self.read = self._entries(data.get("read", {}))
        self.write = self._entries(data.get("write", {}))

    @staticmethod
    def _entries(section: dict) -> dict[str, RegisterEntry]:
        entries: dict[str, RegisterEntry] = {}
        for name, value in section.items():
            if name.startswith("_") or not isinstance(value, dict):
                continue
            address = value.get("address")
            entries[name] = RegisterEntry(
                name,
                None if address is None else int(address),
                int(value.get("count", 1)),
                str(value.get("kind", "pose")),
            )
        return entries

    def read_entry(self, name: str) -> RegisterEntry:
        return self.read.get(name) or RegisterEntry(name, None, 1)

    def write_entry(self, name: str) -> RegisterEntry:
        return self.write.get(name) or RegisterEntry(name, None, 1)

    def scales_for(self, entry: "RegisterEntry") -> list[float]:
        """항목의 성분별 환산 계수를 순서대로 돌려준다."""
        if entry.kind == "raw":
            # 이미 정수 mm/개수 등으로 저장되는 값. 소수점 하위 단위가 없다.
            return [1.0] * entry.count
        if entry.kind == "angle":
            return [self.rotation_scale] * entry.count
        # pose는 위치 3개와 회전 3개로 나뉜다.
        half = entry.count // 2
        return [self.position_scale] * half + [self.rotation_scale] * (entry.count - half)

    def available_writes(self) -> list[str]:
        """주소가 정해진 쓰기 항목 이름을 돌려준다. UI 버튼 활성화에 쓴다."""
        return sorted(name for name, entry in self.write.items() if entry.available)


def default_path() -> str:
    """설치된 패키지의 설정 파일 경로를 찾는다.

    빌드 전이나 ROS 없이 실행할 때를 위해 소스 트리 경로로 되돌아간다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("elite_robot_controller")
        candidate = os.path.join(share, "config", DEFAULT_FILENAME)
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), "config", DEFAULT_FILENAME)


def load(path: str | None = None) -> RegisterMap:
    """레지스터 맵을 읽는다. 경로를 주지 않으면 기본 위치를 사용한다."""
    with open(path or default_path(), encoding="utf-8") as handle:
        return RegisterMap(json.load(handle))
