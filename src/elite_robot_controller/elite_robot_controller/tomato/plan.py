"""시퀀스가 가는 목표 자세를 계산한다 — ROS 없이 시험할 수 있게 떼어 둔다.

벽을 보는 자세(TCP 가 수평)에서도 맞게 동작하도록 **픽 쪽 이동은 모두 TCP
기준**이다.

    중심 맞추기  : TCP 의 x·y 평면에서 카메라 오차만큼
    파지점 정렬  : TCP 의 x·y(·z) 로 카메라 -> 그리퍼 오프셋만큼
    하강        : TCP +z 로 (토마토 쪽)
    상승        : 파지점 정렬 자세로 되돌아감 = 대기 자세와 같은 선상

놓기 쪽(상자)은 위에서 내려놓으므로 **베이스 Z** 다.

목표는 모두 **절대 자세**로 한 번만 계산해 둔다. 일시 정지 뒤 재개하면 같은
목표를 다시 보내므로, "지금 자리에서 얼마만큼" 식으로 두 번 더해지는 일이
없다(하강 도중 멈췄다 재개해도 두 배로 들어가지 않는다).
"""

from __future__ import annotations

import math

from . import geometry


def center_pose(trigger_pose, delta_xy, frame: str = "tool"):
    """찍은 자리에서 카메라 중심이 토마토에 오도록 옮긴 자세.

    delta_xy 는 [dx, dy] (m) — `vision_interface.camera_delta` 가 카메라 축
    대응(camera_axis_map) 을 적용해 만든 값이다.
    """
    dx, dy = float(delta_xy[0]), float(delta_xy[1])
    if frame == "tool":
        return geometry.transform_in_tool(trigger_pose, [dx, dy, 0.0])
    if frame == "base":
        out = list(trigger_pose)
        out[0] += dx
        out[1] += dy
        return out
    raise ValueError(f"align_frame 은 tool 또는 base 여야 합니다: {frame!r}")


def grip_pose(centered, offset_mm, frame: str = "tool"):
    """카메라 중심 자세에서 그리퍼 중심이 토마토에 오도록 옮긴 자세."""
    offset = [float(v) / 1000.0 for v in offset_mm]
    if frame == "tool":
        return geometry.transform_in_tool(centered, offset)
    if frame == "base":
        out = list(centered)
        out[0] += offset[0]
        out[1] += offset[1]
        out[2] += offset[2] if len(offset) > 2 else 0.0
        return out
    raise ValueError(f"offset_frame 은 tool 또는 base 여야 합니다: {frame!r}")


def descend_pose(grip, travel_m: float):
    """파지 자세에서 TCP +z 로 travel 만큼 들어간 자세."""
    return geometry.transform_in_tool(grip, [0.0, 0.0, float(travel_m)])


def above(pose, height_m: float):
    """베이스 +Z 로 height 만큼 위(놓는 자리 접근용)."""
    out = list(pose)
    out[2] += float(height_m)
    return out


def residual_mm(delta_xy) -> float:
    """카메라 중심에서 토마토까지 [mm]."""
    return 1000.0 * math.hypot(float(delta_xy[0]), float(delta_xy[1]))
