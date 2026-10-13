"""로봇을 실제로 움직이는 명령을 만든다.

**소켓은 robot_control_node 가 혼자 들고 있는다.** 여기서는 30001 로 보낼
스크립트 문자열만 만들어 `robot/command/script` 토픽으로 넘긴다. 같은
포트에 두 프로세스가 따로 붙으면 명령이 섞이기 때문이다.

pose 단위는 Elite 스크립트와 같다 — 위치 m, 회전은 rotvec [rad].

자세 표기
--------
Elite 스크립트에서 pose 는 **그냥 리스트** `[x, y, z, rx, ry, rz]` 다.
URScript 의 `p[...]` 리터럴을 쓰면 안 된다 — `p` 가 내장 함수라서
컨트롤러가 ``type_error_arg_not_subscriptable: builtin_function_or_method``
로 거절한다(스크립트를 버리고 아무 동작도 하지 않는다. 2026-09-21 CS616
실장비에서 확인).

`movej` 는 **관절값**을 받는다. 자세로 관절 보간 이동을 하려면
`get_inverse_kin()` 으로 관절값을 먼저 구한다.
"""

from __future__ import annotations


def _fmt(values) -> str:
    return "[" + ", ".join(f"{float(v):.6f}" for v in values) + "]"


def movel(pose, accel: float, speed: float) -> str:
    """직선 이동. pose 는 [x, y, z, rx, ry, rz] (m, rad)."""
    return f"movel({_fmt(pose)}, a={accel:.4f}, v={speed:.4f})"


def movej_pose(pose, accel: float, speed: float) -> str:
    """관절 보간으로 pose 까지 간다. 크게 움직일 때 쓴다.

    movej 는 관절값을 받으므로 역기구학을 로봇 쪽에서 풀게 한다.
    """
    return f"movej(get_inverse_kin({_fmt(pose)}), a={accel:.4f}, v={speed:.4f})"


def movej_joints(joints, accel: float, speed: float) -> str:
    """관절값으로 바로 이동한다 [rad] 6 개."""
    return f"movej({_fmt(joints)}, a={accel:.4f}, v={speed:.4f})"


def with_z(pose, z: float) -> list:
    """자세는 그대로 두고 Z 만 바꾼 pose 를 만든다."""
    out = list(pose)
    out[2] = z
    return out


def with_xy(pose, x: float, y: float) -> list:
    """자세와 Z 는 그대로 두고 X, Y 만 바꾼 pose 를 만든다."""
    out = list(pose)
    out[0] = x
    out[1] = y
    return out


def joint_path_program(name: str, waypoints, target_joints=None, target_pose=None,
                       accel: float = 1.0, speed: float = 0.5, blend: float = 0.0) -> str:
    """경유점을 차례로 거쳐 목표까지 가는 관절 이동 프로그램 하나를 만든다.

    ★ 경유점 — 홈 <-> 대기 / 홈 <-> 놓는 자리 사이에 장애물이 많아 직접 가면
    부딪힐 때, 펜던트로 피해 가는 자세를 가르쳐 `waypoints` 에 넣는다(UI 의
    [경유점 추가]). 가는 길에는 순서대로, 돌아오는 길에는 거꾸로 넘겨 받는다.

    목표는 관절값(`target_joints`) 이 우선이고, 없으면 자세(`target_pose`) 를
    로봇이 역기구학으로 풀어 간다. 여러 줄이라 `def ... end` 로 감싸야 한다
    (30001 은 최상위 if/여러 줄을 한 줄씩 따로 읽는다).

    blend > 0 이면 경유점에서 멈추지 않고 그 반경 안에서 이어 돈다.
    """
    lines = [f"def {name}():"]
    tail = f", a={accel:.4f}, v={speed:.4f}"
    for point in waypoints:
        radius = f", r={blend:.4f}" if blend > 0 else ""
        lines.append(f"  movej({_fmt(point)}{tail}{radius})")
    if target_joints is not None:
        lines.append(f"  movej({_fmt(target_joints)}{tail})")
    elif target_pose is not None:
        lines.append(f"  movej(get_inverse_kin({_fmt(target_pose)}){tail})")
    lines.append("end")
    return "\n".join(lines)
