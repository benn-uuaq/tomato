"""자세 계산에 쓰는 최소한의 기하 함수.

numpy 없이 순수 파이썬으로 둔다. Orin nano 의 ROS 환경에 numpy 가 늘
있긴 하지만, 이 모듈만은 의존을 하나도 늘리지 않는 편이 이식이 쉽다.

자세 표현은 **Elite 스크립트의 pose 와 같다** — `[x, y, z, rx, ry, rz]`,
앞 3 개는 미터, 뒤 3 개는 회전 [rad]. 사람이 고치는 설정 파일에서는 mm 와
deg 를 쓰고, 여기 들어오기 전에 `pick_config` 가 환산한다.

회전 표기는 **RPY** 로 본다(기본). 근거: CS616 이 아래를 보는 자세를
`(-3.1416, 0, -0.7854)` 로 보고했는데, 회전 벡터라면 크기가 3.238 로 π 를
넘어 나올 수 없는 값이다 — RPY(roll -180°, yaw -45°) 로 읽어야 말이 된다.
확정은 `tools/check_pose_convention.py` (로봇을 움직이지 않고 DH 정기구학으로
비교) 로 한다. 회전 벡터로 바꿔야 하면 `set_orientation_format("rotvec")`.

TCP 기준 이동(벽을 보는 자세에서 TCP +z 로 하강 등)은 이 표기로 회전 행렬을
만들어 계산하므로, 표기가 틀리면 엉뚱한 방향으로 간다. 그래서 시퀀스는 계산한
목표에 **실제로 도착했는지**를 확인하고, 아니면 멈춘다.
"""

from __future__ import annotations

import math

Vec3 = list
Mat3 = list
Pose = list

#: pose 의 회전 3 개를 어떻게 읽을지: "rpy" (기본) | "rotvec".
ORIENTATION = "rpy"


def set_orientation_format(fmt: str) -> None:
    global ORIENTATION
    fmt = str(fmt).strip().lower()
    if fmt not in ("rpy", "rotvec"):
        raise ValueError(f"orientation_format 은 rpy 또는 rotvec 이어야 합니다: {fmt!r}")
    ORIENTATION = fmt


def rotvec_to_matrix(rx: float, ry: float, rz: float) -> Mat3:
    """회전 벡터를 3x3 회전 행렬로 바꾼다(로드리게스 공식)."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    kx, ky, kz = rx / theta, ry / theta, rz / theta
    c, s, v = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
    return [
        [kx * kx * v + c,      kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c,      ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
    ]


def matrix_to_rotvec(m: Mat3) -> Vec3:
    """3x3 회전 행렬을 회전 벡터로 되돌린다."""
    trace = m[0][0] + m[1][1] + m[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return [0.0, 0.0, 0.0]
    if abs(math.pi - theta) < 1e-6:
        # 180도 근처는 sin(theta) 가 0 에 가까워 일반식이 불안정하다.
        # 대각 성분에서 축을 뽑는다.
        axis = [
            math.sqrt(max(0.0, (m[0][0] + 1.0) / 2.0)),
            math.sqrt(max(0.0, (m[1][1] + 1.0) / 2.0)),
            math.sqrt(max(0.0, (m[2][2] + 1.0) / 2.0)),
        ]
        # 부호는 비대각 성분으로 정한다.
        if m[2][1] - m[1][2] < 0:
            axis[0] = -axis[0]
        if m[0][2] - m[2][0] < 0:
            axis[1] = -axis[1]
        if m[1][0] - m[0][1] < 0:
            axis[2] = -axis[2]
        return [a * theta for a in axis]
    k = theta / (2.0 * math.sin(theta))
    return [
        (m[2][1] - m[1][2]) * k,
        (m[0][2] - m[2][0]) * k,
        (m[1][0] - m[0][1]) * k,
    ]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Mat3:
    """ZYX 오일러각(rad)을 회전 행렬로 바꾼다. 핸드아이 설정 입력용이다."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def matrix_to_rpy(m: Mat3) -> Vec3:
    """회전 행렬 -> RPY (R = Rz(yaw)·Ry(pitch)·Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:                        # 짐벌락 — roll 을 0 으로 두고 yaw 에 몰아준다
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    return [roll, pitch, yaw]


def orientation_to_matrix(a: float, b: float, c: float) -> Mat3:
    """pose 의 회전 3 개를 지금 표기(ORIENTATION)대로 회전 행렬로."""
    if ORIENTATION == "rpy":
        return rpy_to_matrix(a, b, c)
    return rotvec_to_matrix(a, b, c)


def matrix_to_orientation(m: Mat3) -> Vec3:
    return matrix_to_rpy(m) if ORIENTATION == "rpy" else matrix_to_rotvec(m)


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def pose_to_rt(pose: Pose) -> tuple:
    """pose -> (회전행렬, 위치)."""
    return orientation_to_matrix(pose[3], pose[4], pose[5]), [pose[0], pose[1], pose[2]]


def rt_to_pose(rot: Mat3, pos: Vec3) -> Pose:
    return [pos[0], pos[1], pos[2]] + matrix_to_orientation(rot)


def tool_axis(pose: Pose, axis: int = 2) -> Vec3:
    """TCP 의 축 방향(베이스 기준 단위 벡터). axis 0=x, 1=y, 2=z."""
    rot, _ = pose_to_rt(pose)
    return [rot[0][axis], rot[1][axis], rot[2][axis]]


def transform_point(pose: Pose, point: Vec3) -> Vec3:
    """`pose` 프레임에서 본 점을 기준 프레임 좌표로 바꾼다."""
    rot, pos = pose_to_rt(pose)
    moved = mat_vec(rot, point)
    return [moved[i] + pos[i] for i in range(3)]


def transform_in_tool(pose: Pose, offset: Vec3) -> Pose:
    """툴 프레임 기준 평행이동을 적용한 새 pose 를 돌려준다(자세는 유지)."""
    rot, pos = pose_to_rt(pose)
    moved = mat_vec(rot, offset)
    return [pos[0] + moved[0], pos[1] + moved[1], pos[2] + moved[2],
            pose[3], pose[4], pose[5]]


def distance_mm(a: Pose, b: Pose) -> float:
    """두 pose 의 위치 차이 [mm]. 도착 판정에 쓴다."""
    return 1000.0 * math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def angle_diff_deg(a: Pose, b: Pose) -> float:
    """두 pose 의 자세 차이 [deg]. 도착 판정에 쓴다."""
    ra, _ = pose_to_rt(a)
    rb, _ = pose_to_rt(b)
    # R = Ra^T * Rb 의 회전각.
    rat = [[ra[j][i] for j in range(3)] for i in range(3)]
    rel = mat_mul(rat, rb)
    trace = rel[0][0] + rel[1][1] + rel[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_theta))


def joint_diff_deg(a, b) -> float:
    """두 관절값 [rad] 의 축별 차이 중 가장 큰 것 [deg]. 도착 판정에 쓴다."""
    return max(abs(math.degrees(x - y)) for x, y in zip(a, b))


def fk_dh(joints, a, d, alpha, theta_offset=None) -> tuple:
    """표준 DH 정기구학 -> (회전행렬, 위치[m]) 플랜지 기준.

    T_i = Rz(q_i + θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
    Elite 컨트롤러는 30001 상태 패킷에 이 값들을 실어 준다(dh_a/d/alpha/theta).
    회전 표기 확인 도구와 시뮬레이터가 쓴다.
    """
    theta_offset = theta_offset or [0.0] * 6
    rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    pos = [0.0, 0.0, 0.0]
    for i in range(6):
        th = joints[i] + theta_offset[i]
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(alpha[i]), math.sin(alpha[i])
        link = [[ct, -st * ca, st * sa],
                [st, ct * ca, -ct * sa],
                [0.0, sa, ca]]
        offset = [a[i] * ct, a[i] * st, d[i]]
        moved = mat_vec(rot, offset)
        pos = [pos[k] + moved[k] for k in range(3)]
        rot = mat_mul(rot, link)
    return rot, pos


def rotation_angle_deg(m1: Mat3, m2: Mat3) -> float:
    """두 회전 행렬 사이 각도 [deg]."""
    m1t = [[m1[j][i] for j in range(3)] for i in range(3)]
    rel = mat_mul(m1t, m2)
    trace = rel[0][0] + rel[1][1] + rel[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))
