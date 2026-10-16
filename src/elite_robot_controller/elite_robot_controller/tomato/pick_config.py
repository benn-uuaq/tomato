"""토마토 픽앤플레이스 설정.

**사람이 고치는 단위는 mm 와 deg 다.** 로봇 스크립트(movel/movej)는 m 와
rad 를 쓰므로 여기서 한 번만 환산해 두고, 시퀀스 코드는 환산된 값만 본다.

관절값은 사람 쪽에서 deg, 스크립트 쪽에서 rad 다.

현장에서 가르치는 값(홈·대기·놓는 자리·경유점)은 설정 파일이 아니라
`teach_store` 가 관리하는 파일에 저장되고, 설정 위에 덮어쓴다. 비어 있는
동안 시퀀스는 시작을 거절하고 무엇이 비었는지 알린다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import geometry

#: 사람이 쓰는 pose 단위 -> 스크립트 단위.
MM_TO_M = 0.001
DEG_TO_RAD = math.pi / 180.0

#: 자세를 아직 안 정했다는 표시. 6 개 모두 0 이면 '비어 있음'으로 본다.
_EMPTY_POSE = [0.0] * 6


def pose_to_si(pose_mm_deg) -> list:
    """[mm, mm, mm, deg, deg, deg] -> [m, m, m, rad, rad, rad]."""
    values = [float(v) for v in pose_mm_deg]
    return [values[0] * MM_TO_M, values[1] * MM_TO_M, values[2] * MM_TO_M,
            values[3] * DEG_TO_RAD, values[4] * DEG_TO_RAD, values[5] * DEG_TO_RAD]


def joints_to_rad(joints_deg) -> list:
    return [float(v) * DEG_TO_RAD for v in joints_deg]


def joints_to_deg(joints_rad) -> list:
    return [float(v) / DEG_TO_RAD for v in joints_rad]


def pose_to_human(pose_si) -> list:
    """[m, rad] -> [mm, deg]. UI 표시와 로그에 쓴다."""
    return [pose_si[0] / MM_TO_M, pose_si[1] / MM_TO_M, pose_si[2] / MM_TO_M,
            pose_si[3] / DEG_TO_RAD, pose_si[4] / DEG_TO_RAD, pose_si[5] / DEG_TO_RAD]


@dataclass
class VisionConfig:
    """비전 설정. 토픽은 촬영 종류(vision_interface.MODES)로 정해진다."""

    #: 3 단계 첫 촬영 — 대기 자세에서 찾는다. "nearest"(가장 가까운 1 개, 기본)
    #: | "pick"(전체 중 첫 번째) | "weight"(예전 면적·높이 노드).
    first_capture: str = "nearest"
    #: 4 단계 뒤 재측정 — 카메라 중심을 맞춘 다음. "middle"(화면 3x3 가운데 칸의
    #: 최근접 1 개, 기본) | "nearest" | ...  여기서 받은 무게가 정확한 값이다.
    recapture: str = "middle"
    #: 촬영 후 결과를 기다리는 한도 [s]. 한양대 노드는 1.5 초 측정한다.
    detection_timeout_s: float = 10.0
    #: 검출 좌표 해석 방식: "camera_delta"(기본) | "camera" | "tool" | "base".
    detection_frame: str = "camera_delta"
    #: camera_delta 에서 쓰는 축 대응. 앞이 dx, 뒤가 dy 를 만드는 카메라 축.
    #: README 의 "(x, y) -> (-y, -x)" 가 기본값이다. **align_frame 이 tool 이면
    #: TCP 의 x·y 축 기준**이다 — 카메라는 툴에 붙어 있어 이 대응이 자세와
    #: 상관없이 같다. 벽을 보는 자세로 바꾼 뒤 반드시 확인한다.
    camera_axis_map: list = field(default_factory=lambda: ["-y", "-x"])
    #: 카메라 중심 맞추기를 어느 축으로 움직일지: "tool"(기본, TCP 의 x·y 평면)
    #: | "base"(예전 — 아래를 보는 자세에서만 맞다).
    align_frame: str = "tool"
    #: 한 번에 이만큼보다 크게 옮기라는 결과는 거절한다 [mm]. 축 대응이 틀렸거나
    #: 엉뚱한 물체를 잡았을 때 크게 휘두르지 않게.
    align_max_step_mm: float = 100.0
    #: 두 번째 촬영의 오차가 첫 번째의 이 비율보다 크면 축 대응이 틀린 것으로
    #: 보고 멈춘다(맞게 움직였다면 오차가 확 줄어야 한다).
    axis_check_ratio: float = 0.7
    #: 정렬을 몇 번까지 되풀이할지. 1 이면 "이동 -> 재촬영" 을 한 번 더 한다
    #: (README: 중앙에 놓고 다시 찍은 무게가 정확한 값이다).
    align_iterations: int = 1
    #: 재촬영 후 남은 X·Y 오차가 이보다 크면 한 번 더 맞춘다 [mm].
    align_tolerance_mm: float = 5.0
    #: 핸드인아이 보정 — 툴(플랜지) 프레임에서 본 카메라의 위치·자세.
    #: detection_frame 이 "camera" 일 때만 쓴다.
    hand_eye_xyz_mm: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    hand_eye_rpy_deg: list = field(default_factory=lambda: [0.0, 0.0, 0.0])

    @property
    def configured(self) -> bool:
        """두 촬영 종류가 모두 아는 것이어야 시퀀스를 돌릴 수 있다."""
        from .vision_interface import MODES
        return self.first_capture in MODES and self.recapture in MODES

    @property
    def hand_eye_pose(self) -> list:
        """툴 -> 카메라 변환을 pose 형태로 돌려준다(m, rotvec)."""
        rot = geometry.rpy_to_matrix(*[v * DEG_TO_RAD for v in self.hand_eye_rpy_deg])
        pos = [v * MM_TO_M for v in self.hand_eye_xyz_mm]
        return geometry.rt_to_pose(rot, pos)

    @property
    def hand_eye_set(self) -> bool:
        return any(abs(v) > 1e-9 for v in list(self.hand_eye_xyz_mm) + list(self.hand_eye_rpy_deg))


@dataclass
class GripperConfig:
    """그리퍼 제어 정보. 제어 방법이 정해지기 전까지는 "none" 이다."""

    #: "int_topic"(기본, 다이나믹셀) | "topic" | "serial" | "none".
    #:  int_topic  : std_msgs/Int32 로 목표 위치를 보낸다(XD540 그리퍼).
    #:  topic      : std_msgs/Bool (True 닫기 / False 열기).
    #:  serial     : Orin 에서 직접 시리얼로 문자열을 보낸다.
    #:  none       : 아무것도 안 보내고 로그만 남긴다(로봇 동작만 볼 때).
    backend: str = "int_topic"
    #: 그리퍼 명령 토픽. int_topic 이면 Int32, topic 이면 Bool.
    topic: str = "/set_pos"
    #: int_topic 일 때의 목표 위치 값(0~4095). README.odt(2026-10) 기준 4095 가 파지,
    #: 0 이 열림(예전 3185).
    close_value: int = 4095
    open_value: int = 0
    #: backend 가 serial 일 때(Orin nano 직결). 포트와 보드레이트, 명령 문자열.
    serial_port: str = ""
    serial_baud: int = 115200
    close_command: str = ""
    open_command: str = ""
    #: 그리퍼 동작이 끝날 때까지 기다리는 시간 [s]. 피드백이 없으므로 시간으로 본다.
    action_wait_s: float = 1.5
    #: 토마토 중심에 카메라를 맞춘 상태에서, 그리퍼가 토마토 중심에 오도록
    #: 더 움직일 양 [mm]. README 측정값은 X +100, Y +32.5 (두산·아래 보기 기준).
    tcp_offset_mm: list = field(default_factory=lambda: [100.0, 32.5, 0.0])
    #: 위 값을 어느 프레임에서 더할지: "tool"(기본 — 카메라·그리퍼 모두 툴에
    #: 붙어 있어 자세와 상관없다) | "base"(예전).
    offset_frame: str = "tool"

    @property
    def configured(self) -> bool:
        if self.backend in ("topic", "int_topic"):
            return bool(self.topic)
        if self.backend == "serial":
            return bool(self.serial_port and self.close_command and self.open_command)
        return False


@dataclass
class DescendConfig:
    """7 단계 하강 — **TCP +z 방향으로** 얼마나 들어갈지.

    벽을 보는 자세에서는 베이스 Z 가 아니라 TCP 가 바라보는 쪽으로 들어가야
    한다. 거리는 카메라가 잰 토마토까지의 깊이로 정한다(mode = vision):

        이동 = 토마토 깊이 + camera_z_offset − gripper_length + grasp_depth

    - 토마토 깊이: 중심 맞춘 뒤 마지막 촬영의 카메라 Z (토마토 앞면까지)
    - camera_z_offset: 플랜지 면 -> 카메라 앞면 (README.odt: 8 + 25 = 33 mm)
    - gripper_length: 플랜지 면 -> 핑거 끝 (README.odt: 펼친 끝단 172 mm)
    - grasp_depth: 핑거 끝이 토마토 앞면을 지나 더 들어갈 양(토마토 반지름 정도)
    """

    #: "vision"(기본, 깊이로 계산) | "fixed"(fixed_mm 만큼 그대로).
    mode: str = "vision"
    camera_z_offset_mm: float = 33.0
    gripper_length_mm: float = 172.0
    #: 한양대 실측 5 회(2026-10): 카메라 중심 자세에서 파지 자세까지 TCP 를 늘
    #: (토마토 앞면 깊이 − 91.5) mm 내렸다 -> 33 − 172 + g = −91.5 -> g = 47.5.
    grasp_depth_mm: float = 47.5
    fixed_mm: float = 100.0
    #: (선택) 계산한 이동이 이보다 길면 거절한다 [mm]. 0 이면 끔(기본).
    #: 하강 한계는 이 값이 아니라 **가르친 하강 한계 자세의 Z** 다(PickConfig).
    max_travel_mm: float = 0.0

    def travel_mm(self, depth_mm: float) -> float:
        if self.mode == "fixed":
            return float(self.fixed_mm)
        return (float(depth_mm) + self.camera_z_offset_mm
                - self.gripper_length_mm + self.grasp_depth_mm)


@dataclass
class PickConfig:
    """시퀀스 전체 설정. pose 는 SI(m, rad), 관절값은 rad 로 들고 있는다."""

    #: 1·10·15·17 단계 홈 자세 (관절값). 가르치기 전에는 None.
    home_joint: list = None
    pick_wait_pose: list = field(default_factory=lambda: list(_EMPTY_POSE))
    #: 대기 자세의 관절값. 가르칠 때 같이 저장한다. 없으면 로봇이 pose 로
    #: 역기구학을 풀어 간다(get_inverse_kin).
    pick_wait_joint: list = None
    place_point_pose: list = field(default_factory=lambda: list(_EMPTY_POSE))
    place_point_joint: list = None
    #: ★ 경유점 — 홈 <-> 대기 / 홈 <-> 놓는 자리 사이에 거쳐 갈 관절값들.
    #: 홈에서 나가는 순서로 적고, 돌아올 때는 거꾸로 거친다.
    waypoints_wait: list = field(default_factory=list)
    waypoints_place: list = field(default_factory=list)
    #: 하강 한계 — 가르친 자세(SI). 이 자세의 **베이스 Z 보다 아래로는** 4·6·7 단계
    #: (중심 맞춤·파지점 정렬·하강)가 가지 않는다. 가르치기 전에는 None(시작 거절).
    descend_limit_pose: list = None

    #: 놓기 전 접근 높이 [m] — 놓는 자리 위로 이만큼 (베이스 +Z, 상자 위).
    place_approach_height: float = 0.050
    #: 이 높이가 남았을 때 그리퍼를 연다 [m].
    place_release_gap: float = 0.010

    #: 직선 이동(movel) 속도·가속도 [m/s], [m/s^2].
    move_speed: float = 0.25
    move_accel: float = 0.80
    #: 관절 이동(movej) 속도·가속도 [rad/s], [rad/s^2].
    joint_speed: float = 0.5
    joint_accel: float = 1.0
    #: 경유점에서 멈추지 않고 이어 갈 반경 [m]. 0 이면 경유점마다 선다(안전).
    waypoint_blend: float = 0.0
    descend_speed: float = 0.06
    place_descend_speed: float = 0.02
    #: 도착 판정.
    pose_tolerance_mm: float = 2.0
    angle_tolerance_deg: float = 2.0
    joint_tolerance_deg: float = 0.5
    settle_time_s: float = 0.3
    move_timeout_s: float = 40.0
    #: 로봇이 보고하는 회전 표기: "rpy" | "rotvec" (geometry 머리말 참고).
    orientation_format: str = "rpy"

    vision: VisionConfig = field(default_factory=VisionConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    descend: DescendConfig = field(default_factory=DescendConfig)

    @property
    def wait_pose_set(self) -> bool:
        return any(abs(v) > 1e-9 for v in self.pick_wait_pose)

    @property
    def place_pose_set(self) -> bool:
        return any(abs(v) > 1e-9 for v in self.place_point_pose)

    @property
    def descend_limit_z(self):
        """하강 한계 베이스 Z [m]. 가르치기 전에는 None."""
        return self.descend_limit_pose[2] if self.descend_limit_pose else None

    def missing(self) -> list:
        """아직 못 채운 설정 이름을 돌려준다. UI 가 '시작' 버튼을 막는 근거다."""
        gaps = []
        if not self.home_joint:
            gaps.append("home_joint")
        if not self.wait_pose_set:
            gaps.append("pick_wait_pose")
        if not self.place_pose_set:
            gaps.append("place_point_pose")
        if self.descend_limit_pose is None:
            gaps.append("descend_limit (하강 한계 — 작업 UI 에서 가르치기)")
        if not self.vision.configured:
            gaps.append("vision.first_capture / vision.recapture")
        if self.vision.detection_frame == "camera" and not self.vision.hand_eye_set:
            gaps.append("vision.hand_eye_xyz_mm / hand_eye_rpy_deg")
        if not self.gripper.configured:
            gaps.append("gripper.backend 설정")
        return gaps


#: (파라미터 이름, 기본값). 노드가 그대로 declare 한다.
#: 홈·관절값·경유점은 여기 없다 — 펜던트로 가르쳐 teach_store 파일에 저장한다
#: (경유점처럼 "목록의 목록" 은 ROS 파라미터로 담을 수 없기도 하다).
PARAMETER_DEFAULTS = [
    ("pick_wait_pose_mm_deg", [0.0] * 6),
    ("place_point_pose_mm_deg", [0.0] * 6),
    ("place_approach_mm", 50.0),
    ("place_release_gap_mm", 10.0),
    ("move_speed", 0.25),
    ("move_accel", 0.80),
    ("joint_speed", 0.5),
    ("joint_accel", 1.0),
    ("waypoint_blend_m", 0.0),
    ("descend_speed", 0.06),
    ("place_descend_speed", 0.02),
    ("pose_tolerance_mm", 2.0),
    ("angle_tolerance_deg", 2.0),
    ("joint_tolerance_deg", 0.5),
    ("settle_time_s", 0.3),
    ("move_timeout_s", 40.0),
    ("orientation_format", "rpy"),
    ("vision.first_capture", "nearest"),
    ("vision.recapture", "middle"),
    ("vision.detection_timeout_s", 10.0),
    ("vision.detection_frame", "camera_delta"),
    ("vision.camera_axis_map", ["-y", "-x"]),
    ("vision.align_frame", "tool"),
    ("vision.align_iterations", 1),
    ("vision.align_tolerance_mm", 5.0),
    ("vision.align_max_step_mm", 100.0),
    ("vision.axis_check_ratio", 0.7),
    ("vision.hand_eye_xyz_mm", [0.0, 0.0, 0.0]),
    ("vision.hand_eye_rpy_deg", [0.0, 0.0, 0.0]),
    ("gripper.backend", "int_topic"),
    ("gripper.topic", "/set_pos"),
    ("gripper.close_value", 4095),
    ("gripper.open_value", 0),
    ("gripper.serial_port", ""),
    ("gripper.serial_baud", 115200),
    ("gripper.close_command", ""),
    ("gripper.open_command", ""),
    ("gripper.action_wait_s", 1.5),
    ("gripper.tcp_offset_mm", [100.0, 32.5, 0.0]),
    ("gripper.offset_frame", "tool"),
    ("descend.mode", "vision"),
    ("descend.camera_z_offset_mm", 33.0),
    ("descend.gripper_length_mm", 172.0),
    ("descend.grasp_depth_mm", 47.5),
    ("descend.fixed_mm", 100.0),
    ("descend.max_travel_mm", 0.0),
]


def declare_all(node) -> None:
    """노드에 파라미터를 모두 선언한다."""
    for name, default in PARAMETER_DEFAULTS:
        node.declare_parameter(name, default)


def load(node) -> PickConfig:
    """선언된 파라미터를 읽어 PickConfig 를 만든다."""
    def value(name):
        return node.get_parameter(name).value

    return PickConfig(
        pick_wait_pose=pose_to_si(value("pick_wait_pose_mm_deg")),
        place_point_pose=pose_to_si(value("place_point_pose_mm_deg")),
        place_approach_height=float(value("place_approach_mm")) * MM_TO_M,
        place_release_gap=float(value("place_release_gap_mm")) * MM_TO_M,
        move_speed=float(value("move_speed")),
        move_accel=float(value("move_accel")),
        joint_speed=float(value("joint_speed")),
        joint_accel=float(value("joint_accel")),
        waypoint_blend=float(value("waypoint_blend_m")),
        descend_speed=float(value("descend_speed")),
        place_descend_speed=float(value("place_descend_speed")),
        pose_tolerance_mm=float(value("pose_tolerance_mm")),
        angle_tolerance_deg=float(value("angle_tolerance_deg")),
        joint_tolerance_deg=float(value("joint_tolerance_deg")),
        settle_time_s=float(value("settle_time_s")),
        move_timeout_s=float(value("move_timeout_s")),
        orientation_format=str(value("orientation_format")),
        vision=VisionConfig(
            first_capture=str(value("vision.first_capture")),
            recapture=str(value("vision.recapture")),
            detection_timeout_s=float(value("vision.detection_timeout_s")),
            detection_frame=str(value("vision.detection_frame")),
            camera_axis_map=[str(v) for v in value("vision.camera_axis_map")],
            align_frame=str(value("vision.align_frame")),
            align_iterations=int(value("vision.align_iterations")),
            align_tolerance_mm=float(value("vision.align_tolerance_mm")),
            align_max_step_mm=float(value("vision.align_max_step_mm")),
            axis_check_ratio=float(value("vision.axis_check_ratio")),
            hand_eye_xyz_mm=[float(v) for v in value("vision.hand_eye_xyz_mm")],
            hand_eye_rpy_deg=[float(v) for v in value("vision.hand_eye_rpy_deg")],
        ),
        gripper=GripperConfig(
            backend=str(value("gripper.backend")),
            topic=str(value("gripper.topic")),
            close_value=int(value("gripper.close_value")),
            open_value=int(value("gripper.open_value")),
            serial_port=str(value("gripper.serial_port")),
            serial_baud=int(value("gripper.serial_baud")),
            close_command=str(value("gripper.close_command")),
            open_command=str(value("gripper.open_command")),
            action_wait_s=float(value("gripper.action_wait_s")),
            tcp_offset_mm=[float(v) for v in value("gripper.tcp_offset_mm")],
            offset_frame=str(value("gripper.offset_frame")),
        ),
        descend=DescendConfig(
            mode=str(value("descend.mode")),
            camera_z_offset_mm=float(value("descend.camera_z_offset_mm")),
            gripper_length_mm=float(value("descend.gripper_length_mm")),
            grasp_depth_mm=float(value("descend.grasp_depth_mm")),
            fixed_mm=float(value("descend.fixed_mm")),
            max_travel_mm=float(value("descend.max_travel_mm")),
        ),
    )
