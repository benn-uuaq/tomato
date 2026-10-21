"""펜던트로 옮겨 둔 **지금 위치**를 그대로 저장해 두는 곳.

값을 손으로 옮겨 적으면 단위(mm/deg)나 부호를 틀리기 쉽다. 로봇을 원하는
자리로 옮긴 뒤 버튼 한 번으로 현재 TCP 를 읽어 파일에 적어 두고, 다음
기동 때 그 값을 쓴다.

설정 파일(`tomato_pick.yaml`)을 직접 고치지 않는 이유는 그 파일이 `install/`
안에 설치되어 colcon build 때마다 덮이기 때문이다. 대신 사람이 읽을 수 있는
별도 yaml 한 장에 적고, **설정을 읽은 뒤 그 위에 덮어쓴다.**

    ~/.tomato_pick/taught_poses.yaml

우선순위: `tomato_pick.yaml` < 이 파일. 이 파일을 지우면 설정값으로 돌아간다.
"""

from __future__ import annotations

import os
from datetime import datetime

import yaml

from . import pick_config

#: 이 파일이 덮어쓸 수 있는 항목 — 사람이 펜던트로 가르치거나 UI 에서 정하는 값.
#: 관절값은 deg, 위치는 mm, 회전은 deg 다(펜던트 표시와 같게).
FIELDS = (
    "home_joint_deg",
    "pick_wait_pose_mm_deg", "pick_wait_joint_deg",
    "place_point_pose_mm_deg", "place_point_joint_deg",
    # ★ 경유점: 홈에서 나가는 순서. 돌아올 때는 거꾸로 거친다.
    "waypoints_wait_deg",        # 홈 <-> 대기 자세
    "waypoints_place_deg",       # 홈 <-> 놓는 자리
    "place_approach_mm", "place_release_gap_mm",
    # 하강 한계 — 이 자세의 베이스 Z 보다 아래로는 가지 않는다(4·6·7 단계).
    "descend_limit_pose_mm_deg",
    # 그리퍼 — 작업 UI 의 [그리퍼] 칸. 위치는 다이나믹셀 값, 시간은 초.
    "gripper_close_value", "gripper_open_value",
    "gripper_close_wait_s", "gripper_open_wait_s",
    # 동작별 속도 — 작업 UI 의 [동작 속도]. 화면 단위: movel mm/s, movej °/s.
    "motion_speeds",            # {상태: 속도}
    "motion_accels",            # {"movel": mm/s², "movej": °/s²}
)

#: 작업 UI 에서 숫자로 정하는 값 -> (설정 객체 경로, 형). 시퀀스 노드의 SetParameters
#: 이름도 이것이다(UI -> 파라미터 -> 이 파일).
GRIPPER_SETTINGS = {
    "gripper_close_value": ("close_value", int),
    "gripper_open_value": ("open_value", int),
    "gripper_close_wait_s": ("close_wait_s", float),
    "gripper_open_wait_s": ("open_wait_s", float),
}
#: 예전 파일에 남아 있을 수 있는 항목. 읽을 때 조용히 버린다.
#: (max_descend_z_mm: 예전에 숫자로 넣던 하강 한계 — 이제는 자세를 가르친다.
#:  옛 값이 모르는 사이에 되살아나지 않게 읽지 않는다.)

DEFAULT_PATH = "~/.tomato_pick/taught_poses.yaml"


def resolve(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path or DEFAULT_PATH))


def load(path: str) -> dict:
    """저장해 둔 값을 읽는다. 파일이 없으면 빈 dict."""
    full = resolve(path)
    if not os.path.isfile(full):
        return {}
    try:
        with open(full, encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception:
        return {}
    return {key: value for key, value in data.items() if key in FIELDS}


def save(path: str, values: dict) -> str:
    """가르친 값을 파일에 적는다. 저장한 경로를 돌려준다."""
    full = resolve(path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    body = {key: values[key] for key in FIELDS if key in values}
    header = (
        "# 펜던트로 가르친 값. 시퀀스 노드가 기동할 때 tomato_pick.yaml 위에 덮어쓴다.\n"
        "# 단위: 위치 mm, 회전 deg, 관절 deg (펜던트 표시와 같다).\n"
        "# 경유점(waypoints_*)은 홈에서 나가는 순서다 — 돌아올 때는 거꾸로 거친다.\n"
        f"# 마지막 저장: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
    )
    with open(full, "w", encoding="utf-8") as handle:
        handle.write(header)
        yaml.safe_dump(body, handle, allow_unicode=True, sort_keys=True,
                       default_flow_style=False)
    return full


def _joints(values, key):
    joints = values.get(key)
    if joints and len(joints) == 6:
        return pick_config.joints_to_rad(joints)
    return None


def apply(cfg, values: dict) -> list:
    """읽어 온 값을 설정에 덮어쓴다. 실제로 덮어쓴 항목 이름을 돌려준다."""
    applied = []
    joints = _joints(values, "home_joint_deg")
    if joints:
        cfg.home_joint = joints
        applied.append("home_joint_deg")
    pose = values.get("pick_wait_pose_mm_deg")
    if pose and len(pose) == 6:
        cfg.pick_wait_pose = pick_config.pose_to_si(pose)
        applied.append("pick_wait_pose_mm_deg")
    joints = _joints(values, "pick_wait_joint_deg")
    if joints:
        cfg.pick_wait_joint = joints
        applied.append("pick_wait_joint_deg")
    pose = values.get("place_point_pose_mm_deg")
    if pose and len(pose) == 6:
        cfg.place_point_pose = pick_config.pose_to_si(pose)
        applied.append("place_point_pose_mm_deg")
    joints = _joints(values, "place_point_joint_deg")
    if joints:
        cfg.place_point_joint = joints
        applied.append("place_point_joint_deg")
    for key, attr in (("waypoints_wait_deg", "waypoints_wait"),
                      ("waypoints_place_deg", "waypoints_place")):
        points = [pick_config.joints_to_rad(p) for p in (values.get(key) or []) if len(p) == 6]
        setattr(cfg, attr, points)
        if points:
            applied.append(key)
    pose = values.get("descend_limit_pose_mm_deg")
    if pose and len(pose) == 6:
        cfg.descend_limit_pose = pick_config.pose_to_si(pose)
        applied.append("descend_limit_pose_mm_deg")
    for key, (attr, kind) in GRIPPER_SETTINGS.items():
        if values.get(key) is not None:
            setattr(cfg.gripper, attr, kind(values[key]))
            applied.append(key)
    speeds = values.get("motion_speeds") or {}
    cfg.motion_speeds = {
        state: pick_config.motion_from_ui(pick_config.MOTION_TABLE[state][0], float(v))
        for state, v in speeds.items() if state in pick_config.MOTION_TABLE}
    if cfg.motion_speeds:
        applied.append("motion_speeds")
    accels = values.get("motion_accels") or {}
    cfg.motion_accels = {kind: pick_config.motion_from_ui(kind, float(v))
                         for kind, v in accels.items() if kind in ("movel", "movej")}
    if cfg.motion_accels:
        applied.append("motion_accels")
    if values.get("place_approach_mm") is not None:
        cfg.place_approach_height = float(values["place_approach_mm"]) * pick_config.MM_TO_M
        applied.append("place_approach_mm")
    if values.get("place_release_gap_mm") is not None:
        cfg.place_release_gap = float(values["place_release_gap_mm"]) * pick_config.MM_TO_M
        applied.append("place_release_gap_mm")
    return applied
