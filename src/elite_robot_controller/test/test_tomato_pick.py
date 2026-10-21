"""로봇 없이 검증하는 픽앤플레이스 계산.

노드를 띄우지 않고 순수 계산만 본다 — 좌표 변환, 단위 환산, 검출 파싱,
그리고 "설정이 비어 있으면 시작을 막는다"는 규칙이다.
"""

import math

from elite_robot_controller.tomato import geometry, pick_config, vision_interface


def test_pose_units_round_trip():
    human = [400.0, -50.0, 300.0, 180.0, 0.0, 90.0]
    si = pick_config.pose_to_si(human)
    assert si[0] == 0.4 and si[1] == -0.05
    assert math.isclose(si[3], math.pi, rel_tol=1e-9)
    back = pick_config.pose_to_human(si)
    assert all(math.isclose(a, b, rel_tol=1e-9) for a, b in zip(human, back))


def test_rotvec_round_trip():
    for vec in ([0.1, -0.4, 2.9], [0.0, 0.0, math.pi], [0.0, 0.0, 0.0]):
        out = geometry.matrix_to_rotvec(geometry.rotvec_to_matrix(*vec))
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(vec, out))


def test_flat_array_is_parsed_as_five_fields():
    class Msg:
        data = [100.0, 20.0, 300.0, 55.0, 120.0, 150.0, -30.0, 310.0, 60.0, 140.0]

    detections = vision_interface.parse_detections(Msg())
    assert len(detections) == 2
    assert detections[0].x == 0.1 and detections[0].diameter == 55.0
    assert detections[1].weight == 140.0


def test_hand_in_eye_transform_uses_tool_pose():
    """카메라 좌표는 툴 pose 를 거쳐 베이스 좌표가 된다."""
    cfg = pick_config.VisionConfig(detection_frame="camera",
                                   hand_eye_xyz_mm=[0.0, 0.0, 50.0])
    detection = vision_interface.Detection(x=0.05, y=0.02, z=0.30)
    # TCP 가 X 축으로 180도 돌아 아래를 보는 자세.
    tool_pose = [0.4, 0.0, 0.4, math.pi, 0.0, 0.0]
    x, y, z = vision_interface.detection_to_base(detection, tool_pose, cfg)
    assert math.isclose(x, 0.45, abs_tol=1e-6)
    assert math.isclose(y, -0.02, abs_tol=1e-6)
    # 카메라가 툴보다 50 mm 앞(+Z)에 있고 아래를 보므로 350 mm 아래가 된다.
    assert math.isclose(z, 0.05, abs_tol=1e-6)


def test_base_frame_detection_is_not_transformed():
    cfg = pick_config.VisionConfig(detection_frame="base")
    detection = vision_interface.Detection(x=0.5, y=0.1, z=0.2)
    assert vision_interface.detection_to_base(
        detection, [0, 0, 0, 0, 0, 0], cfg) == [0.5, 0.1, 0.2]


def test_gripper_offset_shifts_tcp_the_other_way():
    """파지점이 툴 +X 로 30 mm 나와 있으면 TCP 는 그만큼 뒤로 선다."""
    pose = [0.45, 0.0, 0.4, 0.0, 0.0, 0.0]   # 회전 없음 -> 툴축 = 베이스축
    shifted = geometry.transform_in_tool(pose, [-0.03, 0.0, 0.0])
    assert math.isclose(shifted[0], 0.42, abs_tol=1e-9)
    # 파지점을 다시 앞으로 보내면 원래 목표로 돌아온다.
    grip = geometry.transform_in_tool(shifted, [0.03, 0.0, 0.0])
    assert math.isclose(grip[0], 0.45, abs_tol=1e-9)


def test_empty_config_lists_every_missing_value():
    gaps = pick_config.PickConfig().missing()
    for name in ("home_joint", "pick_wait_pose", "place_point_pose"):
        assert name in gaps
    # 하강 한계도 가르쳐야 시작할 수 있다.
    assert any(gap.startswith("descend_limit") for gap in gaps)


def test_filled_config_has_no_gaps():
    cfg = pick_config.PickConfig(
        pick_wait_pose=pick_config.pose_to_si([400, 0, 400, 180, 0, 0]),
        place_point_pose=pick_config.pose_to_si([300, -200, 300, 180, 0, 0]),
        home_joint=[0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
        descend_limit_pose=pick_config.pose_to_si([400, 0, 150, 180, 0, 0]),
        vision=pick_config.VisionConfig(hand_eye_xyz_mm=[0.0, 0.0, 50.0]),
        gripper=pick_config.GripperConfig(backend="topic", topic="/gripper/close"),
    )
    assert cfg.missing() == []


def test_camera_delta_matches_readme_mapping():
    """README: 카메라 (x, y) -> (-y, -x) 만큼 움직이면 중심에 온다."""
    cfg = pick_config.VisionConfig(detection_frame="camera_delta",
                                   camera_axis_map=["-y", "-x"])
    detection = vision_interface.Detection(x=0.060, y=0.040, z=0.22)
    tool_pose = [0.400, 0.000, 0.400, math.pi, 0.0, 0.0]
    x, y, z = vision_interface.detection_to_base(detection, tool_pose, cfg)
    assert math.isclose(x, 0.360, abs_tol=1e-9)   # 400 - 40 mm
    assert math.isclose(y, -0.060, abs_tol=1e-9)  # 0 - 60 mm
    assert z == tool_pose[2]                      # Z 는 건드리지 않는다


def test_camera_axis_map_can_be_flipped():
    """장착 방향이 다르면 축 대응만 바꿔 쓸 수 있어야 한다."""
    cfg = pick_config.VisionConfig(detection_frame="camera_delta",
                                   camera_axis_map=["+y", "+x"])
    detection = vision_interface.Detection(x=0.060, y=0.040, z=0.22)
    x, y, _ = vision_interface.detection_to_base(
        detection, [0.0, 0.0, 0.4, 0, 0, 0], cfg)
    assert math.isclose(x, 0.040, abs_tol=1e-9)
    assert math.isclose(y, 0.060, abs_tol=1e-9)


def test_weights_and_heights_merge_by_index():
    """세 토픽은 같은 순서로 온다 — i 번째끼리 같은 토마토여야 한다."""
    class PoseMsg:
        class P:
            def __init__(self, x, y, z):
                self.position = type("pos", (), {"x": x, "y": y, "z": z})()
        poses = [P(0.01, 0.02, 0.2), P(-0.03, 0.04, 0.25)]

    detections = vision_interface.parse_detections(PoseMsg())
    weights, heights = [120.0, 95.5], [0.058, 0.051]
    for i, det in enumerate(detections):
        det.weight = weights[i]
        det.height = heights[i] * 1000.0
    assert detections[0].as_dict()["weight_g"] == 120.0
    assert detections[1].as_dict()["height_mm"] == 51.0


def test_default_config_only_needs_taught_poses():
    """비전·그리퍼 기본값은 실제 인터페이스에 맞춰 두었으므로,
    현장에서 가르칠 값(대기 자세·놓는 자리·하강 높이)만 비어 있어야 한다."""
    gaps = pick_config.PickConfig().missing()
    assert "vision.detection_topic" not in gaps
    assert "gripper.backend 설정" not in gaps


def test_taught_values_override_config_and_ignore_unknown_keys(tmp_path):
    """펜던트로 가르친 값은 설정 파일 위에 덮어쓴다(관절은 deg 로 저장, rad 로 씀)."""
    from elite_robot_controller.tomato import teach_store

    path = str(tmp_path / "taught.yaml")
    teach_store.save(path, {
        "home_joint_deg": [0.0, -90.0, 90.0, 0.0, 90.0, 0.0],
        "pick_wait_pose_mm_deg": [400.0, 0.0, 525.0, -180.0, 0.0, -45.0],
        "pick_wait_joint_deg": [10.0, -80.0, 100.0, -20.0, 90.0, 0.0],
        "waypoints_wait_deg": [[5.0, -85.0, 95.0, -10.0, 90.0, 0.0]],
        "place_approach_mm": 80.0,
        "max_descend_z_mm": 210.0,   # 예전 항목 — 이제는 버린다
        "unknown_key": 1,            # 알 수 없는 항목은 저장도 반영도 하지 않는다
    })
    data = teach_store.load(path)
    assert "unknown_key" not in data and "max_descend_z_mm" not in data

    cfg = pick_config.PickConfig()
    applied = teach_store.apply(cfg, data)
    assert {"home_joint_deg", "pick_wait_pose_mm_deg", "pick_wait_joint_deg",
            "waypoints_wait_deg", "place_approach_mm"} <= set(applied)
    assert math.isclose(cfg.home_joint[1], -math.pi / 2, abs_tol=1e-9)
    assert math.isclose(cfg.pick_wait_pose[2], 0.525, abs_tol=1e-9)
    assert len(cfg.waypoints_wait) == 1 and math.isclose(cfg.waypoints_wait[0][0],
                                                         math.radians(5.0))
    assert math.isclose(cfg.place_approach_height, 0.080)
    # 아직 안 가르친 것만 남는다.
    assert cfg.missing() == ["place_point_pose", "descend_limit (하강 한계 — 작업 UI 에서 가르치기)"]


def test_missing_teach_file_is_not_an_error(tmp_path):
    from elite_robot_controller.tomato import teach_store
    assert teach_store.load(str(tmp_path / "없는파일.yaml")) == {}


def test_scripts_use_plain_lists_not_p_literals():
    """Elite 스크립트의 pose 는 `p[...]` 가 아니라 그냥 리스트다.

    `p` 는 내장 함수라 `p[...]` 를 보내면 컨트롤러가
    ``type_error_arg_not_subscriptable`` 로 스크립트를 버린다 —
    아무 오류 없이 로봇만 안 움직이는 상태가 된다(2026-09-21 CS616 확인).
    """
    from elite_robot_controller.tomato import motion

    pose = [0.4, 0.05, 0.4, 3.14, 0.0, 0.0]
    line = motion.movel(pose, 0.8, 0.25)
    assert line.startswith("movel([")
    assert "p[" not in line

    # movej 는 관절값을 받으므로 역기구학을 로봇이 풀게 한다.
    joint_line = motion.movej_pose(pose, 0.8, 0.25)
    assert joint_line.startswith("movej(get_inverse_kin([")
    assert "p[" not in joint_line

    assert motion.movej_joints([0.0] * 6, 1.0, 0.5).startswith("movej([")


def test_bridge_result_becomes_the_same_three_topics():
    """비전 PC 브리지의 result 한 건이 원래 비전 노드와 같은 토픽 3 개가 된다.

    그래야 시퀀스는 비전이 같은 장비든 다른 장비든 똑같이 동작한다.
    """
    from elite_robot_controller.tomato.vision_bridge_client import result_to_messages

    result = {
        "type": "result", "frame_id": "camera_color_optical_frame", "count": 2,
        "tomatoes": [
            {"index": 0, "position": {"x": 0.0193, "y": -0.0339, "z": 0.3035},
             "weight_g": 60.2, "height_m": 0.0386},
            # 무게·높이가 빠진 토마토도 인덱스가 밀리지 않아야 한다.
            {"index": 1, "position": {"x": 0.05, "y": 0.01, "z": 0.31},
             "weight_g": None, "height_m": None},
        ],
    }
    poses, weights, heights = result_to_messages(result)
    assert poses.header.frame_id == "camera_color_optical_frame"
    assert len(poses.poses) == 2
    assert math.isclose(poses.poses[0].position.x, 0.0193)
    assert list(weights.data)[0] == pytest_approx(60.2)
    assert len(weights.data) == len(heights.data) == 2
    assert weights.data[1] == 0.0 and heights.data[1] == 0.0

    # 0 개 검출도 빈 메시지로 그대로 전달한다(로봇이 대기에 매달리지 않게).
    empty = result_to_messages({"type": "result", "count": 0, "tomatoes": []})
    assert len(empty[0].poses) == 0 and len(empty[1].data) == 0


def pytest_approx(value, tol=1e-4):
    class _Approx:
        def __eq__(self, other):
            return abs(other - value) < tol
    return _Approx()


def test_ui_address_validation():
    """UI 에 넣은 주소를 검사한다 — 오타가 엉뚱한 곳으로 접속하지 않게."""
    from elite_robot_controller.tomato import connection

    assert connection.validate_host(" 192.168.1.200 ")[:2] == (True, "192.168.1.200")
    assert connection.validate_host("nano-desktop.local")[0]
    for bad in ("", "192.168.1.2000", "192.168.1", "192.168.1.200/24", "http://1.2.3.4", "a b"):
        ok, _, reason = connection.validate_host(bad)
        assert not ok and reason, bad


def test_ui_remembers_last_addresses(tmp_path):
    from elite_robot_controller.tomato import connection

    path = str(tmp_path / "connection.yaml")
    assert connection.load(path) == {}
    connection.save(path, {"robot_ip": "192.168.1.200", "vision_host": "192.168.1.10",
                           "unknown": "x"})
    assert connection.load(path) == {"robot_ip": "192.168.1.200",
                                     "vision_host": "192.168.1.10"}


def test_process_manager_start_log_stop():
    """패널이 부품을 켜고, 로그를 모으고, 자식까지 확실히 내린다."""
    import sys
    import time as _time
    from elite_robot_controller.tomato.process_manager import ManagedProcess

    # 자식 프로세스를 하나 더 낳는 명령 — 끌 때 그룹 전체가 죽어야 한다.
    script = ("import subprocess, sys, time;"
              "child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)']);"
              "print('\\x1b[32mready\\x1b[0m', flush=True);"
              "print('child', child.pid, flush=True);"
              "time.sleep(60)")
    process = ManagedProcess("demo", "데모", [sys.executable, "-c", script])
    ok, _ = process.start()
    assert ok and process.running
    assert not process.start()[0]          # 두 번 켜지 않는다

    deadline = _time.time() + 5
    while not process.contains("child") and _time.time() < deadline:
        _time.sleep(0.05)
    assert process.contains("ready")
    assert all("\x1b" not in line for line in process.lines)   # 색 코드는 지운다
    child_pid = int(next(l for l in process.lines if l.startswith("child")).split()[1])

    ok, _ = process.stop(grace_int=2.0)
    assert ok and not process.running
    _time.sleep(0.2)
    import os as _os
    try:
        _os.kill(child_pid, 0)
        alive = True
    except ProcessLookupError:
        alive = False
    assert not alive, "자식 프로세스가 남았다 — 카메라라면 장치를 못 놓는다"


def test_process_manager_reports_failed_exit():
    import sys
    import time as _time
    from elite_robot_controller.tomato.process_manager import ManagedProcess

    process = ManagedProcess("bad", "실패", [sys.executable, "-c",
                                            "print('Address already in use'); raise SystemExit(3)"])
    process.start()
    deadline = _time.time() + 5
    while process.exit_code is None and _time.time() < deadline:
        _time.sleep(0.05)
    assert process.exit_code == 3
    assert process.contains("Address already in use")
    # 없는 실행 파일도 노드를 죽이지 않고 실패로 알린다.
    missing = ManagedProcess("x", "없음", ["/no/such/program"])
    ok, message = missing.start()
    assert not ok and "실행 실패" in message



# ---------------------------------------------------------------------------
# 벽을 보는 자세 — TCP 기준 이동 (2026-10-13 시퀀스 변경)
# ---------------------------------------------------------------------------
def _wall_pose():
    """TCP z 가 베이스 +X(벽 쪽) 를 보는 자세: RPY (0, 90°, 0)."""
    from elite_robot_controller.tomato import geometry
    geometry.set_orientation_format("rpy")
    return [0.5, 0.1, 0.6, 0.0, math.pi / 2, 0.0]


def test_rpy_is_read_as_robot_reports_it():
    """CS616 이 아래 보기를 (-180, 0, -45) 로 보고한다 — RPY 라면 TCP z 가 베이스 -Z."""
    from elite_robot_controller.tomato import geometry
    geometry.set_orientation_format("rpy")
    z_axis = geometry.tool_axis([0.4, 0.0, 0.5, -math.pi, 0.0, -math.pi / 4])
    assert [round(v, 6) for v in z_axis] == [0.0, 0.0, -1.0]
    # +179.97° 와 -180° 는 같은 자세다(예전 회전 벡터 해석에서는 전혀 다른
    # 자세로 보아 도착 판정이 영영 안 됐을 것이다).
    a = [0, 0, 0, math.radians(179.97), 0, -0.785]
    b = [0, 0, 0, -math.pi, 0, -0.785]
    assert geometry.angle_diff_deg(a, b) < 0.1


def test_descend_goes_along_tcp_z_not_base_z():
    from elite_robot_controller.tomato import plan
    pose = _wall_pose()
    down = plan.descend_pose(pose, 0.10)
    # 벽 쪽(+X)으로 100 mm, 높이(Z)는 그대로.
    assert math.isclose(down[0], 0.60, abs_tol=1e-9)
    assert math.isclose(down[2], 0.60, abs_tol=1e-9)
    assert down[3:] == pose[3:]                 # 자세는 바꾸지 않는다


def test_center_and_grip_move_in_tcp_plane():
    """카메라 중심 맞추기·파지점 정렬은 TCP 의 x·y 평면 — 벽까지 거리는 그대로."""
    from elite_robot_controller.tomato import plan
    pose = _wall_pose()
    # RPY(0,90°,0): TCP x = 베이스 -Z, TCP y = 베이스 +Y
    centered = plan.center_pose(pose, [0.02, 0.03], "tool")
    assert math.isclose(centered[0], 0.50, abs_tol=1e-9)    # 벽 쪽으로는 안 움직임
    assert math.isclose(centered[1], 0.13, abs_tol=1e-9)
    assert math.isclose(centered[2], 0.58, abs_tol=1e-9)
    grip = plan.grip_pose(centered, [100.0, 0.0, 0.0], "tool")
    assert math.isclose(grip[2], 0.48, abs_tol=1e-9)
    assert math.isclose(grip[0], 0.50, abs_tol=1e-9)


def test_descend_distance_from_camera_depth():
    """거리 = 깊이 + 카메라 앞면(33) - 핑거 끝(172) + 더 들어갈 양(47.5, 한양대 실측)."""
    cfg = pick_config.DescendConfig()
    assert math.isclose(cfg.travel_mm(300.0), 208.5)
    fixed = pick_config.DescendConfig(mode="fixed", fixed_mm=120.0)
    assert fixed.travel_mm(999.0) == 120.0


def test_waypoint_path_program_goes_in_order():
    """경유점을 차례로 거쳐 목표로 — 돌아올 때는 거꾸로 넘겨 받는다."""
    from elite_robot_controller.tomato import motion
    w1, w2 = [0.1] * 6, [0.2] * 6
    home = [0.0] * 6
    out = motion.joint_path_program("go", [w1, w2], target_joints=[0.3] * 6)
    back = motion.joint_path_program("back", list(reversed([w1, w2])), target_joints=home)
    assert out.index("0.100000") < out.index("0.200000") < out.index("0.300000")
    assert back.index("0.200000") < back.index("0.100000")
    assert out.startswith("def go():") and out.endswith("end")
    assert "r=" not in out                       # 기본은 경유점마다 선다
    assert "r=0.0200" in motion.joint_path_program("b", [w1], target_joints=home, blend=0.02)



# ---------------------------------------------------------------------------
# 한양대 비전 업데이트 (2026-10) — 촬영 종류·그리퍼 4095·하강 보정
# ---------------------------------------------------------------------------
def test_descend_matches_hanyang_measurements():
    """한양대 실측 5 회: 카메라 중심 자세에서 파지 자세까지 TCP 를 (깊이 − 91.5) mm 내렸다."""
    cfg = pick_config.DescendConfig()
    samples = [  # (앞면 깊이 mm, 실제 하강 mm) — gripper_moving_test, v2 test
        (303.4, 450 - 238.1), (306.0, 450 - 235.5), (295.0, 450 - 246.5),
        (299.0, 450 - 242.5), (301.0, 450 - 240.5)]
    for depth, measured in samples:
        assert math.isclose(cfg.travel_mm(depth), measured, abs_tol=0.05), (depth, measured)


def test_gripper_closes_at_4095_by_default():
    assert pick_config.GripperConfig().close_value == 4095
    assert pick_config.GripperConfig().open_value == 0


def test_capture_modes_match_hanyang_node_topics():
    """시퀀스·브리지 클라이언트·브리지 서버가 같은 토픽 표를 쓴다."""
    from elite_robot_controller.tomato.vision_interface import MODES
    from elite_robot_controller.tomato.vision_bridge_client import SOURCES
    for mode, (trigger, pose, weight, height) in MODES.items():
        _, c_trigger, c_pose, c_weight, c_height = SOURCES[mode]
        assert (trigger, pose, weight, height) == (c_trigger, c_pose, c_weight, c_height)
    assert MODES["nearest"][0] == "/tomato/nearest_capture"
    assert MODES["middle"][1] == "/tomato/middle_position"
    cfg = pick_config.VisionConfig()
    assert (cfg.first_capture, cfg.recapture) == ("nearest", "middle")
    assert pick_config.VisionConfig(first_capture="bogus").configured is False


def test_bridge_server_knows_the_same_sources():
    """Orin 브리지(doosan_vision_grasp) 의 표도 같아야 한다 — 파일을 직접 읽어 비교한다."""
    import ast
    import os
    from elite_robot_controller.tomato.vision_bridge_client import SOURCES
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "..", "doosan_vision_grasp", "doosan_vision_grasp",
                        "tomato_tcp_bridge.py")
    tree = ast.parse(open(path, encoding="utf-8").read())
    server = next(ast.literal_eval(node.value) for node in tree.body
                  if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "SOURCES")
    assert {k: tuple(v) for k, v in server.items()} == {k: tuple(v) for k, v in SOURCES.items()}


def test_vision_remote_ssh_command_is_key_only():
    """(두 대로 나눌 때) SSH 는 비밀번호를 기다리지 않고, `~/` 는 원격 홈으로 푼다."""
    from elite_robot_controller.tomato import vision_remote

    cmd = vision_remote.ssh_command("nano", "192.168.1.10", "~/tomato_ws", "echo hi")
    assert "BatchMode=yes" in cmd and cmd[-2] == "nano@192.168.1.10"
    assert cmd[-1] == "cd ~/tomato_ws && echo hi"
    assert vision_remote.workspace_cd("/opt/my ws") == "cd '/opt/my ws'"
    assert "vision_run.sh --no-window" in vision_remote.START_REMOTE


def test_vision_remote_same_machine_needs_no_ssh():
    """한 대에서 다 돌리면 패널은 이 장비 — vision_run.sh 를 SSH 없이 띄운다."""
    from elite_robot_controller.tomato import vision_remote

    remote = vision_remote.VisionRemote(local_workspace="/home/nano/tomato_ws")
    cmd = remote.launch_command("127.0.0.1")
    assert cmd[:2] == ["bash", "-c"] and "ssh" not in cmd
    assert cmd[2].startswith("cd /home/nano/tomato_ws && ")
    assert remote.where("localhost") == "이 PC"
    assert remote.launch_command("192.168.1.10")[0] == "ssh"


def test_vision_remote_uses_panel_then_falls_back_to_launch(tmp_path, monkeypatch):
    """패널(:포트) 이 떠 있으면 HTTP 로 켜고, 없으면 vision_run.sh 를 띄운다(다른 장비면 SSH)."""
    import json
    import os
    import socket
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    from elite_robot_controller.tomato import vision_remote

    calls = []

    class Panel(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, data):
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            self._json({"overall": {"ready": True, "text": "준비 완료"},
                        "components": [{"label": "비전", "state": "ready", "detail": "ok"}]})

        def do_POST(self):
            calls.append(self.path)
            self._json({"success": True, "message": "켰습니다"})

    server = ThreadingHTTPServer(("127.0.0.1", 0), Panel)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        remote = vision_remote.VisionRemote(panel_port=server.server_address[1])
        assert remote.start("127.0.0.1")["success"] and calls == ["/all/start"]
        assert remote.stop("127.0.0.1")["success"] and calls[-1] == "/all/stop"
        summary = vision_remote.summarize(remote.panel_status("127.0.0.1"))
        assert summary["ready"] and summary["components"][0]["label"] == "비전"
    finally:
        server.shutdown()

    # 패널이 없는 다른 장비(127.0.0.2 — 같은 PC 로 보지 않는 루프백) -> 가짜 ssh 가 불린다.
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        free_port = probe.getsockname()[1]
    log = tmp_path / "ssh_args"
    fake = tmp_path / "ssh"
    fake.write_text(f'#!/bin/sh\necho "$@" > {log}\necho started\n')
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{tmp_path}{os.pathsep}{os.environ['PATH']}")
    remote = vision_remote.VisionRemote(user="nano", panel_port=free_port)
    assert remote.panel_status("127.0.0.2") is None
    assert not vision_remote.summarize(None)["reachable"]
    result = remote.start("127.0.0.2")
    assert result["success"], result
    assert "nano@127.0.0.2" in log.read_text() and "vision_run.sh" in log.read_text()

    # 키가 등록되지 않았으면 그 이유와 고칠 명령을 알려 준다.
    fake.write_text('#!/bin/sh\necho "nano@x: Permission denied (publickey,password)." >&2\nexit 255\n')
    result = remote.start("127.0.0.2")
    assert not result["success"] and "ssh-copy-id nano@127.0.0.2" in result["message"]
    assert not remote.stop("127.0.0.2")["success"]     # 패널이 없으면 끌 것도 없다


def test_vision_panel_ignores_its_own_parents():
    """한 대에서 다 돌리면 부모 ros2 launch 의 명령줄에 비전 런치 이름이 들어 있다 —
    그것을 "패널 밖에서 떠 있는 비전 노드" 로 잘못 보면 [비전 시작] 이 아무것도 안 켠다."""
    import os

    from elite_robot_controller.tomato.vision_panel import VisionPanelNode

    mine = VisionPanelNode._ancestors()
    assert os.getpid() in mine and os.getppid() in mine


def test_descend_limit_is_taught_as_a_pose(tmp_path):
    """하강 한계는 숫자가 아니라 가르친 자세 — 그 베이스 Z 가 한계다(250 mm 거리 한도 아님)."""
    from elite_robot_controller.tomato import teach_store

    path = str(tmp_path / "taught.yaml")
    teach_store.save(path, {"descend_limit_pose_mm_deg": [450.0, -20.0, 182.5, 180.0, 0.0, 0.0]})
    cfg = pick_config.PickConfig()
    assert cfg.descend_limit_z is None
    assert "descend_limit_pose_mm_deg" in teach_store.apply(cfg, teach_store.load(path))
    assert math.isclose(cfg.descend_limit_z, 0.1825, abs_tol=1e-9)
    assert not any(gap.startswith("descend_limit") for gap in cfg.missing())
    # 거리 한도는 이제 기본으로 꺼져 있다.
    assert pick_config.DescendConfig().max_travel_mm == 0.0
    assert dict(pick_config.PARAMETER_DEFAULTS)["descend.max_travel_mm"] == 0.0


def test_moves_below_descend_limit_are_refused_before_moving():
    """4·6·7 단계 목표가 한계 Z 아래면 움직이지 않고 멈춘다. 움직이는 중에도 감시한다."""
    from elite_robot_controller.tomato import plan
    from elite_robot_controller.tomato.pick_sequence_node import PickSequenceNode

    assert plan.below_floor([0.4, 0.0, 0.1799, 0, 0, 0], 0.18)
    assert not plan.below_floor([0.4, 0.0, 0.18, 0, 0, 0], 0.18)        # 딱 한계까지는 된다
    assert not plan.below_floor([0.4, 0.0, 0.1785, 0, 0, 0], 0.18, 0.002)  # 감시는 2 mm 여유
    assert not plan.below_floor([0.4, 0.0, -1.0, 0, 0, 0], None)        # 한계 없음

    class Fake:
        FLOOR_STATES = PickSequenceNode.FLOOR_STATES

        def __init__(self, state="DESCEND", tcp_z=0.30):
            self.cfg = pick_config.PickConfig(
                descend_limit_pose=pick_config.pose_to_si([400, 0, 200, 180, 0, 0]))
            self.state = state
            self.tcp_pose = [0.4, 0.0, tcp_z, 3.14159, 0.0, 0.0]
            self.failed = None

        def fail(self, reason):
            self.failed = reason

    node = Fake()
    assert PickSequenceNode._floor_ok(node, [0.4, 0.0, 0.25, 0, 0, 0], "7. 하강")
    assert node.failed is None
    assert not PickSequenceNode._floor_ok(node, [0.4, 0.0, 0.19, 0, 0, 0], "7. 하강")
    assert "하강 한계 Z 200.0 mm" in node.failed and "190.0" in node.failed

    # 움직이는 중: 하강 단계에서 실제 TCP 가 한계보다 2 mm 넘게 아래면 세운다.
    assert PickSequenceNode._watch_floor(Fake(tcp_z=0.199))
    node = Fake(tcp_z=0.197)
    assert not PickSequenceNode._watch_floor(node) and "아래로 내려가 멈췄습니다" in node.failed
    # 하강과 상관없는 단계(홈 이동 등)는 이 감시를 받지 않는다.
    assert PickSequenceNode._watch_floor(Fake(state="HOME_AFTER_PICK", tcp_z=0.05))


def test_sequence_has_no_return_to_wait_step():
    """대기 자세 복귀 단계를 뺐다 — 마지막 홈(16) 다음이 바로 작업 끝(17)이다."""
    from elite_robot_controller.tomato.pick_sequence_node import STEP_ORDER, PickSequenceNode

    names = [name for name, _ in STEP_ORDER]
    assert "RETURN_WAIT" not in names
    assert names[-2:] == ["HOME_AFTER_PLACE", "HOME_END"]
    assert dict(STEP_ORDER)["HOME_END"].startswith("17.")
    assert not hasattr(PickSequenceNode, "enter_return_wait")


def test_gripper_values_and_waits_come_from_the_ui(tmp_path):
    """작업 UI 에서 저장한 닫기·열기 위치와 대기 시간이 설정 위에 덮어써진다."""
    from elite_robot_controller.tomato import teach_store

    path = str(tmp_path / "taught.yaml")
    teach_store.save(path, {"gripper_close_value": 6500, "gripper_open_value": 100,
                            "gripper_close_wait_s": 2.5, "gripper_open_wait_s": 0.8})
    cfg = pick_config.PickConfig()
    applied = teach_store.apply(cfg, teach_store.load(path))
    assert {"gripper_close_value", "gripper_open_wait_s"} <= set(applied)
    g = cfg.gripper
    assert (g.close_value, g.open_value) == (6500, 100)
    assert g.wait_s(closing=True) == 2.5 and g.wait_s(closing=False) == 0.8
    # 따로 정하지 않았으면(음수) 예전 action_wait_s 를 쓴다.
    assert pick_config.GripperConfig(action_wait_s=1.2).wait_s(True) == 1.2
    assert pick_config.GripperConfig().max_position == 8000


def test_gripper_recovery_restores_lost_turns():
    """재부팅하면 모터는 몇 바퀴째였는지 잊는다 — 보정 없이 6000 을 다시 보내면 한 바퀴 더 조인다."""
    from elite_robot_controller.tomato import gripper_dynamixel_node as g

    assert g.turns_lost(6000, 6000 - 4096) == 1 and g.turns_lost(300, 300) == 0
    assert g.same_but_turns(6000, 1904) and not g.same_but_turns(6000, 900)

    class Motor:
        """확장 위치 모드 흉내: 재부팅하면 위치를 한 바퀴 안(0~4095)으로 다시 센다."""
        def __init__(self, position):
            self.position, self.goal, self.error, self.writes = position, position, 0, []

        def reboot(self, port, dxl_id):
            self.position %= 4096
            self.error = 0
            return 0, 0

        def read1ByteTxRx(self, port, dxl_id, addr):
            return (self.error if addr == g.ADDR_HARDWARE_ERROR_STATUS else 4), 0, 0

        def read4ByteTxRx(self, port, dxl_id, addr):
            return self.position & 0xFFFFFFFF, 0, 0

        def write1ByteTxRx(self, port, dxl_id, addr, value):
            return 0, 0

        def write4ByteTxRx(self, port, dxl_id, addr, value):
            self.writes.append((addr, value))
            if addr == g.ADDR_GOAL_POSITION:
                self.goal = value - (1 << 32) if value > 0x7FFFFFFF else value
            return 0, 0

        def getTxRxResult(self, result):
            return "ok"

    class Log:
        def __getattr__(self, name):
            return lambda *a, **k: None

    class Param:
        def __init__(self, value):
            self.value = value

    node = object.__new__(g.GripperDynamixelNode)
    params = {"profile_velocity": 150, "profile_acceleration": 50,
              "min_position": 0, "max_position": 8000}
    node.get_parameter = lambda name: Param(params[name])
    node.get_logger = lambda: Log()
    node.packet, node.port, node.dxl_id = Motor(6000), object(), 1
    node.turn_offset, node.last_present, node.last_goal, node.recovery_count = 0, 6000, 6000, 0

    import time as _time
    sleep, _time.sleep = _time.sleep, (lambda s: None)
    try:
        node._recover()
    finally:
        _time.sleep = sleep
    # 모터는 1904 로 다시 세지만, 보정해서 같은 물리 위치(모터 값 1904)를 목표로 보낸다.
    assert node.turn_offset == 4096
    assert node.packet.goal == 6000 - 4096
    # 재부팅 뒤 지워지는 프로파일 속도·가속도도 다시 썼다.
    addrs = [a for a, _ in node.packet.writes]
    assert g.ADDR_PROFILE_VELOCITY in addrs and g.ADDR_PROFILE_ACCELERATION in addrs

    # 새 명령도 보정된 값으로 간다(8000 -> 모터 값 3904), 범위 밖(8001)은 버린다.
    class Msg:
        def __init__(self, data):
            self.data = data
    node.cb_set_position(Msg(8000))
    assert node.packet.goal == 8000 - 4096
    node.cb_set_position(Msg(8001))
    assert node.packet.goal == 8000 - 4096


def test_weight_job_needs_only_home_and_wait():
    """무게 추정은 놓는 자리·하강 한계·그리퍼가 없어도 시작할 수 있다."""
    from elite_robot_controller.tomato.pick_sequence_node import JOB_ORDERS

    cfg = pick_config.PickConfig(
        home_joint=[0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
        pick_wait_pose=pick_config.pose_to_si([400, 0, 400, 180, 0, 0]))
    assert cfg.missing("weight") == []
    assert "place_point_pose" in cfg.missing("pick")
    names = [n for n, _ in JOB_ORDERS["weight"]]
    assert names[:5] == ["HOME_START", "MOVE_WAIT", "TRIGGER", "WAIT_DETECT", "CENTER"]
    assert "WEIGHT_RESULT" in names and "DESCEND" not in names
    # 무게는 중심 맞춘 뒤의 재측정 값을 그대로 — 무게 추정 촬영(/tomato/capture)은 기본에서 뺐다.
    assert "WEIGH_TRIGGER" not in names
    assert pick_config.VisionConfig().weight_capture == "recapture"
    assert pick_config.VisionConfig().configured


def test_weight_result_uses_tomato_at_image_center():
    """중심을 맞춘 뒤 무게 추정 촬영에서 화면 가운데에 가장 가까운 토마토의 무게를 보여 준다."""
    from elite_robot_controller.tomato.pick_sequence_node import PickSequenceNode

    class Vision:
        latest_stamp = 5.0
        latest = [vision_interface.Detection(x=0.08, y=0.02, z=0.3, weight=150.0),
                  vision_interface.Detection(x=0.003, y=-0.002, z=0.29, height=58.0, weight=112.0)]

    class Log:
        def __getattr__(self, name):
            return lambda *a, **k: None

    class Fake:
        _show_weight = PickSequenceNode._show_weight

        def __init__(self):
            self.vision, self.cfg = Vision(), pick_config.PickConfig()
            self._trigger_stamp, self._entered_at = 1.0, 1.0
            self.target = vision_interface.Detection(weight=110.6)    # 중앙 재측정 결과
            self.went = None

        def now(self):
            return 2.0

        def get_logger(self):
            return Log()

        def go(self, state, message=""):
            self.went = (state, message)

    node = Fake()
    PickSequenceNode.run_weigh_wait(node)
    assert node.went[0] == "WEIGHT_RESULT" and "112.0 g" in node.went[1]
    r = node.weight_result
    assert r["found"] and r["weight_g"] == 112.0 and r["height_mm"] == 58.0
    assert r["recapture_weight_g"] == 110.6 and r["count"] == 2

    node = Fake()
    node.vision.latest = []
    PickSequenceNode.run_weigh_wait(node)
    assert node.went[0] == "WEIGHT_RESULT" and not node.weight_result["found"]


def test_vision_client_prepares_weight_capture():
    """무게 추정 촬영(/tomato/capture)은 weight_capture 가 "weight" 일 때만 준비한다.

    기본(recapture)은 재측정 결과를 그대로 쓰므로 그 토픽을 아예 쓰지 않는다.
    """
    class Log:
        def __getattr__(self, name):
            return lambda *a, **k: None

    class Node:
        def __init__(self):
            self.pubs, self.subs = [], []

        def create_publisher(self, kind, topic, depth):
            self.pubs.append(topic)
            return object()

        def create_subscription(self, kind, topic, cb, depth):
            self.subs.append(topic)

        def get_logger(self):
            return Log()

    node = Node()
    vision_interface.VisionClient(node, pick_config.VisionConfig())
    assert "/tomato/capture" not in node.pubs and "/tomato/middle_capture" in node.pubs
    assert "/tomato/estimated_weights" not in node.subs

    node = Node()
    client = vision_interface.VisionClient(node, pick_config.VisionConfig(weight_capture="weight"))
    assert "/tomato/capture" in node.pubs and "/tomato/nearest_capture" in node.pubs
    assert {"/tomato/center_positions", "/tomato/estimated_weights", "/tomato/heights"} <= set(node.subs)
    assert "weight" in client._trigger_pubs


def test_number_parameters_accept_integers_from_yaml():
    """설정 파일에 `grasp_depth_mm: 60` 처럼 .0 을 빠뜨려도 시퀀스 노드가 죽지 않는다.

    (2026-10-21 현장: 60 으로 적었다가 InvalidParameterTypeException 으로 시퀀스 노드가
    뜨지 못했고, UI 는 로봇이 '끊김' 으로 보였다.)
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        node = Node("loose_params_test", parameter_overrides=[
            Parameter("descend.grasp_depth_mm", value=60),            # 실수 자리에 정수
            Parameter("gripper.close_value", value=6000.0),           # 정수 자리에 실수
            Parameter("gripper.tcp_offset_mm", value=[-32, 100, 10]),  # 실수 목록 자리에 정수 목록
        ])
        pick_config.declare_all(node)
        cfg = pick_config.load(node)
        assert cfg.descend.grasp_depth_mm == 60.0
        assert cfg.gripper.close_value == 6000 and isinstance(cfg.gripper.close_value, int)
        assert cfg.gripper.tcp_offset_mm == [-32.0, 100.0, 10.0]
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_each_motion_has_its_own_speed(tmp_path):
    """작업 UI 에서 동작마다 정한 속도(mm/s, °/s)가 그 동작에만 쓰이고, 나머지는 기본값."""
    import math
    from elite_robot_controller.tomato import teach_store

    cfg = pick_config.PickConfig()
    # 기본: 직선은 move_speed, 하강·상승은 descend_speed, 관절은 joint_speed
    assert cfg.motion("CENTER") == (cfg.move_accel, cfg.move_speed)
    assert cfg.motion("DESCEND")[1] == cfg.descend_speed
    assert cfg.motion("PLACE_DESCEND")[1] == cfg.place_descend_speed
    assert cfg.motion("HOME_START") == (cfg.joint_accel, cfg.joint_speed)

    path = str(tmp_path / "taught.yaml")
    teach_store.save(path, {"motion_speeds": {"DESCEND": 30.0, "HOME_START": 45.0},
                            "motion_accels": {"movel": 500.0}})
    teach_store.apply(cfg, teach_store.load(path))
    assert math.isclose(cfg.motion("DESCEND")[1], 0.030)               # 30 mm/s
    assert math.isclose(cfg.motion("HOME_START")[1], math.radians(45))  # 45 °/s
    assert math.isclose(cfg.motion("DESCEND")[0], 0.5)                 # 500 mm/s²
    assert cfg.motion("LIFT")[1] == cfg.descend_speed                  # 정하지 않은 동작은 그대로
    assert cfg.motion("MOVE_WAIT")[0] == cfg.joint_accel
    assert math.isclose(pick_config.motion_to_ui("movej", math.radians(30)), 30.0)


def test_wrist_rotates_before_gripper_alignment():
    """카메라 중심을 맞춘 뒤 6 축을 +90° 돌리고, 파지점 위치는 카메라 중심 자세 기준 오프셋 그대로."""
    import math
    from elite_robot_controller.tomato.pick_sequence_node import STEP_ORDER, PickSequenceNode

    names = [n for n, _ in STEP_ORDER]
    assert names.index("MEASURE") < names.index("ROTATE_WRIST") < names.index("ALIGN")

    class Log:
        def __getattr__(self, name):
            return lambda *a, **k: None

    class Fake:
        _plan_grip = PickSequenceNode._plan_grip
        _floor_ok = PickSequenceNode._floor_ok

        def __init__(self, wrist, j6_deg=-145.0):
            self.cfg = pick_config.PickConfig(
                wrist_rotate_deg=wrist,
                gripper=pick_config.GripperConfig(tcp_offset_mm=[-32.5, 100.0, 20.0],
                                                  offset_frame="base"))
            self.target = vision_interface.Detection(weight=110.0, height=0.0)
            self.depth_mm, self.residual_mm = 390.0, 4.8
            self.centered_pose = [0.75, -0.24, 0.64, math.pi, 0.0, math.radians(47.0)]
            self.trigger_pose = None
            self.joints = [0.0, -1.5, -1.0, -1.2, 1.57, math.radians(j6_deg)]
            self.went, self.failed = None, None

        def get_logger(self):
            return Log()

        def go(self, state, message=""):
            self.went = state

        def fail(self, reason):
            self.failed = reason

    node = Fake(90.0)
    PickSequenceNode.run_measure(node)
    assert node.went == "ROTATE_WRIST" and node.failed is None
    assert math.isclose(math.degrees(node.wrist_target[5]), -55.0)   # -145 + 90
    assert node.wrist_target[:5] == node.joints[:5]                  # 다른 축은 그대로

    # 돌린 뒤: 위치는 카메라 중심 자세 + 오프셋(베이스), 방향은 돌린 자세 그대로.
    turned = [math.pi, 0.0, math.radians(137.0)]
    assert node._plan_grip(turned)
    g = node.grip_target
    assert [round(v, 4) for v in g[:3]] == [0.7175, -0.14, 0.66]
    assert g[3:] == turned
    assert math.isclose(node.descend_target[2], 0.66 - node.travel_mm / 1000.0, abs_tol=1e-6)

    # 6 축 한도를 넘으면 움직이기 전에 멈춘다.
    node = Fake(90.0, j6_deg=300.0)
    PickSequenceNode.run_measure(node)
    assert node.went is None and "6축 한도" in node.failed

    # 0° 면 돌리지 않고 바로 파지점 정렬로.
    node = Fake(0.0)
    PickSequenceNode.run_measure(node)
    assert node.went == "ALIGN" and node.grip_target[3:] == node.centered_pose[3:]
