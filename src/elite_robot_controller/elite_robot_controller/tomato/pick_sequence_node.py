"""토마토 픽앤플레이스 시퀀스 노드 (Elite CS616).

현장에서는 토마토를 **벽면 쪽으로** 따야 해서 TCP 가 아래가 아니라 벽을
본다. 그래서 픽 쪽 이동은 모두 TCP 기준이고(`plan.py`), 장애물이 많은
홈 <-> 대기 / 홈 <-> 놓는 자리 구간은 관절 이동 + 경유점으로 간다.

    1  홈 자세 (movej)
    2  대기 자세 (movej, 가르친 관절값 — 없으면 pose 를 역기구학으로)   ★ 경유점
    3  촬영 -> 토마토 좌표 수신
    4  카메라 중심을 토마토에 맞춤 (TCP x·y)  -> 다시 촬영
    5  크기·무게 확인 (중심에서 다시 찍은 값)
    6  그리퍼 파지점 정렬 (TCP 기준 오프셋)
    7  하강 — TCP +z, 거리는 카메라 깊이로 계산 (고정 Z 한계 없음)
    8  그리퍼 오므리기
    9  상승 — TCP -z, 6 단계 자세로 = 대기 자세와 같은 선상
    10 홈 자세 (movej)                                              ★ 경유점(거꾸로)
    11 놓을 자리 위로 (접근 높이는 UI 에서 정함)                      ★ 경유점
    12 감속 하강
    13 그리퍼 열기
    14 상승
    15 홈 자세 (movej)                                              ★ 경유점(거꾸로)
    16 대기 자세 (movej) — 연속 운전이면 여기서 3 으로              ★ 경유점
    17 작업 끝 — 홈 자세 (movej)                                     ★ 경유점(거꾸로)

각 단계는 **막지 않는다** — 20 Hz 타이머가 상태를 하나씩 진행시키고, 이동
완료는 실제 TCP(직선 이동) 나 실제 관절값(관절 이동) 이 목표에 들어왔는지로
판정한다. 그래야 도중에 정지·일시정지를 받을 수 있다.

로봇 소켓은 `robot_control_node` 가 혼자 들고 있고, 이 노드는
`robot/command/script` 토픽으로 스크립트만 넘긴다.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, String
from std_srvs.srv import Trigger

from . import geometry, motion, pick_config, plan, teach_store
from .gripper_interface import GripperClient
from .vision_interface import VisionClient, camera_delta

#: 진행 순서대로. UI 가 이 순서로 목록을 그린다.
STEP_ORDER = [
    ("HOME_START", "1. 홈 자세로 이동"),
    ("MOVE_WAIT", "2. 대기 자세로 이동"),
    ("TRIGGER", "3. 촬영 요청 (최근접 / 중앙 재측정)"),
    ("WAIT_DETECT", "3. 토마토 좌표 수신 대기"),
    ("CENTER", "4. 카메라 중심을 토마토에 맞춤 (TCP 기준)"),
    ("MEASURE", "5. 크기·무게 확인"),
    ("ALIGN", "6. 그리퍼 파지점 정렬 (TCP 기준)"),
    ("DESCEND", "7. 하강 (TCP +z)"),
    ("GRIP", "8. 그리퍼 오므리기"),
    ("LIFT", "9. 상승 (TCP -z, 대기 자세 선상)"),
    ("HOME_AFTER_PICK", "10. 홈 자세로 이동"),
    ("PLACE_APPROACH", "11. 놓을 자리 위로 이동"),
    ("PLACE_DESCEND", "12. 감속 하강"),
    ("RELEASE", "13. 그리퍼 열기"),
    ("PLACE_RETREAT", "14. 상승"),
    ("HOME_AFTER_PLACE", "15. 홈 자세로 이동"),
    ("RETURN_WAIT", "16. 대기 자세로 이동"),
    ("HOME_END", "17. 작업 끝 — 홈 자세로 이동"),
]
STEPS = dict(STEP_ORDER)
STEPS.update({
    "IDLE": "대기 중",
    "DONE": "작업 완료",
    "PAUSED": "일시 정지",
    "ERROR": "오류 — 정지함",
})
IDLE_STATES = ("IDLE", "DONE", "ERROR")


class PickSequenceNode(Node):

    #: 타이머 주기 [s]. 20 Hz 면 정지 명령에 50 ms 안에 반응한다.
    TICK_S = 0.05

    def __init__(self):
        super().__init__("tomato_pick_sequence")

        pick_config.declare_all(self)
        self.declare_parameter("teach_file", teach_store.DEFAULT_PATH)
        self.teach_file = self.get_parameter("teach_file").value
        self._load_config()
        # UI 에서 놓기 높이를 바꾸면 여기로 들어온다(SetParameters).
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ---- 로봇 상태 ------------------------------------------------
        self.tcp_pose = None       # 현재 TCP pose (m, rad)
        self.joints = None         # 현재 관절값 (rad)
        self.connected = False
        self.last_alarm = ""
        self.create_subscription(
            Float32MultiArray, "robot/status/tcp_pose", self._on_tcp_pose, 10)
        self.create_subscription(
            Float32MultiArray, "robot/status/joint_position", self._on_joints, 10)
        self.create_subscription(Bool, "robot/status/connected", self._on_connected, 10)
        self.create_subscription(String, "robot/status/alarms", self._on_alarm, 10)
        # 비전이 다른 장비(Orin)에 있을 때 브리지 클라이언트가 연결 상태를 알린다.
        # None 은 "브리지 없음"(비전이 같은 장비) — 이때는 감시하지 않는다.
        self.vision_link = None
        self.create_subscription(Bool, "vision/bridge/connected", self._on_vision_link, 10)

        self.pub_script = self.create_publisher(String, "robot/command/script", 10)
        self.cli_stop = self.create_client(Trigger, "robot/dashboard/stop")

        # ---- 바깥 장치 ------------------------------------------------
        self.vision = VisionClient(self, self.cfg.vision)
        self.gripper = GripperClient(self, self.cfg.gripper)

        # ---- 시퀀스 상태 ----------------------------------------------
        self.state = "IDLE"
        self.message = "대기 중"
        self.loop_mode = False
        self.cycle_count = 0
        self._entered_at = self.now()
        self._goal = None           # ("pose", pose) | ("joints", joints)
        self._reached_at = None     # 목표에 든 시각(안정화 판정)
        self._wait_until = None     # 그리퍼처럼 시간으로 기다리는 단계용
        self._timeout_s = self.cfg.move_timeout_s
        self._paused_state = None
        self._reset_cycle()

        # ---- 명령 인터페이스 ------------------------------------------
        teach = {
            "home_pose": self.cb_teach_home,
            "wait_pose": self.cb_teach_wait,
            "place_pose": self.cb_teach_place,
            "waypoint_wait_add": lambda q, r: self._waypoint("waypoints_wait_deg", "add", r),
            "waypoint_wait_undo": lambda q, r: self._waypoint("waypoints_wait_deg", "undo", r),
            "waypoint_wait_clear": lambda q, r: self._waypoint("waypoints_wait_deg", "clear", r),
            "waypoint_place_add": lambda q, r: self._waypoint("waypoints_place_deg", "add", r),
            "waypoint_place_undo": lambda q, r: self._waypoint("waypoints_place_deg", "undo", r),
            "waypoint_place_clear": lambda q, r: self._waypoint("waypoints_place_deg", "clear", r),
            "clear": self.cb_teach_clear,
        }
        for name, callback in teach.items():
            self.create_service(Trigger, f"pick/teach/{name}", callback)

        self.create_service(Trigger, "pick/start", self.cb_start)
        self.create_service(Trigger, "pick/start_loop", self.cb_start_loop)
        self.create_service(Trigger, "pick/stop", self.cb_stop)
        self.create_service(Trigger, "pick/pause", self.cb_pause)
        self.create_service(Trigger, "pick/resume", self.cb_resume)
        self.pub_status = self.create_publisher(String, "pick/status", 10)
        self.create_subscription(Empty, "pick/trigger", self.cb_trigger_topic, 10)
        self.create_subscription(String, "pick/command", self.cb_command_topic, 10)
        # 한 사이클이 끝나면(성공이든 실패든) 결과를 알린다.
        self.pub_result = self.create_publisher(String, "pick/result", 10)
        self.pub_done = self.create_publisher(Bool, "pick/done", 10)

        self.create_timer(self.TICK_S, self.tick)
        self.create_timer(0.05, self.vision.poll)
        self.create_timer(0.2, self.publish_status)

        gaps = self.cfg.missing()
        if gaps:
            self.get_logger().warn("[설정] 아직 비어 있는 값: " + ", ".join(gaps))
        self.get_logger().info(
            f"[pick] 시퀀스 노드 시작 (회전 표기 {geometry.ORIENTATION}, "
            f"정렬 {self.cfg.vision.align_frame} 기준, 하강 {self.cfg.descend.mode}, "
            f"촬영 {self.cfg.vision.first_capture} -> {self.cfg.vision.recapture})")

    # ------------------------------------------------------------ 설정
    def _load_config(self):
        """설정 파일 값 위에 가르친 값을 덮어쓴다."""
        self.cfg = pick_config.load(self)
        geometry.set_orientation_format(self.cfg.orientation_format)
        self.taught = teach_store.load(self.teach_file)
        applied = teach_store.apply(self.cfg, self.taught)
        if applied:
            self.get_logger().info(
                f"[teach] 가르친 값 적용({teach_store.resolve(self.teach_file)}): "
                + ", ".join(applied))

    def _on_set_parameters(self, params):
        """놓기 접근 높이·남는 거리를 UI 에서 바꾼다. 검사해서 파일에도 남긴다."""
        updates = {}
        for param in params:
            if param.name in ("place_approach_mm", "place_release_gap_mm"):
                updates[param.name] = float(param.value)
        if not updates:
            return SetParametersResult(successful=True)
        if self.state not in IDLE_STATES:
            return SetParametersResult(successful=False, reason="작업 중에는 바꿀 수 없습니다.")
        approach = updates.get("place_approach_mm", self.cfg.place_approach_height * 1000.0)
        gap = updates.get("place_release_gap_mm", self.cfg.place_release_gap * 1000.0)
        if not 5.0 <= approach <= 500.0:
            return SetParametersResult(successful=False,
                                       reason="접근 높이는 5~500 mm 사이여야 합니다.")
        if not 0.0 <= gap < approach:
            return SetParametersResult(successful=False,
                                       reason="여는 높이는 0 이상, 접근 높이보다 작아야 합니다.")
        self.taught["place_approach_mm"] = round(approach, 1)
        self.taught["place_release_gap_mm"] = round(gap, 1)
        try:
            teach_store.save(self.teach_file, self.taught)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=f"저장 실패: {exc}")
        teach_store.apply(self.cfg, self.taught)
        self.get_logger().info(f"[settings] 놓기 접근 {approach:.1f} mm, 여는 높이 {gap:.1f} mm")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 수신
    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_tcp_pose(self, msg):
        """상태 토픽은 mm 와 mrad 로 온다. 스크립트 단위(m, rad)로 바꿔 둔다."""
        if len(msg.data) == 6:
            self.tcp_pose = [v / 1000.0 for v in msg.data]

    def _on_joints(self, msg):
        """관절값도 mrad 로 온다(Modbus 73~78, 실장비 확인)."""
        if len(msg.data) == 6:
            self.joints = [v / 1000.0 for v in msg.data]

    def _on_connected(self, msg):
        self.connected = bool(msg.data)

    def _on_vision_link(self, msg):
        was = self.vision_link
        self.vision_link = bool(msg.data)
        if was and not self.vision_link and self.state not in IDLE_STATES + ("PAUSED",):
            # 촬영 결과도, 그리퍼 명령도 못 오간다. 멈춘다.
            self.fail("비전 PC(Orin) 연결이 끊겼습니다.")

    def _on_alarm(self, msg):
        self.last_alarm = msg.data
        if self.state not in IDLE_STATES:
            self.fail(f"로봇 알람: {msg.data}")

    # ------------------------------------------------------------ 보내기
    def send_script(self, script: str):
        self.pub_script.publish(String(data=script))
        self.get_logger().info("[script] " + script.replace("\n", " | "))

    def robot_stop(self):
        """돌고 있는 동작을 즉시 세운다(29999 stop)."""
        if self.cli_stop.service_is_ready():
            self.cli_stop.call_async(Trigger.Request())
        else:
            self.get_logger().warn("[stop] robot/dashboard/stop 서비스를 찾지 못했습니다.")

    # ------------------------------------------------------- 상태 전이
    def go(self, state: str, message: str = ""):
        self.state = state
        self.message = message or STEPS.get(state, state)
        self._entered_at = self.now()
        self._goal = None
        self._reached_at = None
        self._wait_until = None
        self._timeout_s = self.cfg.move_timeout_s
        self.get_logger().info(f"[state] {state} — {self.message}")
        enter = getattr(self, f"enter_{state.lower()}", None)
        if enter:
            enter()

    def fail(self, reason: str):
        self.get_logger().error(f"[pick] {reason}")
        self.robot_stop()
        self.state = "ERROR"
        self.message = reason
        self._goal = None
        self.loop_mode = False
        self.publish_result(False)

    def publish_result(self, success: bool):
        """한 사이클의 결과를 알린다. 바깥 공정이 이걸 보고 이어받는다."""
        payload = {
            "success": success,
            "message": self.message,
            "cycle": self.cycle_count,
            "weight_g": round(self.target.weight, 1) if self.target else None,
            "height_mm": round(self.target.height, 1) if self.target else None,
        }
        self.pub_result.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.pub_done.publish(Bool(data=success))

    # ------------------------------------------------------- 이동 명령
    def move_linear(self, pose, speed=None):
        """직선 이동. 도착은 실제 TCP 로 판정한다."""
        speed = self.cfg.move_speed if speed is None else speed
        self.send_script(motion.movel(pose, self.cfg.move_accel, speed))
        self._goal = ("pose", list(pose))
        self._reached_at = None

    def move_path(self, name, waypoints, joints=None, pose=None):
        """경유점을 거쳐 관절 이동. 목표가 관절값이면 관절로, 자세면 TCP 로 판정한다."""
        self.send_script(motion.joint_path_program(
            name, waypoints, target_joints=joints, target_pose=pose,
            accel=self.cfg.joint_accel, speed=self.cfg.joint_speed,
            blend=self.cfg.waypoint_blend))
        self._goal = ("joints", list(joints)) if joints is not None else ("pose", list(pose))
        self._reached_at = None
        # 경유점이 많으면 그만큼 오래 걸린다.
        self._timeout_s = self.cfg.move_timeout_s * (1 + len(waypoints))

    # ★ 경유점 — 여기서 구간마다 어느 목록을 어느 방향으로 거칠지 정한다.
    #   홈 -> 대기     : waypoints_wait  (가르친 순서)
    #   대기 쪽 -> 홈  : waypoints_wait  (거꾸로)
    #   홈 -> 놓는 자리 : waypoints_place (가르친 순서)
    #   놓는 쪽 -> 홈  : waypoints_place (거꾸로)
    #   UI 의 [경유점 추가] 로 펜던트 자세를 쌓으면 된다(코드 수정 불필요).
    def go_home_via(self, waypoints_out):
        self.move_path("tomato_to_home", list(reversed(waypoints_out)),
                       joints=self.cfg.home_joint)

    def go_wait_from_home(self):
        if self.cfg.pick_wait_joint:
            self.move_path("tomato_to_wait", self.cfg.waypoints_wait,
                           joints=self.cfg.pick_wait_joint)
        else:
            # 관절값을 안 가르쳤으면 pose 를 로봇이 역기구학으로 푼다.
            self.move_path("tomato_to_wait", self.cfg.waypoints_wait,
                           pose=self.cfg.pick_wait_pose)

    # -------------------------------------------------- 이동 완료 판정
    def goal_reached(self) -> bool:
        """목표 안에 `settle_time_s` 동안 머물렀으면 도착으로 본다."""
        if self._goal is None:
            return True
        kind, target = self._goal
        if kind == "joints":
            if self.joints is None:
                return False
            near = geometry.joint_diff_deg(self.joints, target) <= self.cfg.joint_tolerance_deg
        else:
            if self.tcp_pose is None:
                return False
            near = (geometry.distance_mm(self.tcp_pose, target) <= self.cfg.pose_tolerance_mm
                    and geometry.angle_diff_deg(self.tcp_pose, target)
                    <= self.cfg.angle_tolerance_deg)
        if not near:
            self._reached_at = None
            return False
        if self._reached_at is None:
            self._reached_at = self.now()
        return (self.now() - self._reached_at) >= self.cfg.settle_time_s

    def move_timed_out(self) -> bool:
        return (self.now() - self._entered_at) > self._timeout_s

    def _await_move(self, next_state: str):
        """이동이 끝나면 다음 상태로. 시간이 지나면 실패로 본다."""
        if self.goal_reached():
            self.go(next_state)
        elif self.move_timed_out():
            self.fail(f"{STEPS.get(self.state, self.state)}: 이동이 제한 시간 안에 끝나지 "
                      "않았습니다 (목표에 닿지 못함 — 경로·회전 표기 확인).")

    # ------------------------------------------------------------ 진행
    def tick(self):
        if self.state in IDLE_STATES + ("PAUSED",):
            return
        if not self.connected:
            self.fail("로봇 연결이 끊겼습니다.")
            return
        handler = getattr(self, f"run_{self.state.lower()}", None)
        if handler is None:
            self.fail(f"처리기가 없는 상태입니다: {self.state}")
            return
        handler()

    # ---- 1. 홈 -------------------------------------------------------
    def enter_home_start(self):
        # 어디서 시작할지 모르므로 경유점 없이 바로 홈으로.
        self.move_path("tomato_home", [], joints=self.cfg.home_joint)

    def run_home_start(self):
        self._await_move("MOVE_WAIT")

    # ---- 2. 대기 자세 ------------------------------------------------
    def enter_move_wait(self):
        self.go_wait_from_home()

    def run_move_wait(self):
        self._await_move("TRIGGER")

    # ---- 3. 촬영 -----------------------------------------------------
    def enter_trigger(self):
        self.vision.clear()
        # 찍은 순간의 TCP 를 기억한다 — 카메라가 툴에 붙어 있어 이 자세가
        # 중심 맞추기의 기준이다.
        self.trigger_pose = list(self.tcp_pose) if self.tcp_pose else list(self.cfg.pick_wait_pose)
        self._trigger_stamp = self.now()
        # 대기 자세의 첫 촬영은 최근접, 카메라 중심을 맞춘 뒤에는 중앙 재측정.
        self.capture_mode = (self.cfg.vision.first_capture if self._captures == 0
                             else self.cfg.vision.recapture)
        ok, note = self.vision.trigger(self.capture_mode)
        self.get_logger().info(note)
        if not ok:
            self.fail(note)

    def run_trigger(self):
        self.go("WAIT_DETECT")

    def run_wait_detect(self):
        """이번 촬영으로 새로 들어온 결과만 받는다."""
        if self.vision.latest_stamp <= self._trigger_stamp:
            if (self.now() - self._entered_at) > self.cfg.vision.detection_timeout_s:
                self.fail("비전 검출 결과를 제한 시간 안에 받지 못했습니다.")
            return
        self._captures += 1
        if not self.vision.latest:
            if self._captures > 1:
                # 중심을 맞춘 뒤 다시 찍었는데 없다 — 화면 가운데 칸 밖으로 벗어났거나
                # 놓쳤다. 엉뚱한 토마토를 집지 않게 멈춘다.
                self.fail("중심을 맞춘 뒤 재측정에서 토마토를 찾지 못했습니다 "
                          "(화면 가운데 칸 밖이거나 놓침) — camera_axis_map 을 확인하세요.")
                return
            # 대기 자세에서 토마토가 없으면 오류가 아니라 작업 끝이다
            # (연속 운전의 정상 종료).
            self.finish_reason = (f"토마토 {self.cycle_count}개를 옮기고 더 없어 마쳤습니다."
                                  if self.cycle_count else "검출된 토마토가 없습니다.")
            self.go("HOME_END")
            return

        if self.cfg.vision.detection_frame != "camera_delta":
            self.fail("이 시퀀스는 vision.detection_frame = camera_delta 만 지원합니다.")
            return
        # 좌표 목록의 **첫 번째** 를 쓴다.
        self.target = self.vision.latest[0]
        delta = camera_delta(self.target, self.cfg.vision.camera_axis_map)
        residual = plan.residual_mm(delta)
        self.residual_mm = residual
        self.depth_mm = self.target.z * 1000.0
        self.get_logger().info(
            f"[detect] {self._captures}번째 촬영({self.capture_mode}) — 중심 오차 {residual:.1f} mm, "
            f"깊이 {self.depth_mm:.1f} mm, 무게 {self.target.weight:.1f} g")

        if residual > self.cfg.vision.align_max_step_mm:
            self.fail(f"카메라 중심에서 {residual:.0f} mm 떨어져 있어 한 번에 옮기지 않습니다 "
                      f"(한도 {self.cfg.vision.align_max_step_mm:.0f} mm).")
            return
        # 두 번째 촬영부터: 제대로 옮겼다면 오차가 확 줄어야 한다. 그대로거나
        # 커졌으면 카메라 축 대응이 틀린 것 — 계속 가면 엉뚱한 쪽으로 간다.
        if (self._captures >= 2 and self._prev_residual > 15.0
                and residual > self.cfg.vision.axis_check_ratio * self._prev_residual):
            self.fail(f"중심을 맞췄는데 오차가 {self._prev_residual:.0f} → {residual:.0f} mm 입니다 "
                      "— vision.camera_axis_map(카메라 -> TCP 축 대응) 을 확인하세요.")
            return
        self._prev_residual = residual

        if self._captures == 1 or (residual > self.cfg.vision.align_tolerance_mm
                                   and self._align_left > 0):
            self.center_target = plan.center_pose(self.trigger_pose, delta,
                                                  self.cfg.vision.align_frame)
            self.go("CENTER", "" if self._captures == 1 else "4. 남은 오차만큼 다시 맞춤")
        else:
            self.go("MEASURE")

    # ---- 4. 카메라 중심 맞추기 ---------------------------------------
    def enter_center(self):
        self.move_linear(self.center_target)

    def run_center(self):
        """맞춘 뒤 한 번 더 찍는다(README: 중심에서 다시 찍은 무게가 정확하다)."""
        if not self.goal_reached():
            if self.move_timed_out():
                self.fail("카메라 중심 맞추기 이동이 제한 시간 안에 끝나지 않았습니다.")
            return
        self.centered_pose = list(self.center_target)
        if self._align_left > 0:
            self._align_left -= 1
            self.go("TRIGGER", "3. 중심에서 다시 촬영")
        else:
            self.go("MEASURE")

    # ---- 5. 크기·무게 ------------------------------------------------
    def run_measure(self):
        self.get_logger().info(
            f"[measure] 무게 {self.target.weight:.1f} g, 높이 {self.target.height:.1f} mm, "
            f"깊이 {self.depth_mm:.1f} mm (중심 오차 {self.residual_mm:.1f} mm)")
        # 하강 거리를 여기서 정한다(마지막 촬영의 깊이).
        travel = self.cfg.descend.travel_mm(self.depth_mm)
        if travel > self.cfg.descend.max_travel_mm:
            self.fail(f"하강 거리 {travel:.0f} mm 가 안전 한도 {self.cfg.descend.max_travel_mm:.0f} mm "
                      "를 넘습니다 — 카메라 깊이·descend 설정 확인.")
            return
        self.travel_mm = max(0.0, travel)
        base = self.centered_pose or self.trigger_pose
        self.grip_target = plan.grip_pose(base, self.cfg.gripper.tcp_offset_mm,
                                          self.cfg.gripper.offset_frame)
        self.descend_target = plan.descend_pose(self.grip_target, self.travel_mm / 1000.0)
        self.go("ALIGN")

    # ---- 6. 파지점 정렬 ----------------------------------------------
    def enter_align(self):
        self.move_linear(self.grip_target)

    def run_align(self):
        self._await_move("DESCEND")

    # ---- 7. 하강 (TCP +z) --------------------------------------------
    def enter_descend(self):
        if self.travel_mm <= 0.0:
            self.get_logger().warn("[descend] 이미 집을 거리 안입니다 — 하강을 건너뜁니다.")
            self._goal = None
            return
        self.get_logger().info(f"[descend] TCP +z 로 {self.travel_mm:.1f} mm")
        self.move_linear(self.descend_target, speed=self.cfg.descend_speed)

    def run_descend(self):
        self._await_move("GRIP")

    # ---- 8. 그리퍼 오므리기 ------------------------------------------
    def enter_grip(self):
        self._gripper_action(self.gripper.close)

    def run_grip(self):
        if self._wait_until and self.now() >= self._wait_until:
            self.go("LIFT")

    # ---- 9. 상승 (TCP -z) --------------------------------------------
    def enter_lift(self):
        # 6 단계 자세로 되돌아간다 — TCP 축을 따라 들어간 만큼 그대로 빠져나온다.
        self.move_linear(self.grip_target, speed=self.cfg.descend_speed)

    def run_lift(self):
        self._await_move("HOME_AFTER_PICK")

    # ---- 10. 홈 ------------------------------------------------------
    def enter_home_after_pick(self):
        self.go_home_via(self.cfg.waypoints_wait)

    def run_home_after_pick(self):
        self._await_move("PLACE_APPROACH")

    # ---- 11~14. 놓기 (베이스 Z, 상자 위) ------------------------------
    def enter_place_approach(self):
        above = plan.above(self.cfg.place_point_pose, self.cfg.place_approach_height)
        self.message = (f"11. 놓을 자리 {self.cfg.place_approach_height * 1000:.0f} mm 위로 이동")
        self.move_path("tomato_to_place", self.cfg.waypoints_place, pose=above)

    def run_place_approach(self):
        self._await_move("PLACE_DESCEND")

    def enter_place_descend(self):
        pose = plan.above(self.cfg.place_point_pose, self.cfg.place_release_gap)
        self.message = (f"12. 감속 하강 ({self.cfg.place_release_gap * 1000:.0f} mm 남을 때까지)")
        self.move_linear(pose, speed=self.cfg.place_descend_speed)

    def run_place_descend(self):
        self._await_move("RELEASE")

    def enter_release(self):
        self._gripper_action(self.gripper.open)

    def run_release(self):
        if self._wait_until and self.now() >= self._wait_until:
            self.go("PLACE_RETREAT")

    def enter_place_retreat(self):
        self.move_linear(plan.above(self.cfg.place_point_pose, self.cfg.place_approach_height),
                         speed=self.cfg.descend_speed)

    def run_place_retreat(self):
        self._await_move("HOME_AFTER_PLACE")

    # ---- 15. 홈 ------------------------------------------------------
    def enter_home_after_place(self):
        self.go_home_via(self.cfg.waypoints_place)

    def run_home_after_place(self):
        self._await_move("RETURN_WAIT")

    # ---- 16. 대기 자세 -----------------------------------------------
    def enter_return_wait(self):
        self.go_wait_from_home()

    def run_return_wait(self):
        if not self.goal_reached():
            if self.move_timed_out():
                self.fail("대기 자세 복귀가 제한 시간 안에 끝나지 않았습니다.")
            return
        self.cycle_count += 1
        self.finish_reason = f"토마토 {self.cycle_count}개를 옮겼습니다."
        if self.loop_mode:
            self.publish_result(True)
            self._reset_cycle()
            self.go("TRIGGER", "연속 운전 — 다음 토마토")
        else:
            self.go("HOME_END")

    # ---- 17. 작업 끝 -------------------------------------------------
    def enter_home_end(self):
        self.go_home_via(self.cfg.waypoints_wait)

    def run_home_end(self):
        if not self.goal_reached():
            if self.move_timed_out():
                self.fail("작업 끝 홈 복귀가 제한 시간 안에 끝나지 않았습니다.")
            return
        # ★ 완료 신호 — 작업이 끝났음을 바깥(차량·상위 시스템)에 알릴 자리.
        #   지금은 pick/result(JSON) 와 pick/done(Bool) 만 낸다. 신호가 정해지면
        #   여기에 붙인다(예: Modbus 레지스터, 디지털 출력, MQTT).
        self.go("DONE", self.finish_reason or "작업 완료")
        self.publish_result(self.cycle_count > 0)

    # ------------------------------------------------------------ 공통
    def _gripper_action(self, action):
        ok, note = action()
        self.get_logger().info(note)
        if not ok:
            self.fail(note)
            return
        self._wait_until = self.now() + self.cfg.gripper.action_wait_s

    def _reset_cycle(self):
        """한 사이클에서만 쓰는 값들을 초기화한다."""
        self.target = None
        self.trigger_pose = None
        self.center_target = None
        self.centered_pose = None
        self.grip_target = None
        self.descend_target = None
        self.travel_mm = None
        self.depth_mm = None
        self.residual_mm = None
        self._prev_residual = 0.0
        self._captures = 0
        self._align_left = max(0, int(self.cfg.vision.align_iterations))
        self._trigger_stamp = 0.0
        self.capture_mode = None
        self.finish_reason = ""

    # -------------------------------------------------------- 명령 처리
    def _can_start(self) -> tuple:
        if not self.connected:
            return False, "로봇에 연결되어 있지 않습니다."
        if self.state not in IDLE_STATES:
            return False, f"이미 작업 중입니다({self.state})."
        gaps = self.cfg.missing()
        if gaps:
            return False, "아직 정하지 않은 값이 있습니다: " + ", ".join(gaps)
        if self.vision_link is False:
            return False, "비전 PC(Orin) 에 연결되어 있지 않습니다."
        return True, ""

    def _start(self, loop: bool, res):
        ok, why = self._can_start()
        if not ok:
            res.success = False
            res.message = f"[pick] 시작할 수 없습니다 — {why}"
            self.get_logger().warn(res.message)
            return res
        self.loop_mode = loop
        self.cycle_count = 0
        self._reset_cycle()
        self.go("HOME_START")
        res.success = True
        res.message = "[pick] 연속 작업을 시작합니다." if loop else "[pick] 작업을 시작합니다."
        return res

    def cb_start(self, req, res):
        return self._start(False, res)

    def cb_start_loop(self, req, res):
        return self._start(True, res)

    def cb_trigger_topic(self, msg):
        """`pick/trigger` (Empty) 한 번이면 한 사이클을 돈다."""
        res = Trigger.Response()
        self._start(False, res)
        self.get_logger().info(f"[trigger] {res.message}")

    def cb_command_topic(self, msg):
        """`pick/command` (String): start / loop / stop / pause / resume."""
        command = str(msg.data).strip().lower()
        res = Trigger.Response()
        handlers = {
            "start": lambda: self._start(False, res), "single": lambda: self._start(False, res),
            "loop": lambda: self._start(True, res), "start_loop": lambda: self._start(True, res),
            "stop": lambda: self.cb_stop(None, res), "pause": lambda: self.cb_pause(None, res),
            "resume": lambda: self.cb_resume(None, res),
        }
        if command not in handlers:
            self.get_logger().warn(f"[command] 모르는 명령입니다: {command!r}")
            return
        handlers[command]()
        self.get_logger().info(f"[command] {command} -> {res.message}")

    def cb_stop(self, req, res):
        self.loop_mode = False
        self.robot_stop()
        self.go("IDLE", "작업자 정지")
        res.success = True
        res.message = "[pick] 작업을 정지했습니다."
        return res

    def cb_pause(self, req, res):
        if self.state in IDLE_STATES + ("PAUSED",):
            res.success = False
            res.message = f"[pick] 일시 정지할 상태가 아닙니다({self.state})."
            return res
        self._paused_state = self.state
        self.robot_stop()
        self.state = "PAUSED"
        self.message = f"일시 정지 — {STEPS.get(self._paused_state, self._paused_state)}"
        res.success = True
        res.message = "[pick] 일시 정지했습니다."
        return res

    def cb_resume(self, req, res):
        if self.state != "PAUSED":
            res.success = False
            res.message = "[pick] 일시 정지 상태가 아닙니다."
            return res
        # 멈춘 단계를 처음부터 다시 실행한다. 목표가 모두 절대 자세라 하강
        # 도중 멈췄다 재개해도 두 배로 들어가지 않는다(plan.py 머리말).
        self.go(self._paused_state, f"재개 — {STEPS.get(self._paused_state, '')}")
        res.success = True
        res.message = "[pick] 작업을 재개했습니다."
        return res

    # -------------------------------------------------- 위치 가르치기
    def _teach_ready(self, res, need_joints=True):
        if self.state not in IDLE_STATES:
            res.success = False
            res.message = "[teach] 작업 중에는 가르칠 수 없습니다 — 정지한 뒤에 하세요."
            return False
        if self.tcp_pose is None or (need_joints and self.joints is None):
            res.success = False
            res.message = "[teach] 아직 로봇 자세·관절값을 못 읽었습니다(연결 확인)."
            return False
        return True

    def _save_taught(self, res, message):
        try:
            path = teach_store.save(self.teach_file, self.taught)
        except Exception as exc:
            res.success = False
            res.message = f"[teach] 저장 실패: {exc}"
            self.get_logger().error(res.message)
            return res
        self._load_config()
        res.success = True
        res.message = f"[teach] {message} -> {path}"
        self.get_logger().info(res.message)
        return res

    def _now_pose_deg(self):
        return [round(v, 2) for v in pick_config.pose_to_human(self.tcp_pose)]

    def _now_joints_deg(self):
        return [round(v, 3) for v in pick_config.joints_to_deg(self.joints)]

    def cb_teach_home(self, req, res):
        """지금 관절값을 홈 자세로. **읽기만 한다 — 로봇은 움직이지 않는다.**"""
        if not self._teach_ready(res):
            return res
        self.taught["home_joint_deg"] = self._now_joints_deg()
        return self._save_taught(res, f"홈 자세 저장: {self.taught['home_joint_deg']}")

    def cb_teach_wait(self, req, res):
        """대기 자세 — TCP 자세와 관절값을 같이 저장한다(movej 는 관절값으로 간다)."""
        if not self._teach_ready(res):
            return res
        self.taught["pick_wait_pose_mm_deg"] = self._now_pose_deg()
        self.taught["pick_wait_joint_deg"] = self._now_joints_deg()
        return self._save_taught(res, f"대기 자세 저장: {self.taught['pick_wait_pose_mm_deg']}")

    def cb_teach_place(self, req, res):
        if not self._teach_ready(res):
            return res
        self.taught["place_point_pose_mm_deg"] = self._now_pose_deg()
        self.taught["place_point_joint_deg"] = self._now_joints_deg()
        return self._save_taught(res, f"놓는 자리 저장: {self.taught['place_point_pose_mm_deg']}")

    def _waypoint(self, key, action, res):
        """★ 경유점 목록을 바꾼다. add 는 지금 관절값을 맨 뒤에 붙인다."""
        label = "홈↔대기" if key == "waypoints_wait_deg" else "홈↔놓는 자리"
        points = list(self.taught.get(key) or [])
        if action == "add":
            if not self._teach_ready(res):
                return res
            points.append(self._now_joints_deg())
            message = f"{label} 경유점 {len(points)}번 추가"
        else:
            if self.state not in IDLE_STATES:
                res.success = False
                res.message = "[teach] 작업 중에는 바꿀 수 없습니다."
                return res
            if action == "undo" and points:
                points.pop()
                message = f"{label} 마지막 경유점 지움 (남은 {len(points)}개)"
            elif action == "clear":
                points = []
                message = f"{label} 경유점 모두 지움"
            else:
                res.success = False
                res.message = f"[teach] {label} 경유점이 없습니다."
                return res
        self.taught[key] = points
        return self._save_taught(res, message)

    def cb_teach_clear(self, req, res):
        """가르친 값을 모두 지우고 설정 파일 값으로 되돌린다."""
        if self.state not in IDLE_STATES:
            res.success = False
            res.message = "[teach] 작업 중에는 지울 수 없습니다."
            return res
        self.taught = {}
        return self._save_taught(res, "가르친 값을 모두 지웠습니다")

    # ------------------------------------------------------------ 상태
    def publish_status(self):
        def human_pose(pose):
            return [round(v, 1) for v in pick_config.pose_to_human(pose)] if pose else None

        def human_joints(joints):
            return [round(v, 1) for v in pick_config.joints_to_deg(joints)] if joints else None

        status = {
            "state": self.state,
            "step": STEPS.get(self.state, self.state),
            "order": STEP_ORDER,
            "message": self.message,
            "connected": self.connected,
            "loop": self.loop_mode,
            "cycles": self.cycle_count,
            "tcp_mm_deg": [round(v, 2) for v in pick_config.pose_to_human(self.tcp_pose)]
                          if self.tcp_pose else None,
            "joints_deg": human_joints(self.joints),
            "target": self.target.as_dict() if self.target else None,
            "residual_mm": round(self.residual_mm, 1) if self.residual_mm is not None else None,
            "depth_mm": round(self.depth_mm, 1) if self.depth_mm is not None else None,
            "travel_mm": round(self.travel_mm, 1) if self.travel_mm is not None else None,
            "captures": self._captures,
            "capture_mode": self.capture_mode,
            "alarm": self.last_alarm,
            "vision_link": self.vision_link,
            "missing": self.cfg.missing(),
            "taught": self.taught,
            "config": {
                "home_joint": human_joints(self.cfg.home_joint),
                "pick_wait_pose": human_pose(self.cfg.pick_wait_pose)
                                  if self.cfg.wait_pose_set else None,
                "pick_wait_joint": human_joints(self.cfg.pick_wait_joint),
                "place_point_pose": human_pose(self.cfg.place_point_pose)
                                    if self.cfg.place_pose_set else None,
                "waypoints_wait": [human_joints(p) for p in self.cfg.waypoints_wait],
                "waypoints_place": [human_joints(p) for p in self.cfg.waypoints_place],
                "place_approach_mm": round(self.cfg.place_approach_height * 1000.0, 1),
                "place_release_gap_mm": round(self.cfg.place_release_gap * 1000.0, 1),
            },
        }
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def destroy_node(self):
        self.gripper.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickSequenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
