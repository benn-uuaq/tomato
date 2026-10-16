"""비전 PC(Orin) 조작 패널 — README.odt 의 수동 절차를 버튼으로 대신한다.

    ./vision_run.sh          # 이 패널을 띄우고 세 부품을 바로 켠다(빠진 것만)
    # 브라우저: http://<Orin IP>:8081   (Orin 화면에서는 http://localhost:8081)

한양대 README.odt(2026-10) 의 절차:

    ros2 launch doosan_vision_grasp tomato_vision_d435i_v2.launch.py   # 카메라+YOLO
    ros2 run doosan_vision_grasp dyn_test                              # 그리퍼
    ros2 topic pub --once /tomato/nearest_capture std_msgs/msg/Empty "{}"
    ros2 topic pub --once /set_pos std_msgs/msg/Int32 "{data: 4095}"   # 집기

| 부품 | 패널이 띄우는 것 | "준비됐다" 고 보는 조건 |
| --- | --- | --- |
| 비전 (카메라 + YOLO) | `tomato_vision_d435i_v2.launch.py` | 카메라 영상이 들어오고, 비전 노드가 `/tomato/nearest_capture` 를 구독 |
| 그리퍼 | `gripper_dynamixel_node` (에러 복구 포함) | 모터가 현재 위치를 답함 |
| 외부 통신 | `tomato_tcp_bridge` (protocol 3) | 브리지가 상태를 알림 — 로봇 PC 가 붙으면 그 주소도 |

**한양대 쪽에서 직접 켠 노드는 그대로 쓴다.** 런치를 다른 터미널에서 띄웠거나
`dyn_test` 를 켜 두었으면, 패널은 그것을 "패널 밖 — 그대로 사용" 으로 보고 같은
것을 또 띄우지 않는다(같은 카메라·시리얼 포트를 둘이 잡으면 둘 다 망가진다).
패널 밖에서 켠 것은 패널에서 끄지도 않는다 — 켠 터미널에서 Ctrl+C.

이를 위해 Orin 쪽 ROS 는 **새 터미널과 같은 기본 설정**(ROS_DOMAIN_ID 0, 모든
인터페이스)을 쓴다(scripts/env.sh). 한양대 터미널에서 켠 노드가 보여야 하기 때문이다.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Float32MultiArray, Int32, String

from .process_manager import ManagedProcess, ProcessManager

PACKAGE = "elite_robot_controller"
VISION_PACKAGE = "doosan_vision_grasp"

#: 부품마다 "패널 밖에서 떠 있는 것" 을 알아보는 패턴(명령줄).
EXTERNAL = {
    "vision": "tomato_vision_d435i|tomato_area_d435i",
    "gripper": "gripper_dynamixel_node|dyn_test",
    "bridge": "tomato_tcp_bridge",
}
#: 비전 런치가 띄우는 카메라. 카메라만 따로 떠 있으면 비전 런치가 장치를 못 잡는다.
CAMERA_PATTERN = "realsense2_camera_node"


def _share(package):
    try:
        return get_package_share_directory(package)
    except Exception:
        return None


def _wait_future(future, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if future.done():
            return True
        time.sleep(0.02)
    return False


class VisionPanelNode(Node):

    def __init__(self):
        super().__init__("tomato_vision_panel")
        self.declare_parameter("ui_host", "0.0.0.0")
        self.declare_parameter("ui_port", 8081)
        self.declare_parameter("bridge_port", int(os.environ.get("BRIDGE_PORT", "9000")))
        self.declare_parameter("gripper_close_value", 4095)
        self.declare_parameter("gripper_open_value", 0)
        #: True 면 패널이 뜨자마자 세 부품을 켠다 — 이미 떠 있는 것은 건너뛴다.
        self.declare_parameter("autostart", False)
        #: 비전 런치 파일 (한양대 v1 / v2 / v3). README.odt 기준 v2 가 기본.
        self.declare_parameter("vision_launch", os.environ.get(
            "VISION_LAUNCH", "tomato_vision_d435i_v2.launch.py"))
        elite_share = _share(PACKAGE)
        self.declare_parameter(
            "pick_config", os.path.join(elite_share, "config", "tomato_pick.yaml"))

        self.close_value = int(self.get_parameter("gripper_close_value").value)
        self.open_value = int(self.get_parameter("gripper_open_value").value)
        self.bridge_port = int(self.get_parameter("bridge_port").value)
        self._group = ReentrantCallbackGroup()

        # ---- 부품 ------------------------------------------------------
        self.manager = ProcessManager()
        self._build_components()

        # ---- 실제 신호 ---------------------------------------------------
        self.camera_info_times = []          # 최근 camera_info 수신 시각(fps 계산)
        self.gripper_position = None
        self.gripper_position_at = 0.0
        self.bridge_status = None
        self.bridge_status_at = 0.0
        self.last_result = None
        self._weights = []
        self.capture_requested_at = None
        self.capture_timeout_note = ""

        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                 self._on_camera_info, qos_profile_sensor_data,
                                 callback_group=self._group)
        self.create_subscription(Int32, "gripper/present_position", self._on_gripper,
                                 10, callback_group=self._group)
        self.create_subscription(String, "vision/bridge/server_status", self._on_bridge,
                                 10, callback_group=self._group)
        # 촬영 시험은 최근접 촬영(로봇 PC 가 대기 자세에서 쓰는 것과 같다).
        self.create_subscription(PoseArray, "/tomato/nearest_position", self._on_poses,
                                 10, callback_group=self._group)
        self.create_subscription(Float32MultiArray, "/tomato/nearest_weight",
                                 self._on_weights, 10, callback_group=self._group)
        self.pub_capture = self.create_publisher(Empty, "/tomato/nearest_capture", 10)
        self._last_overall = None
        self.create_timer(1.0, self._log_overall, callback_group=self._group)
        self.pub_gripper = self.create_publisher(Int32, "/set_pos", 10)

        # 미리보기 영상은 볼 때만 구독한다 — 1280x720 컬러를 늘 받으면 Orin 이 바쁘다.
        self._preview_sub = None
        self._preview_frame = None
        self._preview_requested_at = 0.0
        self.create_timer(1.0, self._preview_idle_check, callback_group=self._group)
        try:
            import cv2  # noqa: F401  미리보기 JPEG 만들 때만 쓴다
            import numpy  # noqa: F401
            self.preview_available = True
        except Exception:
            self.preview_available = False

        # ---- 화면 --------------------------------------------------------
        page = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "vision_panel.html")
        try:
            with open(page, encoding="utf-8") as handle:
                self._page = handle.read().encode("utf-8")
        except OSError as exc:
            self.get_logger().error(f"[panel] 화면 파일을 못 읽었습니다: {exc}")
            self._page = b"<h1>vision_panel.html not found</h1>"
        host = str(self.get_parameter("ui_host").value)
        port = int(self.get_parameter("ui_port").value)
        self._server = ThreadingHTTPServer((host, port), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name="panel-http",
                         daemon=True).start()
        self.get_logger().info(f"[panel] http://{host}:{port} 에서 비전 PC 패널을 엽니다.")

        if self.get_parameter("autostart").value:
            threading.Thread(target=self.start_all, daemon=True).start()

    # ------------------------------------------------------------ 부품 정의
    def _build_components(self):
        config = str(self.get_parameter("pick_config").value)
        launch = str(self.get_parameter("vision_launch").value)
        self.manager.add(ManagedProcess(
            "vision", "비전 (카메라 + YOLO)",
            ["ros2", "launch", VISION_PACKAGE, launch]))
        self.manager.add(ManagedProcess(
            "gripper", "그리퍼 (다이나믹셀)",
            ["ros2", "run", PACKAGE, "gripper_dynamixel_node", "--ros-args",
             "-r", "__node:=gripper_dynamixel", "--params-file", config]))
        self.manager.add(ManagedProcess(
            "bridge", "외부 통신 (로봇 PC 브리지)",
            ["ros2", "run", VISION_PACKAGE, "tomato_tcp_bridge", "--ros-args",
             "-p", f"port:={self.bridge_port}"]))

    # ------------------------------------------------------------ 신호 수신
    def _on_camera_info(self, _msg):
        now = time.monotonic()
        self.camera_info_times.append(now)
        self.camera_info_times = [t for t in self.camera_info_times if now - t < 2.0]

    def _on_gripper(self, msg):
        self.gripper_position = int(msg.data)
        self.gripper_position_at = time.monotonic()

    def _on_bridge(self, msg):
        try:
            self.bridge_status = json.loads(msg.data)
            self.bridge_status_at = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _on_poses(self, msg):
        # 무게·높이는 바로 뒤따라 온다. 표시할 때 인덱스로 합친다.
        self.last_result = {
            "at": time.time(),
            "frame": msg.header.frame_id,
            "poses": [(p.position.x, p.position.y, p.position.z) for p in msg.poses],
        }
        self.capture_requested_at = None
        self.capture_timeout_note = ""

    def _on_weights(self, msg):
        self._weights = list(msg.data)

    # ------------------------------------------------------------ 미리보기
    def _on_preview(self, msg):
        self._preview_frame = msg

    def _preview_idle_check(self):
        """5 초 동안 아무도 미리보기를 안 보면 구독을 끊는다."""
        if self._preview_sub is not None and time.monotonic() - self._preview_requested_at > 5.0:
            self.destroy_subscription(self._preview_sub)
            self._preview_sub = None
            self._preview_frame = None

    def snapshot_jpeg(self):
        """가장 최근 컬러 영상 한 장을 JPEG 로. 없으면 (None, 사유)."""
        if not self.preview_available:
            return None, "이 장비의 파이썬에 cv2/numpy 가 없어 미리보기를 못 만듭니다."
        self._preview_requested_at = time.monotonic()
        if self._preview_sub is None:
            self._preview_sub = self.create_subscription(
                Image, "/camera/camera/color/image_raw", self._on_preview,
                qos_profile_sensor_data, callback_group=self._group)
            time.sleep(0.6)               # 첫 장을 기다린다
        frame = self._preview_frame
        if frame is None:
            return None, "카메라 영상이 아직 없습니다."
        import cv2
        import numpy as np
        channels = 3 if frame.encoding in ("rgb8", "bgr8") else None
        if channels is None:
            return None, f"지원하지 않는 영상 형식입니다: {frame.encoding}"
        image = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, 3)
        if frame.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if frame.width > 640:
            scale = 640.0 / frame.width
            image = cv2.resize(image, (640, int(frame.height * scale)))
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            return None, "JPEG 변환 실패"
        return buffer.tobytes(), ""

    # ------------------------------------------------------------ 조작
    def _pids(self, pattern):
        try:
            out = subprocess.run(["pgrep", "-f", pattern], capture_output=True,
                                 text=True, timeout=3).stdout.split()
        except Exception:
            return []
        return [int(pid) for pid in out if pid.isdigit()]

    def _external_pids(self, name, pattern=None):
        """패널 밖에서 떠 있는 프로세스. 패널이 띄운 프로세스 그룹과 자기 자신은 뺀다."""
        process = self.manager.get(name)
        mine_group = process.pid if process and process.running else None
        found = []
        for pid in self._pids(pattern or EXTERNAL[name]):
            if pid == os.getpid():
                continue
            try:
                if mine_group is not None and os.getpgid(pid) == mine_group:
                    continue
                with open(f"/proc/{pid}/cmdline", "rb") as handle:
                    cmd = handle.read().replace(b"\0", b" ").decode(errors="replace")
            except (OSError, ProcessLookupError):
                continue
            # pgrep 를 부른 셸 같은 것은 빼고 실제 노드·런치만 본다.
            if "pgrep" in cmd or cmd.startswith(("bash", "/bin/bash", "sh ")):
                continue
            found.append(pid)
        return found

    def start(self, name):
        process = self.manager.get(name)
        if process is None:
            return {"success": False, "message": f"모르는 부품입니다: {name}"}
        if process.running:
            return {"success": True, "message": f"{process.label}: 이미 실행 중입니다."}
        if self._external_pids(name):
            return {"success": True,
                    "message": f"{process.label}: 패널 밖에서 이미 실행 중 — 그대로 씁니다."}
        if name == "vision" and self._external_pids(name, CAMERA_PATTERN):
            return {"success": False,
                    "message": "카메라 노드만 패널 밖에서 떠 있습니다 — 그 터미널에서 끄고 다시 "
                               "시작하세요(비전 런치가 카메라를 같이 띄운다)."}
        ok, message = process.start()
        return {"success": ok, "message": message}

    def stop(self, name):
        process = self.manager.get(name)
        if process is None:
            return {"success": False, "message": f"모르는 부품입니다: {name}"}
        if not process.running and self._external_pids(name):
            return {"success": False,
                    "message": f"{process.label}: 패널 밖에서 켠 것은 여기서 끄지 않습니다 "
                               "— 켠 터미널에서 Ctrl+C."}
        ok, message = process.stop()
        return {"success": ok, "message": message}

    def restart(self, name):
        self.stop(name)
        return self.start(name)

    def start_all(self):
        messages = [self.start(name)["message"] for name in ("vision", "gripper", "bridge")]
        return {"success": True, "message": " / ".join(messages)}

    def stop_all(self):
        self.manager.stop_all()
        return {"success": True, "message": "패널이 켠 것을 모두 중지했습니다(패널 밖에서 켠 것은 그대로)."}

    def capture(self):
        if not self._vision_ready():
            return {"success": False, "message": "비전 노드가 아직 촬영 대기 상태가 아닙니다."}
        # 카메라 영상이 없으면 비전 노드는 촬영 요청을 **조용히 무시**한다(로그에만
        # 남긴다). 그러면 결과가 영영 안 오니 여기서 미리 막는다.
        if self._camera_fps() == 0:
            return {"success": False, "message": "카메라 영상이 없어 촬영할 수 없습니다."}
        self.capture_timeout_note = ""
        self.capture_requested_at = time.monotonic()
        self.pub_capture.publish(Empty())
        return {"success": True, "message": "최근접 촬영 요청 — 약 1.5 초 측정합니다."}

    def gripper(self, position):
        if not isinstance(position, int) or not 0 <= position <= 4095:
            return {"success": False, "message": f"위치는 0~4095 정수여야 합니다: {position!r}"}
        if not self.manager.get("gripper").running and not self._external_pids("gripper"):
            return {"success": False, "message": "그리퍼 노드가 꺼져 있습니다 — 먼저 시작하세요."}
        self.pub_gripper.publish(Int32(data=position))
        return {"success": True, "message": f"그리퍼 목표 위치 {position}"}

    # ------------------------------------------------------------ 상태 판정
    def _camera_fps(self):
        now = time.monotonic()
        recent = [t for t in self.camera_info_times if now - t < 2.0]
        return len(recent) / 2.0

    def _vision_ready(self):
        """비전 노드가 최근접 촬영을 받을 수 있다(모델 2 개를 다 불러온 뒤에 구독한다)."""
        return self.count_subscribers("/tomato/nearest_capture") > 0

    def _gripper_label(self, position):
        if position is None:
            return ""
        if abs(position - self.close_value) <= 150:
            return "닫힘"
        if abs(position - self.open_value) <= 150:
            return "열림"
        return "중간"

    def _component_status(self, name):
        process = self.manager.get(name)
        now = time.monotonic()
        running = process.running
        external = (not running) and bool(self._external_pids(name))
        where = " (패널 밖)" if external else ""
        state, detail = "stopped", "꺼짐"
        if name == "vision":
            fps = self._camera_fps()
            if self.capture_requested_at is not None and now - self.capture_requested_at > 8.0:
                self.capture_requested_at = None
                self.capture_timeout_note = "마지막 촬영 요청에 결과가 없었습니다 — 로그를 보세요"
            if self._vision_ready() and fps > 0:
                if self.capture_requested_at is not None:
                    state, detail = "busy", "촬영 중..."
                elif self.capture_timeout_note:
                    state, detail = "warning", self.capture_timeout_note
                else:
                    state, detail = "ready", f"촬영 대기 중 — 카메라 {fps:.0f} fps{where}"
            elif self._vision_ready():
                state, detail = "warning", f"비전 노드는 대기 중인데 카메라 영상이 없습니다{where}"
            elif running or external:
                if process.contains("No RealSense devices were found"):
                    state, detail = "error", "카메라를 찾지 못했습니다 — USB 연결 확인"
                elif process.contains("busy"):
                    state, detail = "error", "카메라를 다른 프로그램이 쓰고 있습니다"
                elif process.contains("Traceback"):
                    state, detail = "error", "비전 노드 오류 — 로그를 보세요"
                elif running and process.uptime() > 90:
                    state, detail = "error", "켠 지 90 초가 지나도 준비되지 않았습니다 — 로그를 보세요"
                else:
                    state, detail = "starting", (f"카메라 {fps:.0f} fps, YOLO 모델 2 개 불러오는 중"
                                                 if fps > 0 else "카메라 켜는 중") + where
        elif name == "gripper":
            fresh = now - self.gripper_position_at < 2.0
            if (running or external) and fresh:
                label = self._gripper_label(self.gripper_position)
                state, detail = "ready", f"모터 응답 — 현재 위치 {self.gripper_position} ({label}){where}"
            elif external:
                # 한양대 dyn_test 는 현재 위치를 내보내지 않는다 — 떠 있는 것만 확인된다.
                state, detail = "ready", "패널 밖 그리퍼 노드(dyn_test 등) 실행 중 — 위치 피드백 없음"
            elif running:
                if process.contains("포트를 열지 못했습니다"):
                    state, detail = "error", "USB 시리얼 포트를 못 열었습니다 (/dev/ttyUSB0)"
                elif process.contains("dynamixel_sdk 가 없습니다"):
                    state, detail = "error", "dynamixel_sdk 가 설치되어 있지 않습니다"
                elif process.contains("no status packet") or process.contains("응답 없음") \
                        or process.uptime() > 5:
                    state, detail = "error", "모터가 응답하지 않습니다 — 전원·ID·보드레이트 확인"
                else:
                    state, detail = "starting", "모터 연결 중"
        elif name == "bridge":
            fresh = now - self.bridge_status_at < 3.0
            if (running or external) and fresh:
                status = self.bridge_status or {}
                clients = status.get("clients") or []
                protocol = status.get("protocol", 2)
                if protocol < 3:
                    state, detail = "warning", (f"브리지가 protocol {protocol} 입니다 — 최근접·중앙 "
                                                "촬영을 모릅니다(이 워크스페이스 것으로 켜세요)")
                elif clients:
                    hosts = ", ".join(c.split(":")[0] for c in clients)
                    state, detail = "ready", f"로봇 PC 연결됨 ({hosts}){where}"
                else:
                    state, detail = "ready", f"포트 {self.bridge_port} 에서 로봇 PC 를 기다리는 중{where}"
            elif external:
                state, detail = "warning", ("패널 밖 브리지가 떠 있지만 상태를 알리지 않습니다 — "
                                            "예전(protocol 1) 브리지면 새 촬영을 모릅니다")
            elif running:
                if process.contains("Address already in use"):
                    state, detail = "error", f"포트 {self.bridge_port} 를 다른 프로그램이 쓰고 있습니다"
                else:
                    state, detail = "starting", "여는 중"
        if not running and not external and state == "stopped" \
                and process.exit_code not in (None, 0) and not process.stopping:
            state = "error"
            last = next((line for line in reversed(process.tail(40))
                         if "[패널]" not in line), "")
            detail = f"꺼짐 (종료 코드 {process.exit_code}) {last[-120:]}"
        return {
            "name": name, "label": process.label, "state": state, "detail": detail,
            "running": running or external, "external": external, "pid": process.pid,
            "uptime": round(process.uptime()), "log": process.tail(14),
        }

    def _log_overall(self):
        """준비 상태가 바뀔 때마다 터미널에도 남긴다(vision_run.sh 를 화면 없이 쓸 때)."""
        status = self.status()
        text = status["overall"]["text"]
        if text != self._last_overall:
            self._last_overall = text
            details = " | ".join(f"{c['label']}: {c['detail']}" for c in status["components"])
            self.get_logger().info(f"[상태] {text}  ({details})")

    def status(self):
        components = [self._component_status(name) for name in self.manager.items]
        ready = all(c["state"] in ("ready", "busy") for c in components)
        clients = ((self.bridge_status or {}).get("clients") or []) \
            if time.monotonic() - self.bridge_status_at < 3.0 else []
        result = None
        if self.last_result:
            tomatoes = []
            for i, (x, y, z) in enumerate(self.last_result["poses"]):
                tomatoes.append({
                    "index": i, "x_mm": round(x * 1000, 1), "y_mm": round(y * 1000, 1),
                    "z_mm": round(z * 1000, 1),
                    "weight_g": round(self._weights[i], 1) if i < len(self._weights) else None,
                })
            result = {"age_s": round(time.time() - self.last_result["at"], 1),
                      "count": len(tomatoes), "tomatoes": tomatoes}
        if ready:
            overall = ("준비 완료 — 로봇 PC 에서 [작업 시작] 하면 됩니다" if clients
                       else "준비 완료 — 로봇 PC(robot_run.sh) 의 [연결] 을 기다리는 중")
        elif any(c["state"] == "error" for c in components):
            overall = "확인이 필요한 부품이 있습니다"
        elif any(c["running"] for c in components):
            overall = "준비 중..."
        else:
            overall = "꺼져 있습니다 — [전체 시작] 을 누르세요 (vision_run.sh 면 자동)"
        return {
            "overall": {"ready": ready, "text": overall},
            "components": components,
            "gripper": {"position": self.gripper_position,
                        "label": self._gripper_label(self.gripper_position)
                        if time.monotonic() - self.gripper_position_at < 2.0 else "",
                        "close_value": self.close_value, "open_value": self.open_value},
            "bridge": {"port": self.bridge_port, "clients": clients},
            "result": result,
            "preview_available": self.preview_available,
        }

    # ------------------------------------------------------------ HTTP
    def _make_handler(node_self):
        node = node_self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _reply(self, code, body, content_type):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _json(self, obj, code=200):
                self._reply(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                            "application/json; charset=utf-8")

            def _body(self):
                length = int(self.headers.get("Content-Length") or 0)
                if length <= 0 or length > 4096:
                    return {}
                try:
                    return json.loads(self.rfile.read(length).decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    return {}

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._reply(200, node._page, "text/html; charset=utf-8")
                elif self.path == "/status":
                    self._json(node.status())
                elif self.path.startswith("/snapshot.jpg"):
                    data, reason = node.snapshot_jpeg()
                    if data is None:
                        self._reply(503, reason.encode("utf-8"), "text/plain; charset=utf-8")
                    else:
                        self._reply(200, data, "image/jpeg")
                elif self.path.startswith("/log/"):
                    process = node.manager.get(self.path[len("/log/"):])
                    if process is None:
                        self._reply(404, b"not found", "text/plain")
                    else:
                        self._reply(200, "\n".join(process.tail(400)).encode("utf-8"),
                                    "text/plain; charset=utf-8")
                else:
                    self._reply(404, b"not found", "text/plain")

            def do_POST(self):
                parts = [p for p in self.path.split("/") if p]
                if parts == ["all", "start"]:
                    self._json(node.start_all())
                elif parts == ["all", "stop"]:
                    self._json(node.stop_all())
                elif len(parts) == 3 and parts[0] == "component" \
                        and parts[2] in ("start", "stop", "restart"):
                    self._json(getattr(node, parts[2])(parts[1]))
                elif parts == ["capture"]:
                    self._json(node.capture())
                elif parts == ["gripper"]:
                    self._json(node.gripper(node_self._int(self._body().get("position"))))
                else:
                    self._reply(404, b"not found", "text/plain")

        return Handler

    @staticmethod
    def _int(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return value

    def destroy_node(self):
        try:
            self._server.shutdown()
        except Exception:
            pass
        self.manager.stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionPanelNode()
    # ./stop.sh 는 SIGTERM 을 보낸다 — 그때도 자식(카메라 등)을 정리하고 끝나게 한다.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
