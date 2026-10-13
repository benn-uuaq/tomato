"""비전 PC(Orin) 조작 패널 — 브라우저 하나로 README.odt 의 수동 절차를 대신한다.

    ros2 run elite_robot_controller vision_panel_node
    # 브라우저: http://<Orin IP>:8081   (Orin 화면에서는 http://localhost:8081)

예전에는 터미널을 여러 개 열어 이렇게 했다.

    ros2 launch doosan_vision_grasp tomato_area_d435i_v2.launch.py      # 카메라+YOLO
    ros2 run doosan_vision_grasp dyn_test                               # 그리퍼
    ros2 topic pub --once /tomato/capture std_msgs/msg/Empty "{}"       # 촬영
    ros2 topic pub --once /set_pos std_msgs/msg/Int32 "{data: 3185}"    # 집기

이 패널은 그 넷(카메라 · YOLO · 그리퍼 · 외부 통신 브리지)을 각각 켜고 끄고,
촬영 시험과 그리퍼 수동 조작을 버튼으로 한다. [전체 시작] 을 눌러 두면 로봇
PC 에서 [작업 시작] 만 누르면 된다.

**"켜졌다" 와 "준비됐다" 를 구분한다.** 프로세스가 떠 있어도 카메라가 영상을
안 보내거나 모터가 답하지 않을 수 있다. 그래서 부품마다 실제 신호를 본다.

| 부품 | 준비됐다고 보는 조건 |
| --- | --- |
| 카메라 | camera_info 가 2 초 안에 들어옴 |
| YOLO | `/tomato/capture` 를 구독하고 있음 (모델을 다 불러온 뒤에 구독한다) |
| 그리퍼 | 모터가 현재 위치를 답함 (`gripper/present_position`) |
| 외부 통신 | 브리지가 상태를 알림 — 로봇 PC 가 붙어 있으면 그 주소도 |
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

#: 부품별로 "패널 밖에서 떠 있던 것" 을 알아보는 패턴. 시작하기 전에 정리한다
#: — 같은 카메라·같은 시리얼 포트·같은 TCP 포트를 둘이 잡으면 둘 다 망가진다.
LEFTOVER = {
    "camera": "realsense2_camera_node",
    "vision": "tomato_area_d435i_v2",
    "gripper": "gripper_dynamixel_node|lib/doosan_vision_grasp/dyn_test",
    "bridge": "lib/doosan_vision_grasp/tomato_tcp_bridge",
}


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
        self.declare_parameter("gripper_close_value", 3185)
        self.declare_parameter("gripper_open_value", 0)
        #: True 면 패널이 뜨자마자 네 부품을 모두 켠다(사람 없이 켜 둘 때).
        self.declare_parameter("autostart", False)
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
        self._weights, self._heights = [], []
        self.capture_requested_at = None
        self.capture_timeout_note = ""

        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                 self._on_camera_info, qos_profile_sensor_data,
                                 callback_group=self._group)
        self.create_subscription(Int32, "gripper/present_position", self._on_gripper,
                                 10, callback_group=self._group)
        self.create_subscription(String, "vision/bridge/server_status", self._on_bridge,
                                 10, callback_group=self._group)
        self.create_subscription(PoseArray, "/tomato/center_positions", self._on_poses,
                                 10, callback_group=self._group)
        self.create_subscription(Float32MultiArray, "/tomato/estimated_weights",
                                 self._on_weights, 10, callback_group=self._group)
        self.create_subscription(Float32MultiArray, "/tomato/heights",
                                 self._on_heights, 10, callback_group=self._group)
        self.pub_capture = self.create_publisher(Empty, "/tomato/capture", 10)
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
        vision_share = _share(VISION_PACKAGE)
        venv_python = os.environ.get("TOMATO_VENV_PY", "")
        if not venv_python or not os.path.exists(venv_python):
            venv_python = "python3"
        model = os.environ.get("TOMATO_MODEL", "")

        self.manager.add(ManagedProcess(
            "camera", "카메라 (RealSense D435i)",
            ["ros2", "launch", "realsense2_camera", "rs_launch.py",
             "align_depth.enable:=true", "pointcloud.enable:=false", "enable_color:=true"]))

        vision_cmd = [venv_python,
                      os.path.join(vision_share or "", "scripts", "tomato_area_d435i_v2.py"),
                      "--ros-args", "-r", "__node:=tomato_area_calculator_d435i_v2",
                      "--params-file", config]
        if model:
            vision_cmd += ["-p", f"model_path:={model}"]
        self.manager.add(ManagedProcess("vision", "토마토 검출 (YOLO)", vision_cmd))

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

    def _on_heights(self, msg):
        self._heights = list(msg.data)

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
    def _clean_leftover(self, name):
        """패널 밖에서 같은 장치를 잡고 있는 프로세스를 정리한다."""
        pattern = LEFTOVER.get(name)
        process = self.manager.get(name)
        if not pattern:
            return
        mine = {process.pid} if process and process.pid else set()
        try:
            found = subprocess.run(["pgrep", "-f", pattern], capture_output=True,
                                   text=True, timeout=3).stdout.split()
        except Exception:
            return
        others = [int(pid) for pid in found
                  if pid.isdigit() and int(pid) not in mine and int(pid) != os.getpid()]
        for pid in others:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        if others:
            process.lines.append(f"[패널] 패널 밖에서 떠 있던 프로세스를 정리했습니다: {others}")
            time.sleep(3.0)
            for pid in others:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def start(self, name):
        process = self.manager.get(name)
        if process is None:
            return {"success": False, "message": f"모르는 부품입니다: {name}"}
        if not process.running:
            self._clean_leftover(name)
        ok, message = process.start()
        return {"success": ok, "message": message}

    def stop(self, name):
        process = self.manager.get(name)
        if process is None:
            return {"success": False, "message": f"모르는 부품입니다: {name}"}
        ok, message = process.stop()
        return {"success": ok, "message": message}

    def restart(self, name):
        self.stop(name)
        return self.start(name)

    def start_all(self):
        messages = [self.start(name)["message"] for name in ("camera", "gripper", "bridge", "vision")]
        return {"success": True, "message": " / ".join(messages)}

    def stop_all(self):
        self.manager.stop_all()
        return {"success": True, "message": "모두 중지했습니다."}

    def capture(self):
        if not self._vision_ready():
            return {"success": False, "message": "YOLO 가 아직 촬영 대기 상태가 아닙니다."}
        # 카메라 영상이 없으면 YOLO 는 촬영 요청을 **조용히 무시**한다(로그에만
        # 남긴다). 그러면 결과가 영영 안 오니 여기서 미리 막는다.
        if self._camera_fps() == 0:
            return {"success": False, "message": "카메라 영상이 없어 촬영할 수 없습니다 — 카메라부터 켜세요."}
        self.capture_timeout_note = ""
        self.capture_requested_at = time.monotonic()
        self.pub_capture.publish(Empty())
        return {"success": True, "message": "촬영 요청 — 약 1 초 측정합니다."}

    def gripper(self, position):
        if not isinstance(position, int) or not 0 <= position <= 4095:
            return {"success": False, "message": f"위치는 0~4095 정수여야 합니다: {position!r}"}
        if not self.manager.get("gripper").running:
            return {"success": False, "message": "그리퍼 노드가 꺼져 있습니다 — 먼저 시작하세요."}
        self.pub_gripper.publish(Int32(data=position))
        return {"success": True, "message": f"그리퍼 목표 위치 {position}"}

    # ------------------------------------------------------------ 상태 판정
    def _camera_fps(self):
        now = time.monotonic()
        recent = [t for t in self.camera_info_times if now - t < 2.0]
        return len(recent) / 2.0

    def _vision_ready(self):
        return (self.manager.get("vision").running
                and self.count_subscribers("/tomato/capture") > 0)

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
        state, detail = "stopped", "꺼짐"
        if name == "camera":
            fps = self._camera_fps()
            if running and fps > 0:
                state, detail = "ready", f"영상 수신 중 ({fps:.0f} fps)"
            elif running:
                if process.contains("No RealSense devices were found"):
                    state, detail = "error", "카메라를 찾지 못했습니다 — USB 연결 확인"
                elif process.contains("busy"):
                    state, detail = "error", "카메라를 다른 프로그램이 쓰고 있습니다 — 재시작"
                elif process.uptime() > 25:
                    state, detail = "error", "켠 지 25 초가 지나도 영상이 없습니다 — USB 연결 확인"
                else:
                    state, detail = "starting", "켜는 중 (Orin 은 몇 초 걸립니다)"
            elif fps > 0:
                state, detail = "external", "패널 밖에서 실행 중 — [시작] 하면 정리 후 다시 켭니다"
        elif name == "vision":
            # 촬영은 1 초 측정 + 전송이다. 6 초 안에 결과가 없으면 응답이 없는 것.
            if self.capture_requested_at is not None and now - self.capture_requested_at > 6.0:
                self.capture_requested_at = None
                self.capture_timeout_note = "마지막 촬영 요청에 결과가 없었습니다 — 로그를 보세요"
            if self._vision_ready():
                if self.capture_requested_at is not None:
                    state, detail = "busy", "촬영 중..."
                else:
                    state, detail = "ready", "촬영 대기 중"
                    if self.capture_timeout_note:
                        state, detail = "warning", self.capture_timeout_note
                if self._camera_fps() == 0:
                    state, detail = "warning", "촬영 대기 중이지만 카메라 영상이 없습니다"
            elif running:
                if process.contains("Traceback") or process.contains("Error"):
                    state, detail = "error", "오류 — 로그를 보세요"
                else:
                    state, detail = "starting", "YOLO 모델 불러오는 중"
        elif name == "gripper":
            fresh = now - self.gripper_position_at < 2.0
            if running and fresh:
                label = self._gripper_label(self.gripper_position)
                state, detail = "ready", f"모터 응답 — 현재 위치 {self.gripper_position} ({label})"
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
            if running and fresh:
                clients = (self.bridge_status or {}).get("clients") or []
                if clients:
                    hosts = ", ".join(c.split(":")[0] for c in clients)
                    state, detail = "ready", f"로봇 PC 연결됨 ({hosts})"
                else:
                    state, detail = "ready", f"포트 {self.bridge_port} 에서 로봇 PC 를 기다리는 중"
            elif running:
                if process.contains("Address already in use"):
                    state, detail = "error", f"포트 {self.bridge_port} 를 다른 프로그램이 쓰고 있습니다"
                else:
                    state, detail = "starting", "여는 중"
        if not running and state == "stopped" and process.exit_code not in (None, 0) \
                and not process.stopping:
            state = "error"
            last = next((line for line in reversed(process.tail(40))
                         if "[패널]" not in line), "")
            detail = f"꺼짐 (종료 코드 {process.exit_code}) {last[-120:]}"
        return {
            "name": name, "label": process.label, "state": state, "detail": detail,
            "running": running, "pid": process.pid, "uptime": round(process.uptime()),
            "log": process.tail(14),
        }

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
                    "height_mm": round(self._heights[i] * 1000, 1) if i < len(self._heights) else None,
                })
            result = {"age_s": round(time.time() - self.last_result["at"], 1),
                      "count": len(tomatoes), "tomatoes": tomatoes}
        if ready:
            overall = ("준비 완료 — 로봇 PC 에서 [작업 시작] 하면 됩니다" if clients
                       else "준비 완료 — 로봇 PC 의 [연결] 을 기다리는 중")
        elif any(c["state"] == "error" for c in components):
            overall = "확인이 필요한 부품이 있습니다"
        elif any(c["running"] for c in components):
            overall = "준비 중..."
        else:
            overall = "꺼져 있습니다 — [전체 시작] 을 누르세요"
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
