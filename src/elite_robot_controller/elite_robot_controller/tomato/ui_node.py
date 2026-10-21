"""작업 시작·정지와 현재 상태를 보는 웹 UI 노드.

Orin nano 는 화면 없이 쓰는 일이 많아 **브라우저에서 여는 형태**로 만들었다.
파이썬 표준 라이브러리의 HTTP 서버만 쓰므로 의존이 늘지 않고, 같은 네트워크의
노트북이나 태블릿에서도 열 수 있다.

    ./robot_run.sh
    # 브라우저에서 http://<Orin IP>:8080   (Orin 화면에는 창이 자동으로 뜬다)

UI 는 `pick/status` 를 그대로 보여주고, 버튼은 `pick/*` 서비스를 부른다.

연결도 UI 에서 한다. 로봇 IP 를 넣고 [연결] 을 누르면 `robot_control_node.robot_ip`
를 바꾸고 connect 서비스를 부른다. 마지막에 쓴 주소는 `~/.tomato_pick/connection.yaml`
에 남겨 다음에 미리 채운다.

[비전 시작] / [비전 정지] 는 비전 패널(:8081, 같은 장비)에 카메라·YOLO·그리퍼를
켜고 끄라고 한다(`vision_remote`).

두 대로 나눠 쓸 때(브리지 클라이언트가 떠 있을 때)만 Orin IP 칸이 생기고, [연결] 이
브리지(`vision_bridge_client.host`)도 붙이며, [비전 시작] 은 그 Orin 의 패널을 부른다.
"""

from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import connection
from .pick_sequence_node import IDLE_STATES, STEPS
from .vision_remote import VisionRemote, summarize

#: UI 버튼 -> 서비스 이름.
COMMANDS = {
    "start": "pick/start",
    "start_loop": "pick/start_loop",
    "stop": "pick/stop",
    "pause": "pick/pause",
    "resume": "pick/resume",
    # 위치 가르치기 — 지금 TCP·관절값을 읽어 저장만 한다(로봇은 움직이지 않는다).
    "teach_home": "pick/teach/home_pose",
    "teach_wait": "pick/teach/wait_pose",
    "teach_place": "pick/teach/place_pose",
    "teach_limit": "pick/teach/descend_limit",
    # ★ 경유점 — 홈 <-> 대기 / 홈 <-> 놓는 자리
    "wp_wait_add": "pick/teach/waypoint_wait_add",
    "wp_wait_undo": "pick/teach/waypoint_wait_undo",
    "wp_wait_clear": "pick/teach/waypoint_wait_clear",
    "wp_place_add": "pick/teach/waypoint_place_add",
    "wp_place_undo": "pick/teach/waypoint_place_undo",
    "wp_place_clear": "pick/teach/waypoint_place_clear",
    "teach_clear": "pick/teach/clear",
}


def _index_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "index.html")


class PickUiNode(Node):

    def __init__(self):
        super().__init__("tomato_pick_ui")
        self.declare_parameter("ui_port", 8080)
        self.declare_parameter("ui_host", "0.0.0.0")
        #: 서비스 응답을 기다리는 한도 [s]. UI 버튼은 곧바로 답이 와야 한다.
        self.declare_parameter("service_timeout_s", 5.0)

        # 연결 관련. 처음 화면에 채울 주소는 기억해 둔 파일 > 런치 인자 순이다.
        self.declare_parameter("robot_ip", "192.168.1.200")
        self.declare_parameter("vision_host", "192.168.1.10")
        self.declare_parameter("connection_file", connection.DEFAULT_PATH)
        self.declare_parameter("robot_node", "robot_control_node")
        self.declare_parameter("bridge_node", "vision_bridge_client")
        #: 로봇 연결은 소켓 세 개를 차례로 여느라 실패할 때 몇 초 걸린다.
        self.declare_parameter("connect_timeout_s", 12.0)
        #: True 면 띄우자마자 기억해 둔 주소로 [연결] 을 누른 것처럼 한다
        #: (robot_run.sh --connect). 다른 노드가 뜰 때까지 잠깐 기다렸다가 건다.
        self.declare_parameter("connect_on_start", False)
        # [비전 시작] — 비전 패널(HTTP) 이 없으면 vision_run.sh 를 띄운다(두 대로 나눴으면 SSH, 키 인증).
        self.declare_parameter("orin_user", os.environ.get("ORIN_USER", "nano"))
        self.declare_parameter("orin_workspace", os.environ.get("ORIN_WS", "~/tomato_ws"))
        self.declare_parameter("vision_panel_port", 8081)

        self.connection_file = str(self.get_parameter("connection_file").value)
        remembered = connection.load(self.connection_file)
        self.addresses = {
            "robot_ip": remembered.get("robot_ip") or str(self.get_parameter("robot_ip").value),
            "vision_host": remembered.get("vision_host")
                           or str(self.get_parameter("vision_host").value),
        }
        self.last_connect = {}          # 마지막 [연결] 결과 문구
        self.vision_state = None        # 브리지 클라이언트가 알려 주는 상태
        self.remote = VisionRemote(
            user=str(self.get_parameter("orin_user").value),
            workspace=str(self.get_parameter("orin_workspace").value),
            panel_port=int(self.get_parameter("vision_panel_port").value))
        self.vision_panel = summarize(None)     # 비전 패널 상태(요약)
        self.vision_action = ""                 # 마지막 [비전 시작/정지] 결과 문구
        self._vision_busy = threading.Lock()    # 버튼을 연달아 눌러도 한 번만

        self.status = {"state": "IDLE", "step": STEPS["IDLE"], "connected": False}
        self._group = ReentrantCallbackGroup()
        self.create_subscription(
            String, "pick/status", self._on_status, 10, callback_group=self._group)
        self.command_clients = {
            name: self.create_client(Trigger, service, callback_group=self._group)
            for name, service in COMMANDS.items()
        }
        robot_node = str(self.get_parameter("robot_node").value)
        bridge_node = str(self.get_parameter("bridge_node").value)
        self.param_clients = {
            "sequence": self.create_client(
                SetParameters, "/tomato_pick_sequence/set_parameters", callback_group=self._group),
            "robot": self.create_client(
                SetParameters, f"/{robot_node}/set_parameters", callback_group=self._group),
            "vision": self.create_client(
                SetParameters, f"/{bridge_node}/set_parameters", callback_group=self._group),
        }
        self.link_clients = {
            ("robot", "connect"): self.create_client(
                Trigger, "robot/dashboard/connect", callback_group=self._group),
            ("robot", "disconnect"): self.create_client(
                Trigger, "robot/dashboard/disconnect", callback_group=self._group),
            ("vision", "connect"): self.create_client(
                Trigger, "vision/bridge/connect", callback_group=self._group),
            ("vision", "disconnect"): self.create_client(
                Trigger, "vision/bridge/disconnect", callback_group=self._group),
        }
        self.create_subscription(
            String, "vision/bridge/state", self._on_vision_state, 10,
            callback_group=self._group)

        try:
            with open(_index_path(), encoding="utf-8") as handle:
                self._page = handle.read().encode("utf-8")
        except OSError as exc:
            self.get_logger().error(f"[ui] 화면 파일을 못 읽었습니다: {exc}")
            self._page = b"<h1>index.html not found</h1>"

        host = self.get_parameter("ui_host").value
        port = int(self.get_parameter("ui_port").value)
        self._server = ThreadingHTTPServer((host, port), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever,
                         name="pick-ui-http", daemon=True).start()
        self.get_logger().info(f"[ui] http://{host}:{port} 에서 화면을 엽니다.")
        if self.get_parameter("connect_on_start").value:
            threading.Thread(target=self._connect_when_ready, name="auto-connect",
                             daemon=True).start()
        threading.Thread(target=self._watch_vision_panel, name="vision-panel-watch",
                         daemon=True).start()

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_vision_state(self, msg):
        try:
            self.vision_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------ 연결
    @property
    def has_bridge(self) -> bool:
        """비전이 다른 장비(Orin)라 브리지 클라이언트가 떠 있는지."""
        return self.link_clients[("vision", "connect")].service_is_ready()

    def _set_param(self, target: str, name: str, value) -> tuple:
        client = self.param_clients[target]
        if not client.service_is_ready():
            return False, f"{target} 노드를 찾지 못했습니다"
        if isinstance(value, bool):
            kind = Parameter.Type.BOOL
        elif isinstance(value, int):
            kind = Parameter.Type.INTEGER
        elif isinstance(value, float):
            kind = Parameter.Type.DOUBLE
        else:
            kind = Parameter.Type.STRING
        request = SetParameters.Request(
            parameters=[Parameter(name, kind, value).to_parameter_msg()])
        future = client.call_async(request)
        if not _wait_future(future, 3.0):
            return False, "파라미터 설정 응답이 없습니다"
        result = future.result().results[0]
        return bool(result.successful), result.reason

    def _trigger(self, target: str, action: str, timeout: float) -> dict:
        client = self.link_clients[(target, action)]
        if not client.service_is_ready():
            return {"success": False, "message": f"{client.srv_name} 서비스를 찾지 못했습니다."}
        future = client.call_async(Trigger.Request())
        if not _wait_future(future, timeout):
            return {"success": False, "message": f"{client.srv_name} 응답이 없습니다."}
        result = future.result()
        return {"success": bool(result.success), "message": result.message}

    def connect(self, body: dict) -> dict:
        """[연결] — 주소를 검사해 두 노드에 넣고 각자 붙게 한다."""
        wanted = {}
        for key in ("robot_ip", "vision_host"):
            if key == "vision_host" and not self.has_bridge:
                continue            # 비전이 같은 장비면 Orin 주소는 쓰지 않는다
            ok, value, reason = connection.validate_host(body.get(key, ""))
            if not ok:
                label = "로봇 IP" if key == "robot_ip" else "Orin IP"
                return {"success": False, "message": f"{label}: {reason}"}
            wanted[key] = value
        self.addresses.update(wanted)
        try:
            connection.save(self.connection_file, self.addresses)
        except Exception as exc:
            self.get_logger().warn(f"[ui] 주소를 기억하지 못했습니다: {exc}")

        results = {}
        # Orin 쪽은 바로 답이 오므로 먼저 건다(실제 접속은 배경에서 이어진다).
        if "vision_host" in wanted:
            ok, reason = self._set_param("vision", "host", wanted["vision_host"])
            results["vision"] = (self._trigger("vision", "connect", 3.0) if ok else
                                 {"success": False, "message": f"Orin 주소 설정 실패: {reason}"})
        ok, reason = self._set_param("robot", "robot_ip", wanted["robot_ip"])
        timeout = float(self.get_parameter("connect_timeout_s").value)
        results["robot"] = (self._trigger("robot", "connect", timeout) if ok else
                            {"success": False, "message": f"로봇 주소 설정 실패: {reason}"})
        if not results["robot"]["success"]:
            results["robot"]["message"] += " — 2초마다 다시 시도합니다([연결 해제] 로 멈춤)."
        self.last_connect = {key: value["message"] for key, value in results.items()}
        return {
            "success": all(value["success"] for value in results.values()),
            "message": " / ".join(value["message"] for value in results.values()),
            "results": results,
        }

    #: 화면에서 저장하는 숫자 -> (시퀀스 노드 파라미터, 이름, 형, 단위).
    #: 시퀀스 노드가 범위를 검사하고 ~/.tomato_pick/taught_poses.yaml 에 남긴다.
    SETTINGS = {
        "place_approach_mm": ("place_approach_mm", "접근 높이", float, "mm"),
        "place_release_gap_mm": ("place_release_gap_mm", "여는 높이", float, "mm"),
        "gripper_close_value": ("gripper.close_value", "그리퍼 닫기 위치", int, ""),
        "gripper_open_value": ("gripper.open_value", "그리퍼 열기 위치", int, ""),
        "gripper_close_wait_s": ("gripper.close_wait_s", "닫을 때 대기", float, "초"),
        "gripper_open_wait_s": ("gripper.open_wait_s", "열 때 대기", float, "초"),
    }

    def save_settings(self, body: dict) -> dict:
        """놓기 높이 [mm], 그리퍼 위치·대기 시간. 시퀀스 노드가 검사하고 파일에 남긴다."""
        messages = []
        for key, (param, label, kind, unit) in self.SETTINGS.items():
            if body.get(key) in (None, ""):
                continue
            try:
                number = float(body[key])
                if kind is int and number != int(number):
                    raise ValueError
                value = kind(number)
            except (TypeError, ValueError):
                return {"success": False, "message": f"{label}: "
                        + ("정수를 넣으세요." if kind is int else "숫자를 넣으세요.")}
            ok, reason = self._set_param("sequence", param, value)
            if not ok:
                return {"success": False, "message": f"{label}: {reason}"}
            messages.append(f"{label} {value:g}{(' ' + unit) if unit else ''}")
        if not messages:
            return {"success": False, "message": "저장할 값이 없습니다."}
        return {"success": True, "message": "저장: " + ", ".join(messages)}

    def _connect_when_ready(self):
        """노드들이 뜰 때까지(서비스가 보일 때까지) 기다렸다가 기억한 주소로 연결한다."""
        import time
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if self._set_param_ready() and self.link_clients[("robot", "connect")].service_is_ready():
                break
            time.sleep(0.5)
        result = self.connect(dict(self.addresses))
        self.get_logger().info(f"[ui] 시작하며 연결: {result['message']}")

    def _set_param_ready(self) -> bool:
        return self.param_clients["robot"].service_is_ready()

    def disconnect(self) -> dict:
        results = {"robot": self._trigger("robot", "disconnect", 5.0)}
        if self.has_bridge:
            results["vision"] = self._trigger("vision", "disconnect", 3.0)
        self.last_connect = {key: value["message"] for key, value in results.items()}
        return {
            "success": all(value["success"] for value in results.values()),
            "message": " / ".join(value["message"] for value in results.values()),
        }

    # ------------------------------------------------------------ 비전 켜기/끄기
    def _vision_host(self, body: dict) -> tuple:
        """비전 패널이 있는 곳. 한 대면 이 장비, 두 대로 나눴으면 화면의 Orin IP.

        Orin IP 는 검사하고 기억한다. (ok, host 또는 사유)
        """
        if not self.has_bridge:
            return True, "127.0.0.1"
        ok, value, reason = connection.validate_host(
            body.get("vision_host") or self.addresses["vision_host"])
        if not ok:
            return False, f"Orin IP: {reason}"
        if value != self.addresses["vision_host"]:
            self.addresses["vision_host"] = value
            try:
                connection.save(self.connection_file, self.addresses)
            except Exception as exc:
                self.get_logger().warn(f"[ui] 주소를 기억하지 못했습니다: {exc}")
        return True, value

    def vision_start(self, body: dict) -> dict:
        """[비전 시작] — 카메라·YOLO·그리퍼를 켠다. 두 대로 나눴으면 브리지도 그쪽에 붙인다."""
        ok, host = self._vision_host(body)
        if not ok:
            return {"success": False, "message": host}
        if not self._vision_busy.acquire(blocking=False):
            return {"success": False, "message": "이미 비전을 켜는 중입니다."}
        try:
            self.vision_action = f"{self.remote.where(host)} 비전을 켜는 중..."
            result = self.remote.start(host)
            if result["success"] and self.has_bridge:
                # 브리지 서버가 뜰 때까지 2초마다 다시 붙으므로 지금 걸어 두면 된다.
                ok, reason = self._set_param("vision", "host", host)
                link = (self._trigger("vision", "connect", 3.0) if ok else
                        {"success": False, "message": f"Orin 주소 설정 실패: {reason}"})
                self.last_connect["vision"] = link["message"]
                if not link["success"]:
                    result = {"success": False, "message": result["message"] + " / " + link["message"]}
            self.vision_action = result["message"]
            self.get_logger().info(f"[ui] 비전 시작: {result['message']}")
            return result
        finally:
            self._vision_busy.release()

    def vision_stop(self, body: dict) -> dict:
        """[비전 정지] — 비전 패널이 켠 것만 내린다. 작업 중이면 막는다."""
        state = self.status.get("state", "IDLE")
        if state not in IDLE_STATES:
            return {"success": False,
                    "message": f"작업 중({self.status.get('step', state)})에는 비전을 끌 수 없습니다 — 먼저 [정지]."}
        ok, host = self._vision_host(body)
        if not ok:
            return {"success": False, "message": host}
        result = self.remote.stop(host)
        if result["success"] and self.has_bridge:
            # 일부러 끈 것이니 "접속 실패 — 다시 시도 중" 을 띄우지 않게 브리지도 쉰다.
            link = self._trigger("vision", "disconnect", 3.0)
            self.last_connect["vision"] = link["message"]
        self.vision_action = result["message"]
        self.get_logger().info(f"[ui] 비전 정지: {result['message']}")
        return result

    def _watch_vision_panel(self):
        """비전 패널의 상태를 계속 읽어 화면에 보여 준다."""
        import time
        while rclpy.ok():
            host = self.addresses["vision_host"] if self.has_bridge else "127.0.0.1"
            status = self.remote.panel_status(host, timeout=1.5)
            self.vision_panel = summarize(status)
            time.sleep(1.0 if status else 2.0)

    def call(self, name: str) -> dict:
        """버튼 하나를 서비스 호출로 바꾼다. 응답을 그대로 화면에 돌려준다."""
        client = self.command_clients.get(name)
        if client is None:
            return {"success": False, "message": f"모르는 명령입니다: {name}"}
        if not client.service_is_ready():
            return {"success": False,
                    "message": f"{COMMANDS[name]} 서비스를 찾지 못했습니다 — "
                               "시퀀스 노드가 떠 있는지 확인하세요."}
        future = client.call_async(Trigger.Request())
        timeout = float(self.get_parameter("service_timeout_s").value)
        # 별도 스레드(HTTP)에서 부르고 있으므로 executor 를 돌리지 않고 기다린다.
        if not _wait_future(future, timeout):
            return {"success": False, "message": f"{COMMANDS[name]} 응답이 없습니다."}
        result = future.result()
        return {"success": bool(result.success), "message": result.message}

    def payload(self) -> bytes:
        data = dict(self.status)
        data["labels"] = STEPS
        data["connection"] = {
            "robot_ip": self.addresses["robot_ip"],
            "vision_host": self.addresses["vision_host"],
            "has_bridge": self.has_bridge,
            "vision_state": self.vision_state,
            "last": self.last_connect,
        }
        data["vision_panel"] = dict(self.vision_panel, action=self.vision_action,
                                    starting=self._vision_busy.locked(),
                                    port=self.remote.panel_port)
        return json.dumps(data, ensure_ascii=False).encode("utf-8")

    def _make_handler(node_self):
        node = node_self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # 폴링이 잦아 기본 접근 로그는 끈다.

            def _reply(self, code, body, content_type):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._reply(200, node._page, "text/html; charset=utf-8")
                elif self.path == "/status":
                    self._reply(200, node.payload(), "application/json; charset=utf-8")
                else:
                    self._reply(404, b"not found", "text/plain")

            def _json_body(self) -> dict:
                length = int(self.headers.get("Content-Length") or 0)
                if length <= 0 or length > 4096:
                    return {}
                try:
                    return json.loads(self.rfile.read(length).decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    return {}

            def do_POST(self):
                if self.path == "/connect":
                    result = node.connect(self._json_body())
                elif self.path == "/disconnect":
                    result = node.disconnect()
                elif self.path == "/settings":
                    result = node.save_settings(self._json_body())
                elif self.path == "/vision/start":
                    result = node.vision_start(self._json_body())
                elif self.path == "/vision/stop":
                    result = node.vision_stop(self._json_body())
                elif self.path.startswith("/command/"):
                    result = node.call(self.path[len("/command/"):])
                else:
                    self._reply(404, b"not found", "text/plain")
                    return
                self._reply(200, json.dumps(result, ensure_ascii=False).encode("utf-8"),
                            "application/json; charset=utf-8")

        return Handler

    def destroy_node(self):
        try:
            self._server.shutdown()
        except Exception:
            pass
        super().destroy_node()


def _wait_future(future, timeout: float) -> bool:
    """future 가 끝날 때까지 기다린다. executor 는 다른 스레드가 돌린다."""
    import time
    deadline = time.time() + timeout
    while time.time() < deadline:
        if future.done():
            return True
        time.sleep(0.02)
    return False


def main(args=None):
    rclpy.init(args=args)
    node = PickUiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
