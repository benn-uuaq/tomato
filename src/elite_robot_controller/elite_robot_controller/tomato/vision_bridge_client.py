"""비전 PC(Orin) 와 TCP 로 이어 주는 클라이언트 — 로봇 PC 쪽에서 돈다.

분리 구성에서는 두 장비의 ROS 가 서로를 보지 않는다(`ROS_LOCALHOST_ONLY=1`).
대신 Orin 의 `tomato_tcp_bridge`(비전 팀이 만든 TCP 서버) 에 이 노드가 접속해
필요한 것만 주고받는다.

    [로봇 PC]                                    [Orin]
    시퀀스 ─/tomato/capture─► 이 노드 ─{"cmd":"capture"}──► 브리지 ─► YOLO
    시퀀스 ─/set_pos───────► 이 노드 ─{"cmd":"gripper"}──► 브리지 ─► 그리퍼
    시퀀스 ◄─/tomato/* 3종── 이 노드 ◄─{"type":"result"}─── 브리지 ◄─ YOLO

**로봇 PC 가 먼저 접속하는 쪽**이다. WSL2 는 기본(NAT) 네트워크에서 바깥으로
나가는 연결은 그대로 되지만 들어오는 연결은 막혀 있다. 그래서 Orin 을 서버로
두고 WSL 이 붙으면 Windows 쪽 포트포워딩·방화벽 설정이 필요 없다.

받은 결과는 **원래 비전 노드와 똑같은 토픽 3 개**로 다시 내보낸다. 그래서
시퀀스 노드는 비전이 같은 장비에 있든 다른 장비에 있든 똑같이 동작한다.

프로토콜(JSON Lines, UTF-8) 은 `doosan_vision_grasp/tomato_tcp_bridge.py`
머리말에 있다.
"""

from __future__ import annotations

import json
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32, String
from std_srvs.srv import Trigger


def result_to_messages(result: dict):
    """브리지의 `result` 한 건을 (PoseArray, 무게, 높이) 메시지로 바꾼다.

    단위는 원래 비전 노드와 같다 — 위치 m, 무게 g, 높이 m.
    값이 빠진 토마토는 무게·높이를 0 으로 채운다(인덱스가 어긋나지 않게).
    """
    poses = PoseArray()
    poses.header.frame_id = str(result.get("frame_id") or "camera_color_optical_frame")
    weights = Float32MultiArray()
    heights = Float32MultiArray()
    for tomato in result.get("tomatoes") or []:
        position = tomato.get("position") or {}
        pose = Pose()
        pose.position.x = float(position.get("x", 0.0))
        pose.position.y = float(position.get("y", 0.0))
        pose.position.z = float(position.get("z", 0.0))
        pose.orientation.w = 1.0
        poses.poses.append(pose)
        weight = tomato.get("weight_g")
        height = tomato.get("height_m")
        weights.data.append(float(weight) if weight is not None else 0.0)
        heights.data.append(float(height) if height is not None else 0.0)
    return poses, weights, heights


class VisionBridgeClient(Node):

    def __init__(self):
        super().__init__("vision_bridge_client")
        self.declare_parameter("host", "192.168.1.10")   # Orin 주소 — 연결할 때마다 다시 읽는다
        # False 면 기동해도 붙지 않고 vision/bridge/connect 를 기다린다(UI 의 [연결]).
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("port", 9000)
        self.declare_parameter("reconnect_s", 2.0)
        #: 이만큼 아무것도 못 받으면 끊긴 것으로 본다. ping 을 그 1/3 간격으로 보낸다.
        self.declare_parameter("timeout_s", 6.0)
        self.declare_parameter("capture_topic", "/tomato/capture")
        self.declare_parameter("gripper_topic", "/set_pos")
        self.declare_parameter("pose_topic", "/tomato/center_positions")
        self.declare_parameter("weight_topic", "/tomato/estimated_weights")
        self.declare_parameter("height_topic", "/tomato/heights")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        #: 운영자가 연결을 원하는지. 끊겨도 이 값이 True 면 다시 붙는다.
        self._want = bool(self.get_parameter("auto_connect").value)
        self._wake = threading.Event()
        self._last_error = ""
        self.reconnect_s = float(self.get_parameter("reconnect_s").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        def param(name):
            return str(self.get_parameter(name).value)

        self.pub_poses = self.create_publisher(PoseArray, param("pose_topic"), 10)
        self.pub_weights = self.create_publisher(Float32MultiArray, param("weight_topic"), 10)
        self.pub_heights = self.create_publisher(Float32MultiArray, param("height_topic"), 10)
        # 시퀀스와 UI 가 이 값을 보고 "비전 PC 끊김" 을 알린다.
        self.pub_link = self.create_publisher(Bool, "vision/bridge/connected", 10)
        # UI 가 "연결 중 / 연결됨 / 실패: 사유" 를 보여 주는 데 쓴다(JSON).
        self.pub_state = self.create_publisher(String, "vision/bridge/state", 10)
        self.create_service(Trigger, "vision/bridge/connect", self.cb_connect)
        self.create_service(Trigger, "vision/bridge/disconnect", self.cb_disconnect)

        self.create_subscription(Empty, param("capture_topic"), self.cb_capture, 10)
        self.create_subscription(Int32, param("gripper_topic"), self.cb_gripper, 10)

        self._sock = None
        self._send_lock = threading.Lock()
        self._connected = False
        self._last_rx = 0.0
        self._last_ping = 0.0
        self._running = True
        self._protocol = None

        threading.Thread(target=self._connect_loop, name="vision-bridge", daemon=True).start()
        self.create_timer(0.5, self._watchdog)
        if self._want:
            self.get_logger().info(f"[bridge] 비전 PC {self.host}:{self.port} 에 접속합니다.")
        else:
            self.get_logger().info("[bridge] UI 에서 Orin IP 를 넣고 [연결] 을 누르면 붙습니다.")

    # ------------------------------------------------------------ 연결
    def _connect_loop(self):
        """운영자가 원하는 동안만 붙어 있는다. 끊기면 `reconnect_s` 마다 다시 붙는다.

        주소(host 파라미터)는 **붙을 때마다 다시 읽는다** — UI 에서 Orin IP 를
        바꾸고 [연결] 을 누르면 그 주소로 붙게 하기 위해서다.
        """
        announced_fail = False
        while self._running and rclpy.ok():
            if not self._want:
                self._wake.wait(0.5)
                self._wake.clear()
                announced_fail = False
                continue
            self.host = str(self.get_parameter("host").value)
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
            except OSError as exc:
                self._last_error = f"{self.host}:{self.port} 접속 실패 ({exc})"
                if not announced_fail:
                    self.get_logger().warn(
                        f"[bridge] {self._last_error} — {self.reconnect_s:g}초마다 다시 시도합니다.")
                    announced_fail = True
                self._publish_state()
                self._wake.wait(self.reconnect_s)
                self._wake.clear()
                continue
            announced_fail = False
            self._last_error = ""
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(1.0)
            with self._send_lock:
                self._sock = sock
            self._last_rx = time.monotonic()
            self._receive(sock)
            self._drop("연결 끊김")
            if self._want:
                self._wake.wait(self.reconnect_s)
                self._wake.clear()

    def _receive(self, sock):
        buf = b""
        while self._running and self._sock is sock:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            self._last_rx = time.monotonic()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle(line.decode("utf-8", "replace").strip())

    def _drop(self, reason):
        with self._send_lock:
            sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if self._connected:
            self.get_logger().warn(f"[bridge] 비전 PC 와 {reason}.")
        self._connected = False
        self._publish_link()

    # ------------------------------------------------------------ 수신
    def _handle(self, line):
        if not line:
            return
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"[bridge] JSON 이 아닌 줄을 받았습니다: {line[:80]}")
            return
        kind = message.get("type")
        if kind == "hello":
            self._protocol = message.get("protocol")
            self._connected = True
            self._publish_link()
            self.get_logger().info(
                f"[bridge] 비전 PC 연결됨 (protocol {self._protocol})")
            if self._protocol is not None and int(self._protocol) < 2:
                self.get_logger().warn(
                    "[bridge] 비전 PC 브리지가 protocol 1 입니다 — 그리퍼 명령을 모릅니다. "
                    "Orin 의 doosan_vision_grasp 를 갱신하세요.")
        elif kind == "result":
            poses, weights, heights = result_to_messages(message)
            # 원래 비전 노드와 같은 순서(위치 -> 무게 -> 높이)로 내보낸다.
            self.pub_poses.publish(poses)
            self.pub_weights.publish(weights)
            self.pub_heights.publish(heights)
            self.get_logger().info(f"[bridge] 검출 결과 수신: 토마토 {message.get('count', 0)}개")
        elif kind == "ack":
            self.get_logger().info(f"[bridge] 비전 PC 확인: {message}")
        elif kind == "error":
            self.get_logger().error(f"[bridge] 비전 PC 오류: {message.get('message')}")
        # pong 은 _last_rx 갱신만으로 충분하다.

    # ------------------------------------------------------------ 송신
    def _send(self, payload: dict) -> bool:
        data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        with self._send_lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall(data)
                return True
            except OSError:
                pass
        self._drop("전송 실패로 끊김")
        return False

    def cb_capture(self, _msg):
        if not self._send({"cmd": "capture"}):
            self.get_logger().error("[bridge] 비전 PC 에 연결되어 있지 않아 촬영 요청을 못 보냈습니다.")

    def cb_gripper(self, msg):
        position = int(msg.data)
        if not self._send({"cmd": "gripper", "position": position}):
            self.get_logger().error(
                f"[bridge] 비전 PC 에 연결되어 있지 않아 그리퍼 명령({position})을 못 보냈습니다.")

    # ------------------------------------------------------------ 감시
    def _watchdog(self):
        now = time.monotonic()
        if self._sock is not None:
            if now - self._last_rx > self.timeout_s:
                self._drop(f"{self.timeout_s:g}초 동안 응답이 없어 끊음")
            elif now - self._last_ping > self.timeout_s / 3.0:
                self._last_ping = now
                self._send({"cmd": "ping"})
        self._publish_link()
        self._publish_state()

    def _publish_link(self):
        self.pub_link.publish(Bool(data=bool(self._connected and self._sock is not None)))

    def cb_connect(self, req, res):
        """[연결] — host 파라미터의 주소로 붙는다. 다른 주소에 붙어 있었으면 끊고 옮긴다."""
        target = str(self.get_parameter("host").value)
        if self._sock is not None and target != self.host:
            self._drop(f"주소 변경({self.host} -> {target})")
        self._want = True
        self._last_error = ""
        self._wake.set()
        res.success = True
        res.message = f"[bridge] 비전 PC {target}:{self.port} 에 연결을 시도합니다."
        self.get_logger().info(res.message)
        return res

    def cb_disconnect(self, req, res):
        """[연결 해제] — 다시 붙지 않는다."""
        self._want = False
        self._wake.set()
        self._drop("작업자 연결 해제")
        res.success = True
        res.message = "[bridge] 비전 PC 연결을 해제했습니다."
        return res

    def _publish_state(self):
        connected = bool(self._connected and self._sock is not None)
        if connected:
            phase = "connected"
        elif not self._want:
            phase = "idle"
        elif self._last_error:
            phase = "error"
        else:
            phase = "connecting"
        self.pub_state.publish(String(data=json.dumps({
            "phase": phase,
            "host": str(self.get_parameter("host").value),
            "port": self.port,
            "protocol": self._protocol,
            "error": self._last_error,
        }, ensure_ascii=False)))

    def destroy_node(self):
        self._running = False
        self._wake.set()
        self._drop("종료")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionBridgeClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
