"""비전 쪽과 맞닿는 곳을 여기 한 파일에 모은다.

지금 붙는 상대는 `doosan_vision_grasp` 의 `tomato_area_d435i_v2` 노드다
(README.odt 와 같은 인터페이스).

| 방향 | 토픽 | 타입 | 단위 |
| --- | --- | --- | --- |
| 로봇 -> 비전 | `/tomato/capture` | `std_msgs/Empty` | 트리거. 약 1 초간 측정한다 |
| 비전 -> 로봇 | `/tomato/center_positions` | `geometry_msgs/PoseArray` | **m**, 카메라 optical frame |
| 비전 -> 로봇 | `/tomato/estimated_weights` | `std_msgs/Float32MultiArray` | **g** |
| 비전 -> 로봇 | `/tomato/heights` | `std_msgs/Float32MultiArray` | **m** |

세 토픽은 **같은 순서**로 한 번씩 발행된다(i 번째 pose 와 i 번째 무게가 같은
토마토다). 검출이 0 개여도 빈 목록을 보내 주므로 로봇이 매달리지 않는다.

좌표 변환에는 두 가지 방식이 있다(`vision.detection_frame`).

``camera_delta`` (기본)
    README 에 적힌 검증된 방식이다. 카메라 X·Y 를 **지금 위치에서 움직일 양**
    으로 바로 바꾼다(`camera_axis_map`, 기본 `["-y", "-x"]` — 카메라 X·Y 를
    각각 부호 반전해 베이스 Y·X 로). 핸드인아이 회전 보정이 필요 없어
    카메라를 아래로 내려 보는 배치에서 가장 단순하다.
``camera`` / ``tool`` / ``base``
    핸드인아이 변환(`hand_eye_*`)을 제대로 태우는 일반 방식이다. 카메라가
    기울어 달렸거나 툴 자세가 자주 바뀌면 이쪽을 쓴다.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field

from . import geometry


@dataclass
class Detection:
    """토마토 하나. 좌표 단위는 m, 무게는 g, 높이·지름은 mm 다."""

    index: int = 0
    x: float = 0.0          # m (검출 프레임 기준)
    y: float = 0.0          # m
    z: float = 0.0          # m — 카메라에서 토마토까지의 거리
    diameter: float = 0.0   # mm
    height: float = 0.0     # mm — /tomato/heights
    weight: float = 0.0     # g  — /tomato/estimated_weights
    extra: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "x_mm": round(self.x * 1000.0, 1),
            "y_mm": round(self.y * 1000.0, 1),
            "z_mm": round(self.z * 1000.0, 1),
            "diameter_mm": round(self.diameter, 1),
            "height_mm": round(self.height, 1),
            "weight_g": round(self.weight, 1),
        }


#: 검출 토픽 타입을 지정하지 않았을 때의 기본값. 지금 쓰는 비전 노드에 맞춘다.
DEFAULT_TYPE = "geometry_msgs/msg/PoseArray"
#: Float32MultiArray 로 받을 때의 묶음 크기 [x, y, z, 지름, 무게].
DEFAULT_STRIDE = 5
#: 축 이름 -> (성분 번호, 부호).
_AXIS = {"x": (0, 1.0), "+x": (0, 1.0), "-x": (0, -1.0),
         "y": (1, 1.0), "+y": (1, 1.0), "-y": (1, -1.0),
         "z": (2, 1.0), "+z": (2, 1.0), "-z": (2, -1.0)}


def import_message(type_name: str):
    """"pkg/msg/Type" 또는 "pkg/Type" 문자열로 메시지 클래스를 가져온다."""
    parts = [p for p in type_name.split("/") if p]
    if len(parts) == 2:
        package, kind, name = parts[0], "msg", parts[1]
    elif len(parts) == 3:
        package, kind, name = parts
    else:
        raise ValueError(f"메시지 타입 형식이 아닙니다: {type_name!r}")
    module = importlib.import_module(f"{package}.{kind}")
    return getattr(module, name)


def parse_detections(msg) -> list:
    """비전 메시지를 Detection 목록으로 바꾼다.

    지금 쓰는 `geometry_msgs/PoseArray` 외에 두 가지를 더 알아본다. 실제
    메시지가 이와 다르면 **여기에 분기 하나만 더 붙이면 된다** — 시퀀스
    코드는 건드릴 필요가 없다.

    1. ``geometry_msgs/PoseArray`` — 위치(m). 무게·높이는 따로 온다.
    2. ``std_msgs/Float32MultiArray`` — [x, y, z, 지름, 무게] * N (mm, g)
    3. ``vision_msgs/Detection3DArray`` — bbox 중심과 크기
    """
    poses = getattr(msg, "poses", None)
    if poses is not None:
        return [
            Detection(index=i, x=p.position.x, y=p.position.y, z=p.position.z)
            for i, p in enumerate(poses)
        ]

    data = getattr(msg, "data", None)
    if data is not None and not isinstance(data, (str, bytes)):
        return _parse_flat_array(list(data))

    detections = getattr(msg, "detections", None)
    if detections is not None:
        return _parse_detection3d(detections)

    raise TypeError(f"알 수 없는 비전 메시지 구조입니다: {type(msg).__name__}")


def _parse_flat_array(values: list) -> list:
    """[x, y, z, 지름, 무게] * N (mm). 5 의 배수가 아니면 3 개씩(위치만)으로 본다."""
    stride = DEFAULT_STRIDE if values and len(values) % DEFAULT_STRIDE == 0 else 3
    if not values or len(values) % stride:
        return []
    out = []
    for i in range(0, len(values), stride):
        chunk = values[i:i + stride]
        out.append(Detection(
            index=i // stride,
            x=float(chunk[0]) / 1000.0,
            y=float(chunk[1]) / 1000.0,
            z=float(chunk[2]) / 1000.0,
            diameter=float(chunk[3]) if stride == DEFAULT_STRIDE else 0.0,
            weight=float(chunk[4]) if stride == DEFAULT_STRIDE else 0.0,
        ))
    return out


def _parse_detection3d(detections: list) -> list:
    out = []
    for i, det in enumerate(detections):
        center = det.bbox.center.position
        size = det.bbox.size
        out.append(Detection(index=i, x=center.x, y=center.y, z=center.z,
                             diameter=max(size.x, size.y, size.z) * 1000.0))
    return out


def camera_delta(detection: Detection, axis_map) -> list:
    """카메라 X·Y 를 **베이스에서 움직일 양** [dx, dy] (m) 로 바꾼다.

    `axis_map` 은 ["-y", "-x"] 처럼 두 개다 — 앞이 베이스 dx 를 만드는
    카메라 축, 뒤가 dy 를 만드는 축이다. README 의 "(x, y) -> (-y, -x)" 가
    기본값이다. 실제 장착 방향이 다르면 이 값만 바꾸면 된다.
    """
    point = [detection.x, detection.y, detection.z]
    out = []
    for name in list(axis_map)[:2]:
        key = str(name).strip().lower()
        if key not in _AXIS:
            raise ValueError(f"camera_axis_map 값이 이상합니다: {name!r}")
        index, sign = _AXIS[key]
        out.append(point[index] * sign)
    return out


def detection_to_base(detection: Detection, tool_pose, config) -> list:
    """검출 좌표를 로봇 베이스 좌표 [x, y, z] (m) 로 바꾼다.

    `tool_pose` 는 **찍은 순간의** TCP pose 다(핸드인아이라 카메라가 툴을
    따라 움직인다). `camera_delta` 방식에서는 Z 를 건드리지 않으므로
    돌려주는 Z 는 찍은 자리의 Z 그대로다.
    """
    frame = config.detection_frame
    if frame == "camera_delta":
        dx, dy = camera_delta(detection, config.camera_axis_map)
        return [tool_pose[0] + dx, tool_pose[1] + dy, tool_pose[2]]
    point = [detection.x, detection.y, detection.z]
    if frame == "base":
        return point
    if frame == "tool":
        return geometry.transform_point(tool_pose, point)
    if frame == "camera":
        return geometry.transform_point(
            tool_pose, geometry.transform_point(config.hand_eye_pose, point))
    raise ValueError(f"detection_frame 값이 이상합니다: {frame!r}")


class VisionClient:
    """검출 3 종 구독과 트리거 발행을 맡는다. 시퀀스 노드는 이 객체만 쓴다.

    한 번의 촬영 결과는 세 토픽으로 나뉘어 **거의 동시에** 들어온다. 위치가
    먼저 와도 무게가 아직 안 왔을 수 있어, 위치를 받은 뒤 `merge_grace_s`
    만큼 기다렸다가 하나로 합쳐 내보낸다.
    """

    #: 위치를 받은 뒤 무게·높이를 기다리는 시간 [s].
    MERGE_GRACE_S = 0.2

    def __init__(self, node, config, on_detections=None):
        self.node = node
        self.config = config
        self.on_detections = on_detections
        self.latest = []            # 합쳐진 Detection 목록
        self.latest_stamp = 0.0     # 합쳐진 시각 [s]
        self._pending = None        # 위치만 먼저 받아 둔 목록
        self._pending_stamp = 0.0
        self._weights = []
        self._heights = []
        self._trigger_pub = None
        self._trigger_client = None
        self._setup()

    # ------------------------------------------------------------- 구성
    def _setup(self):
        cfg = self.config
        if cfg.detection_topic:
            type_name = cfg.detection_type or DEFAULT_TYPE
            try:
                msg_type = import_message(type_name)
            except Exception as exc:
                self.node.get_logger().error(
                    f"[vision] 검출 메시지 타입을 못 불러왔습니다({type_name}): {exc}")
                return
            self.node.create_subscription(msg_type, cfg.detection_topic, self._on_msg, 10)
            self.node.get_logger().info(
                f"[vision] 검출 구독: {cfg.detection_topic} ({type_name})")
        else:
            self.node.get_logger().warn(
                "[vision] detection_topic 이 비어 있습니다 — 비전 정보를 받지 못합니다.")

        from std_msgs.msg import Float32MultiArray
        if cfg.weight_topic:
            self.node.create_subscription(
                Float32MultiArray, cfg.weight_topic, self._on_weights, 10)
        if cfg.height_topic:
            self.node.create_subscription(
                Float32MultiArray, cfg.height_topic, self._on_heights, 10)

        if cfg.trigger_topic:
            self._trigger_pub = self.node.create_publisher(
                self._trigger_msg_type(), cfg.trigger_topic, 10)
            self.node.get_logger().info(
                f"[vision] 트리거 발행: {cfg.trigger_topic} ({cfg.trigger_type})")
        if cfg.trigger_service:
            from std_srvs.srv import Trigger
            self._trigger_client = self.node.create_client(Trigger, cfg.trigger_service)

    def _trigger_msg_type(self):
        if self.config.trigger_type == "bool":
            from std_msgs.msg import Bool
            return Bool
        from std_msgs.msg import Empty
        return Empty

    # ------------------------------------------------------------- 수신
    def _on_msg(self, msg):
        try:
            detections = parse_detections(msg)
        except Exception as exc:
            self.node.get_logger().error(f"[vision] 검출 메시지 해석 실패: {exc}")
            return
        self._pending = detections
        self._pending_stamp = self._now()
        # 위치만 따로 오는 형식(PoseArray)이 아니면 곧바로 확정한다.
        if not (self.config.weight_topic or self.config.height_topic):
            self._commit()

    def _on_weights(self, msg):
        self._weights = list(msg.data)

    def _on_heights(self, msg):
        self._heights = list(msg.data)

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _commit(self):
        """위치·무게·높이를 순서대로 합쳐 하나의 검출 목록으로 확정한다."""
        detections = self._pending or []
        for i, det in enumerate(detections):
            if i < len(self._weights):
                det.weight = float(self._weights[i])
            if i < len(self._heights):
                # /tomato/heights 는 m 로 온다 -> mm 로 둔다.
                det.height = float(self._heights[i]) * 1000.0
        self.latest = detections
        self.latest_stamp = self._now()
        self._pending = None
        if self.on_detections:
            self.on_detections(detections)

    def poll(self):
        """타이머마다 부른다. 합칠 때가 됐으면 합친다."""
        if self._pending is None:
            return
        if (self._now() - self._pending_stamp) >= self.MERGE_GRACE_S:
            self._commit()

    def clear(self):
        """트리거 직전에 비운다 — 지난 검출을 이번 것으로 착각하지 않게."""
        self.latest = []
        self.latest_stamp = 0.0
        self._pending = None
        self._weights = []
        self._heights = []

    # ------------------------------------------------------------- 트리거
    def trigger(self) -> tuple:
        """카메라를 찍게 한다. (성공, 안내문) 을 돌려준다."""
        if self._trigger_pub is not None:
            if self.config.trigger_type == "bool":
                from std_msgs.msg import Bool
                self._trigger_pub.publish(Bool(data=True))
            else:
                from std_msgs.msg import Empty
                self._trigger_pub.publish(Empty())
            return True, f"[vision] 트리거 발행: {self.config.trigger_topic}"
        if self._trigger_client is not None:
            if not self._trigger_client.service_is_ready():
                return False, f"[vision] 트리거 서비스 대기 중: {self.config.trigger_service}"
            from std_srvs.srv import Trigger
            self._trigger_client.call_async(Trigger.Request())
            return True, f"[vision] 트리거 서비스 호출: {self.config.trigger_service}"
        return False, "[vision] 트리거 토픽/서비스가 설정되지 않았습니다."
