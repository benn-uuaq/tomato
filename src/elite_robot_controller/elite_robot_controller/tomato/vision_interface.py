"""비전 쪽과 맞닿는 곳을 여기 한 파일에 모은다.

지금 붙는 상대는 한양대 통합 비전 노드 `tomato_vision_d435i*.py`(2026-10)다.
트리거 하나에 결과 토픽이 정해져 있다(위치는 **m, 카메라 optical frame**,
무게 g, 높이 m). 검출이 0 개여도 빈 목록을 보내 주므로 로봇이 매달리지 않는다.

| 촬영 (MODES) | 트리거 | 위치 | 무게 | 높이 |
| --- | --- | --- | --- | --- |
| nearest — 가장 가까운 1 개 | `/tomato/nearest_capture` | `/tomato/nearest_position` | `/tomato/nearest_weight` | - |
| middle — 화면 3x3 가운데 칸의 최근접 1 개 | `/tomato/middle_capture` | `/tomato/middle_position` | `/tomato/middle_weight` | - |
| pick — 전체, 가까운 순 | `/tomato/pick_capture` | `/tomato/pick_positions` | `/tomato/pick_weights` | - |
| weight — 면적·높이 무게 (아래를 볼 때만) | `/tomato/capture` | `/tomato/center_positions` | `/tomato/estimated_weights` | `/tomato/heights` |

시퀀스는 대기 자세에서 `nearest` 로 찾고, 카메라 중심을 맞춘 뒤 `middle` 로 다시
잰다(`vision.first_capture`, `vision.recapture`). 위치 Z 는 토마토 **앞면** 깊이다
(마스크 깊이의 10 백분위 — 한양대 노드 `depth_percentile`).

비전이 Orin 에 있을 때는 로봇 PC 의 브리지 클라이언트가 같은 토픽 이름으로
다시 내보내 주므로 이 파일은 차이를 모른다.

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

#: 촬영 종류 -> (트리거, 위치, 무게, 높이) 토픽. 한양대 통합 비전 노드와 같다.
MODES = {
    "nearest": ("/tomato/nearest_capture", "/tomato/nearest_position",
                "/tomato/nearest_weight", None),
    "middle": ("/tomato/middle_capture", "/tomato/middle_position",
               "/tomato/middle_weight", None),
    "pick": ("/tomato/pick_capture", "/tomato/pick_positions", "/tomato/pick_weights", None),
    "weight": ("/tomato/capture", "/tomato/center_positions",
               "/tomato/estimated_weights", "/tomato/heights"),
}
MODE_LABELS = {"nearest": "최근접 촬영", "middle": "중앙 재측정",
               "pick": "전체 촬영", "weight": "무게 촬영"}
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
    """촬영 트리거를 내고 그 촬영의 결과(위치·무게·높이)를 모은다.

    한 번의 촬영 결과는 토픽 여러 개로 나뉘어 **거의 동시에** 들어온다. 위치가
    먼저 와도 무게가 아직 안 왔을 수 있어, 위치를 받은 뒤 `MERGE_GRACE_S`
    만큼 기다렸다가 하나로 합쳐 내보낸다. **이번에 요청한 촬영 종류의 결과만**
    받는다 — 비전 패널에서 누른 시험 촬영 같은 다른 결과는 버린다.
    """

    #: 위치를 받은 뒤 무게·높이를 기다리는 시간 [s].
    MERGE_GRACE_S = 0.2

    def __init__(self, node, config, on_detections=None):
        self.node = node
        self.config = config
        self.on_detections = on_detections
        self.latest = []            # 합쳐진 Detection 목록
        self.latest_stamp = 0.0     # 합쳐진 시각 [s]
        self.latest_mode = None
        self.expect = None          # 지금 기다리는 촬영 종류
        self._pending = None
        self._pending_stamp = 0.0
        self._weights = []
        self._heights = []
        self._trigger_pubs = {}
        from std_msgs.msg import Empty, Float32MultiArray
        msg_type = import_message(DEFAULT_TYPE)
        modes = []
        # 픽앤플레이스(첫 촬영·재측정)와 무게 추정 작업이 쓰는 촬영 종류를 모두 준비한다.
        for mode in (config.first_capture, config.recapture,
                     getattr(config, "weight_capture", None)):
            if mode in MODES and mode not in modes:
                modes.append(mode)
        for mode in modes:
            trigger, pose_topic, weight_topic, height_topic = MODES[mode]
            self._trigger_pubs[mode] = node.create_publisher(Empty, trigger, 10)
            node.create_subscription(
                msg_type, pose_topic, lambda m, md=mode: self._on_msg(md, m), 10)
            node.create_subscription(
                Float32MultiArray, weight_topic,
                lambda m, md=mode: self._on_array(md, "_weights", m), 10)
            if height_topic:
                node.create_subscription(
                    Float32MultiArray, height_topic,
                    lambda m, md=mode: self._on_array(md, "_heights", m), 10)
            node.get_logger().info(
                f"[vision] {MODE_LABELS[mode]}: {trigger} -> {pose_topic}")

    # ------------------------------------------------------------- 수신
    def _on_msg(self, mode, msg):
        if mode != self.expect:
            return
        try:
            detections = parse_detections(msg)
        except Exception as exc:
            self.node.get_logger().error(f"[vision] 검출 메시지 해석 실패: {exc}")
            return
        self._pending = detections
        self._pending_stamp = self._now()

    def _on_array(self, mode, attr, msg):
        if mode == self.expect:
            setattr(self, attr, list(msg.data))

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _commit(self):
        """위치·무게·높이를 순서대로 합쳐 하나의 검출 목록으로 확정한다."""
        detections = self._pending or []
        for i, det in enumerate(detections):
            if i < len(self._weights):
                det.weight = float(self._weights[i])
            if i < len(self._heights):
                det.height = float(self._heights[i]) * 1000.0   # m -> mm
        self.latest = detections
        self.latest_stamp = self._now()
        self.latest_mode = self.expect
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
    def trigger(self, mode: str = None) -> tuple:
        """카메라를 찍게 한다. (성공, 안내문) 을 돌려준다."""
        mode = mode or self.config.first_capture
        pub = self._trigger_pubs.get(mode)
        if pub is None:
            return False, f"[vision] 모르는 촬영 종류입니다: {mode}"
        from std_msgs.msg import Empty
        self.clear()
        self.expect = mode
        pub.publish(Empty())
        return True, f"[vision] {MODE_LABELS[mode]} 요청: {MODES[mode][0]}"
