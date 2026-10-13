"""레지스터 맵 적용, 자세 발행, 쓰기 거부 동작을 검증한다.

로봇 없이 실행할 수 있도록 Modbus 클라이언트와 퍼블리셔를 대체한다.
RobotControlNode.__init__은 세 채널 연결을 요구하므로 우회한다.
"""

import threading
from unittest.mock import patch

from elite_robot_controller import register_map
from elite_robot_controller.robot.robot_control_node import RobotControlNode


class FakeModbus:
    """지정한 시작 주소에만 응답하는 Modbus 대역."""

    def __init__(self, registers=None):
        self.registers = registers or {}
        self.reads = []
        self.writes = []
        self.write_result = True

    def get_all_registers(self, start_address, count):
        self.reads.append((start_address, count))
        return self.registers.get(start_address, [])

    def get_register(self, address):
        self.reads.append((address, 1))
        values = self.registers.get(address)
        return values[0] if values else None

    def set_register(self, address, value):
        self.writes.append((address, value))
        return self.write_result


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(list(msg.data) if hasattr(msg.data, "__len__") else msg.data)


class FakeLogger:
    def __init__(self):
        self.warnings = []
        self.infos = []

    def warn(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.warnings.append(message)


class FakeResponse:
    success = False
    message = ""


def make_node(registers=None, map_data=None):
    node = RobotControlNode.__new__(RobotControlNode)
    node.robot_modbus = FakeModbus(registers)
    node.registers = register_map.RegisterMap(map_data or _default_map())
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    # __init__ 을 건너뛰므로 연결 감시 상태를 직접 채운다.
    node._connect_lock = threading.Lock()
    node._want_connected = True
    node._link_fails = 0
    return node


def _default_map():
    return {
        "scale": {"position_per_count": 0.1, "rotation_per_count": 1.0},
        "read": {
            "robot_mode": {"address": 66, "count": 1},
            "tcp_absolute": {"address": 384, "count": 6},
            "tcp_zero_relative": {"address": 280, "count": 6},
            "joint_position": {"address": None, "count": 6},
        },
        "write": {
            "linear_speed": {"address": 300, "count": 1},
            "save_home_pose": {"address": None, "count": 1},
        },
    }


def test_shipped_register_map_matches_known_addresses():
    """패키지에 담긴 설정 파일이 확인된 주소를 그대로 갖고 있어야 한다."""
    registers = register_map.load()

    assert registers.read_entry("tcp_absolute").address == 384
    assert registers.read_entry("tcp_absolute").count == 6
    assert registers.read_entry("tcp_zero_relative").address == 280
    assert registers.read_entry("robot_mode").address == 66
    assert registers.read_entry("control_method").address == 71
    assert registers.read_entry("operation_mode").address == 72
    assert registers.read_entry("joint_position").address == 73
    # 쓰기 주소는 로봇 태스크가 읽는 범용 레지스터 대역에 있어야 한다.
    for name in ("linear_speed", "speed_ratio", "home_joint", "start_pose"):
        entry = registers.write_entry(name)
        assert entry.available, f"{name} 주소가 비어 있습니다"
        assert 256 <= entry.address <= 383, f"{name}이 범용 대역 밖입니다"
    assert registers.write_entry("home_joint").count == 6
    assert registers.write_entry("home_joint").kind == "angle"


def test_joint_angles_use_rotation_scale_for_every_axis():
    """관절 각도는 6축 모두 mrad이므로 위치 환산을 적용하면 안 된다."""
    registers = register_map.load()
    entry = registers.read_entry("joint_position")

    assert entry.kind == "angle"
    assert registers.scales_for(entry) == [1.0] * 6
    # 자세는 앞 3개만 위치 환산을 받는다.
    pose = registers.read_entry("tcp_absolute")
    assert registers.scales_for(pose) == [0.1, 0.1, 0.1, 1.0, 1.0, 1.0]


def test_angle_entry_is_published_without_position_scaling():
    node = make_node(
        {73: [207, -1466, -1875, -1371, 1570, 207]},
        {
            "scale": {"position_per_count": 0.1, "rotation_per_count": 1.0},
            "read": {"joint_position": {"address": 73, "count": 6, "kind": "angle"}},
            "write": {},
        },
    )
    publisher = FakePublisher()

    node.publish_pose("joint_position", publisher)

    assert publisher.messages[0] == [207.0, -1466.0, -1875.0, -1371.0, 1570.0, 207.0]


def test_pose_uses_addresses_from_map():
    node = make_node({384: [0] * 6, 280: [0] * 6})
    absolute, zero = FakePublisher(), FakePublisher()

    node.publish_pose("tcp_absolute", absolute)
    node.publish_pose("tcp_zero_relative", zero)

    assert node.robot_modbus.reads == [(384, 6), (280, 6)]
    assert len(absolute.messages) == 1
    assert len(zero.messages) == 1


def test_position_and_rotation_scaling():
    """위치는 0.1 mm 단위, 회전은 1 mrad 단위로 환산한다."""
    node = make_node({384: [1205, -3400, 5000, 1571, 0, -3141]})
    publisher = FakePublisher()

    node.publish_pose("tcp_absolute", publisher)

    assert publisher.messages[0] == [120.5, -340.0, 500.0, 1571.0, 0.0, -3141.0]


def test_short_or_missing_response_is_not_published():
    """레지스터를 다 읽지 못하면 잘못된 자세를 발행하지 않는다."""
    node = make_node({384: [1, 2, 3]})
    publisher = FakePublisher()

    node.publish_pose("tcp_absolute", publisher)
    node.publish_pose("tcp_zero_relative", publisher)

    assert publisher.messages == []


def test_entry_without_address_is_never_read():
    """주소가 없는 항목은 Modbus 요청 자체를 하지 않는다."""
    node = make_node()
    publisher = FakePublisher()

    node.publish_pose("joint_position", publisher)

    assert node.robot_modbus.reads == []
    assert publisher.messages == []


def test_write_is_rejected_when_address_is_unset():
    """주소 미정 항목은 쓰기를 시도하지 않는다. 엉뚱한 레지스터를 건드리면 위험하다."""
    node = make_node()

    success, message = node.write_register("save_home_pose", 1)

    assert success is False
    assert "주소가 설정되지 않았습니다" in message
    assert node.robot_modbus.writes == []


def test_write_uses_address_from_map():
    node = make_node()

    success, message = node.write_register("linear_speed", 150)

    assert success is True
    assert node.robot_modbus.writes == [(300, 150)]
    assert "300" in message


def test_write_failure_is_reported():
    node = make_node()
    node.robot_modbus.write_result = False

    success, _ = node.write_register("linear_speed", 150)

    assert success is False


def test_pose_write_fills_every_register():
    """자세 저장은 6개 레지스터를 한 번에 채운다."""
    node = make_node(map_data={
        "scale": {"position_per_count": 0.1, "rotation_per_count": 1.0},
        "read": {},
        "write": {
            "home_joint": {"address": 310, "count": 6, "kind": "angle"},
            "pose_src": {"address": 308, "count": 1},
        },
    })

    success, _ = node.write_pose("home_joint", [207.0, -1466.0, -1875.0, -1371.0, 1570.0, 207.0])

    assert success is True
    assert node.robot_modbus.writes == [
        (310, 207), (311, -1466), (312, -1875), (313, -1371), (314, 1570), (315, 207)
    ]


def test_pose_write_rejects_wrong_length():
    """성분이 모자라면 절반만 쓰지 않는다."""
    node = make_node()
    node.registers = register_map.RegisterMap({
        "scale": {}, "read": {},
        "write": {"home_joint": {"address": 310, "count": 6, "kind": "angle"}},
    })

    success, message = node.write_pose("home_joint", [1.0, 2.0])

    assert success is False
    assert "6개가 아닙니다" in message
    assert node.robot_modbus.writes == []


def test_jog_code_maps_axis_and_direction():
    """부호가 방향, 절댓값이 축 번호다. 범위를 벗어나면 움직이지 않는다."""
    node = make_node()

    assert node._jog_vector(1) == [1.0, 0, 0, 0, 0, 0]
    assert node._jog_vector(-6) == [0, 0, 0, 0, 0, -1.0]
    assert node._jog_vector(7) is None
    assert node._jog_vector(0) is None


class FakeChannel:
    """연결/해제만 흉내 내는 통신 채널."""

    def __init__(self, ok=True):
        self.ok = ok
        self.opened = 0
        self.closed = 0

    def connect(self):
        self.opened += 1
        return self.ok

    def disconnect(self):
        self.closed += 1


def make_connectable_node(dash_ok=True, primary_ok=True, modbus_ok=True):
    node = make_node()
    node.robot_ip = "10.0.0.9"
    node.robot_dash = FakeChannel(dash_ok)
    node.robot_primary = FakeChannel(primary_ok)
    node.robot_modbus_channel = FakeChannel(modbus_ok)
    node.robot_dash.connect_29999 = node.robot_dash.connect
    node.robot_dash.disconnect_29999 = node.robot_dash.disconnect
    node.robot_primary.connect_30001 = node.robot_primary.connect
    node.robot_primary.disconnect_30001 = node.robot_primary.disconnect
    modbus = node.robot_modbus
    modbus.connect = node.robot_modbus_channel.connect
    modbus.disconnect = node.robot_modbus_channel.disconnect
    node.connected = False
    node.pub_connected = FakePublisher()
    return node


def test_connect_requires_all_three_channels():
    """한 채널이라도 실패하면 연결로 보지 않는다."""
    node = make_connectable_node(modbus_ok=False)

    assert node.connect_all_servers() is False
    assert node.connected is False

    node = make_connectable_node()
    assert node.connect_all_servers() is True
    assert node.connected is True


def test_connect_service_reports_result():
    node = make_connectable_node(dash_ok=False)
    response = node.cb_connect(None, FakeResponse())

    assert response.success is False
    assert "10.0.0.9" in response.message


def test_disconnect_closes_every_channel():
    """한 채널 정리에 실패해도 나머지는 정리한다."""
    node = make_connectable_node()
    node.connect_all_servers()

    def boom():
        raise RuntimeError("소켓 오류")

    node.robot_dash.disconnect_29999 = boom
    node.cb_disconnect(None, FakeResponse())

    assert node.connected is False
    assert node.robot_primary.closed == 1
    assert node.robot_modbus_channel.closed == 1


def test_loop_does_not_touch_modbus_before_connect():
    """연결 전에 소켓을 건드리면 예외가 난다. 읽기를 건너뛰어야 한다."""
    node = make_connectable_node()
    node.pub_robot_mode = FakePublisher()

    node.update_robot_loop()

    assert node.robot_modbus.reads == []
    assert node.pub_connected.messages == [False]


class FakeParameter:
    def __init__(self, value):
        self.value = value


def make_jog_node(ratio=100):
    node = make_connectable_node()
    node.connected = True
    node.speed_ratio = ratio
    params = {
        'jog_joint_speed_max': 0.50,
        'jog_tcp_speed_max': 0.10,
        'jog_tcp_rot_speed_max': 0.50,
        'jog_accel_max': 1.00,
        'jog_tcp_accel_max': 0.40,
        'jog_hold_time': 0.5,
    }
    node.get_parameter = lambda name: FakeParameter(params[name])
    node.sent = []
    node.robot_primary.send_script = lambda s: node.sent.append(s) or True
    node.robot_dash.robot_stop = lambda: node.sent.append("STOP")
    return node


def test_jog_speed_follows_speed_ratio():
    """조그 속도는 레지스터 307 의 비율만큼 줄어든다."""
    full = make_jog_node(100)
    full.cb_jog_joint(FakeMessageInt(1))
    assert "speedj" in full.sent[0]
    assert "0.5" in full.sent[0]

    slow = make_jog_node(20)
    slow.cb_jog_joint(FakeMessageInt(1))
    # 20 % 이면 0.5 rad/s 의 1/5, 가속도도 같은 비율로 줄어든다.
    assert slow.sent[0] == "speedj([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2, 0.5)"


def test_jog_command_carries_hold_time():
    """t 를 짧게 주어 명령이 끊기면 로봇이 스스로 서게 한다."""
    node = make_jog_node()
    node.cb_jog_joint(FakeMessageInt(1))

    # 마지막 인자가 jog_hold_time 이다. 길게 주면 정지 명령이 실패했을 때
    # 그 시간만큼 계속 움직인다.
    assert node.sent[0].endswith(", 0.5)")


def test_jog_values_have_no_float_noise():
    """스크립트 문자열로 나가므로 부동소수 잡음이 없어야 한다."""
    node = make_jog_node(20)
    node.cb_jog_tcp(FakeMessageInt(3))

    assert node.sent[0] == "speedl([0.0, 0.0, 0.02, 0.0, 0.0, 0.0], 0.08, 0.5)"


def test_jog_tcp_uses_separate_linear_and_angular_speeds():
    """직선과 회전은 단위가 다르므로 따로 곱한다."""
    node = make_jog_node(50)
    node.cb_jog_tcp(FakeMessageInt(1))     # X+
    node.cb_jog_tcp(FakeMessageInt(4))     # RX+

    assert node.sent[0] == "speedl([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2, 0.5)"
    assert node.sent[1] == "speedl([0.0, 0.0, 0.0, 0.25, 0.0, 0.0], 0.2, 0.5)"


def test_jog_stop_uses_dashboard_stop():
    """멈출 때는 감속 없이 서도록 29999 stop 을 쓴다."""
    node = make_jog_node()
    node.cb_jog_joint(FakeMessageInt(0))
    node.cb_jog_tcp(FakeMessageInt(0))

    assert node.sent == ["STOP", "STOP"]


class FakeMessageInt:
    def __init__(self, data):
        self.data = data


def test_connected_status_is_republished_every_cycle():
    """상태가 바뀌지 않아도 매 주기 발행해야 늦게 구독해도 값을 받는다."""
    node = make_connectable_node()
    node.connect_all_servers()
    node.pub_robot_mode = FakePublisher()
    node.pub_control_method = FakePublisher()
    node.pub_op_mode = FakePublisher()
    node.pub_tcp_pose = FakePublisher()
    node.pub_tcp_pose_zero = FakePublisher()
    node.pub_joint_position = FakePublisher()
    node.pub_scan_state = FakePublisher()
    node.pub_task_state = FakePublisher()
    node.pub_home_flag = FakePublisher()
    node.pub_speed_scale = FakePublisher()
    node.pub_alarm = FakePublisher()
    node.alarm_mgr = type("M", (), {"process": lambda self, a: False})()
    node.robot_primary.get_data = lambda: None
    node.robot_primary.alarm_queue = type("Q", (), {"empty": lambda self: True})()

    node.update_robot_loop()
    node.update_robot_loop()

    assert node.pub_connected.messages == [True, True, True]  # connect() 1회 + loop 2회


def test_work_area_register_is_raw_mm():
    """work_area는 0.1mm 아니라 정수 mm 그대로 저장된다."""
    registers = register_map.load()
    entry = registers.write_entry("work_area")

    assert entry.address == 256
    # 256~264: 호 길이, 높이, 스캐너 높이, 겹침, 반지름, 두께, EOAT 가로/세로, EOAT 종류.
    assert entry.count == 9
    assert entry.kind == "raw"
    assert registers.scales_for(entry) == [1.0] * 9
    assert registers.write_entry("param_src").address == 266


def test_work_area_write_sets_param_src_flag():
    """작업 영역을 쓰면 256~259와 param_src(266)=1이 함께 나가야 한다."""
    node = make_node(map_data={
        "scale": {"position_per_count": 0.1, "rotation_per_count": 1.0},
        "read": {},
        "write": {
            "work_area": {"address": 256, "count": 4, "kind": "raw"},
            "param_src": {"address": 266, "count": 1},
        },
    })

    node._write_pose_topic(
        "work_area", FakeMessageList([600.0, 800.0, 150.0, 20.0]), flag_name="param_src"
    )

    assert node.robot_modbus.writes == [
        (256, 600), (257, 800), (258, 150), (259, 20), (266, 1),
    ]


def test_work_area_write_skipped_without_param_src_address():
    """flag 레지스터 주소가 없으면 값은 쓰되 플래그는 세우지 않는다."""
    node = make_node(map_data={
        "scale": {"position_per_count": 0.1, "rotation_per_count": 1.0},
        "read": {},
        "write": {"work_area": {"address": 256, "count": 4, "kind": "raw"}},
    })

    node._write_pose_topic(
        "work_area", FakeMessageList([600.0, 800.0, 150.0, 20.0]), flag_name="param_src"
    )

    assert node.robot_modbus.writes == [(256, 600), (257, 800), (258, 150), (259, 20)]
    assert node.get_logger().warnings


class FakeMessageList:
    def __init__(self, data):
        self.data = data


def test_connect_rebuilds_channels_when_ip_changes():
    """운영 UI가 robot_ip 파라미터를 바꾸면 그 주소로 다시 붙어야 한다.

    예전에는 노드를 띄울 때 읽은 주소로 만든 소켓을 계속 써서, UI에서
    IP를 아무리 고쳐도 옛 주소로만 붙었다(화면 표시와 실제 연결이 어긋남).
    """
    node = make_connectable_node()
    node.robot_ip = "10.0.0.9"
    built = []

    class _Chan:
        def __init__(self, *args):
            built.append(args)

        def connect(self):
            return True

        connect_29999 = connect_30001 = connect

        def disconnect(self):
            return True

        disconnect_29999 = disconnect_30001 = disconnect

    with patch.multiple(
        "elite_robot_controller.robot.robot_control_node",
        Robot_29999=_Chan, Robot_30001=_Chan, Robot_modbus=_Chan,
    ):
        node.connect_all_servers("192.168.1.101")

    assert node.robot_ip == "192.168.1.101"
    assert built, "새 주소로 채널을 다시 만들지 않았다"
    assert all(args[0] == "192.168.1.101" for args in built)


def test_connect_keeps_channels_when_ip_is_unchanged():
    """같은 주소면 굳이 소켓을 새로 만들지 않는다(불필요한 끊김 방지)."""
    node = make_connectable_node()
    node.robot_ip = "10.0.0.9"
    dash_before = node.robot_dash

    assert node.connect_all_servers("10.0.0.9") is True
    assert node.robot_dash is dash_before


# ---- 연결 감시: 끊김 감지와 자동 재연결 ---------------------------------------------
def _linked_node(fail_reads):
    """연결된 노드. fail_reads 가 True 면 Modbus 읽기가 None 을 돌려준다(로봇 응답 없음)."""
    node = make_connectable_node()
    assert node.connect_all_servers() is True
    for name in ("pub_robot_mode", "pub_control_method", "pub_op_mode", "pub_tcp_pose",
                 "pub_tcp_pose_zero", "pub_joint_position", "pub_scan_state", "pub_task_state",
                 "pub_home_flag", "pub_speed_scale", "pub_alarm"):
        setattr(node, name, FakePublisher())
    node.alarm_mgr = type("M", (), {"process": lambda self, a: False})()
    node.robot_primary.get_data = lambda: None
    node.robot_primary.alarm_queue = type("Q", (), {"empty": lambda self: True})()
    node.robot_modbus.get_register = (lambda address: None) if fail_reads else (lambda address: 5)
    return node


def test_link_is_dropped_after_consecutive_read_failures():
    """로봇이 응답을 안 하면 '연결됨'을 계속 내보내지 않고 끊김으로 바꾼다."""
    node = _linked_node(fail_reads=True)
    for _ in range(node.LINK_FAIL_LIMIT - 1):
        node.update_robot_loop()
    assert node.connected is True, "한두 번 실패로는 끊지 않는다"
    node.update_robot_loop()
    assert node.connected is False
    assert node.pub_connected.messages[-1] is False
    assert node._want_connected is True, "운영자가 끊은 게 아니므로 다시 붙는다"


def test_a_good_read_resets_the_failure_count():
    node = _linked_node(fail_reads=True)
    node.update_robot_loop()
    node.update_robot_loop()
    node.robot_modbus.get_register = lambda address: 5
    node.update_robot_loop()
    assert node._link_fails == 0 and node.connected is True


def test_auto_reconnect_only_when_the_operator_wants_it():
    node = _linked_node(fail_reads=True)
    for _ in range(node.LINK_FAIL_LIMIT):
        node.update_robot_loop()
    assert node.connected is False
    assert node._try_reconnect() is True and node.connected is True

    res = type("R", (), {})()
    node.cb_disconnect(None, res)
    assert node._want_connected is False and node.connected is False
    assert node._try_reconnect() is False and node.connected is False, "연결 해제 뒤에는 안 붙는다"
    node.cb_connect(None, res)
    assert node._want_connected is True and node.connected is True
