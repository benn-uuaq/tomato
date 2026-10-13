import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32, Int32MultiArray, Float32MultiArray
from std_srvs.srv import Trigger

from elite_robot_controller import register_map
# 방금 만든 robot.py 내부 모듈 정상 참조 조치
from elite_robot_controller.robot.robot_driver import Robot_30001, Robot_29999, Robot_modbus, AlarmManager

class RobotControlNode(Node):
    # 레지스터 주소는 config/modbus_registers.json에서만 관리한다.
    # 주소가 아직 없는 항목은 요청을 거부해 엉뚱한 레지스터에 쓰지 않는다.

    def __init__(self):
        super().__init__('robot_control_node')

        self.declare_parameter('robot_ip', '192.168.227.134')
        # False 면 기동해도 붙지 않고 UI 의 [연결] 을 기다린다. 운영자가 화면에서
        # IP 를 넣고 연결하는 구성(로봇 PC)에서 쓴다.
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('register_map', '')
        # 502는 리눅스에서 권한이 필요해 시뮬레이터 시험용으로 바꿀 수 있게 뺐다.
        self.declare_parameter('modbus_port', 502)
        # 조그 속도는 레지스터 307(속도 비율 %)을 그대로 쓴다. 단위가 달라서
        # 아래 값을 100 % 기준으로 두고 비율만큼 줄인다.
        #   speedj  qd [rad/s], a [rad/s^2]   (스크립트 매뉴얼 3.1.26)
        #   speedl  xd [m/s],   a [m/s^2]     (스크립트 매뉴얼 3.1.27)
        self.declare_parameter('jog_joint_speed_max', 0.50)    # rad/s
        self.declare_parameter('jog_tcp_speed_max', 0.10)      # m/s
        self.declare_parameter('jog_tcp_rot_speed_max', 0.50)  # rad/s
        self.declare_parameter('jog_accel_max', 1.00)          # speedj rad/s^2
        # TCP 조그(speedl) 가속도 [m/s^2]. 직선 동작 운영 기준 400 mm/s^2.
        self.declare_parameter('jog_tcp_accel_max', 0.40)
        # speedj/speedl 의 t. 이 시간이 지나면 로봇이 스스로 멈춘다.
        # UI는 버튼을 누른 순간 딱 한 번만 명령을 보낸다(2026-08-31 이전에는
        # held 동안 짧은 간격으로 되풀이했는데, 그때마다 로봇이 새 스크립트를
        # 실행하느라 STOPPED→RUNNING을 반복해 움직임이 덜컹거렸다). 실제
        # 정지는 손을 뗄 때 나가는 29999 stop이 맡으므로, 이 값은 "혹시 그
        # stop 신호가 아예 안 왔을 때"의 안전 타임아웃일 뿐이다 — 평소 조그
        # 조작을 방해하지 않을 만큼 넉넉하게 잡는다.
        self.declare_parameter('jog_hold_time', 3.0)
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        map_path = self.get_parameter('register_map').get_parameter_value().string_value
        self.registers = register_map.load(map_path or None)

        self.robot_ip = robot_ip
        self.robot_dash = Robot_29999(robot_ip, 29999)
        self.robot_primary = Robot_30001(robot_ip, 30001)
        modbus_port = self.get_parameter('modbus_port').get_parameter_value().integer_value
        self.robot_modbus = Robot_modbus(robot_ip, modbus_port)
        self.alarm_mgr = AlarmManager()
        self.connected = False
        # 연결 감시. 로봇 쪽 Modbus 읽기가 LINK_FAIL_LIMIT 주기 연속으로 실패하면
        # 끊긴 것으로 보고 connected 를 내린다. 운영자가 '연결 해제'를 누르지
        # 않았다면(_want_connected) 배경 스레드가 RECONNECT_S 마다 다시 붙는다.
        self._want_connected = True
        self._link_fails = 0
        self._connect_lock = threading.Lock()
        # 로봇 자체 속도 비율 [%]. 조그 속도도 이 값을 따른다.
        self.speed_ratio = 100

        # 발행할 토픽 정의
        self.pub_robot_mode = self.create_publisher(Int32, 'robot/status/robot_mode', 10)
        self.pub_control_method = self.create_publisher(Int32, 'robot/status/control_method', 10)
        self.pub_op_mode = self.create_publisher(Int32, 'robot/status/operation_mode', 10)
        self.pub_tcp_pose = self.create_publisher(Float32MultiArray, 'robot/status/tcp_pose', 10)
        self.pub_tcp_pose_zero = self.create_publisher(Float32MultiArray, 'robot/status/tcp_pose_zero', 10)
        self.pub_joint_position = self.create_publisher(Float32MultiArray, 'robot/status/joint_position', 10)
        self.pub_alarm = self.create_publisher(String, 'robot/status/alarms', 10)
        self.pub_connected = self.create_publisher(Bool, 'robot/status/connected', 10)
        # 로봇 태스크가 쓰는 스캔 진행 상태(290~298). 운영 UI의 격자 순회가
        # 이 값으로 한 셀의 완료(state==5 && finished==1)를 판정한다.
        self.pub_scan_state = self.create_publisher(Int32MultiArray, 'robot/status/scan_state', 10)
        # 로봇 컨트롤러가 주는 태스크 상태(1 실행 중, 2 일시 중지, 3 중지됨).
        # 29999로 물으면 매번 명령을 던져야 하므로 Modbus로 상시 읽는다.
        self.pub_task_state = self.create_publisher(Int32, 'robot/status/task_state', 10)
        # 로봇이 홈에 있는지(레지스터 276). 태스크가 제어주기마다 쓰고,
        # 태스크가 안 돌 때는 이 노드가 대신 관리한다(조그 0 / 홈 도착 1).
        # RCS 는 이 값과 태스크 상태를 같이 보고 차량·리프트를 움직인다.
        self.pub_home_flag = self.create_publisher(Int32, 'robot/status/at_home', 10)
        # 로봇이 실제로 쓰고 있는 속도 비율[%]. 펜던트에서 바꿔도 여기로 나온다.
        self.pub_speed_scale = self.create_publisher(Int32, 'robot/status/speed_scale', 10)

        # 연결과 해제도 서비스로 노출해 운영 UI에서 다룰 수 있게 한다.
        self.create_service(Trigger, 'robot/dashboard/connect', self.cb_connect)
        self.create_service(Trigger, 'robot/dashboard/disconnect', self.cb_disconnect)

        # 대시보드 명령 서비스 매핑
        self.create_service(Trigger, 'robot/dashboard/robot_mode', self.cb_dash_mode)
        self.create_service(Trigger, 'robot/dashboard/status', self.cb_dash_status)
        self.create_service(Trigger, 'robot/dashboard/power_on', self.cb_dash_power_on)
        self.create_service(Trigger, 'robot/dashboard/power_off', self.cb_dash_power_off)
        self.create_service(Trigger, 'robot/dashboard/brake_release', self.cb_dash_brake)
        self.create_service(Trigger, 'robot/dashboard/play', self.cb_dash_play)
        self.create_service(Trigger, 'robot/dashboard/remote_control_on',
                            self.cb_dash_remote_on)
        self.create_service(Trigger, 'robot/dashboard/pause', self.cb_dash_pause)
        self.create_service(Trigger, 'robot/dashboard/stop', self.cb_dash_stop)
        # 태스크는 펜던트/로봇 쪽에서 고정이라 운영 UI가 고르지 않는다 —
        # 지금 뭐가 올라가 있는지만 29999 "task -s"로 물어 보여준다.
        self.create_service(Trigger, 'robot/dashboard/task_status', self.cb_dash_task_status)

        # 홈 이동은 값이 없는 한 번짜리 명령이므로 Trigger를 쓴다. 기준 위치
        # 저장은 UI가 값을 보내므로 아래 토픽으로 받는다.
        self.create_service(Trigger, 'robot/command/move_home', self.cb_move_home)

        # 값이 있는 명령은 토픽으로 받는다.
        # 작업 속도는 mm/s, 속도 비율은 2~100 [%]이며 레지스터에 쓴다.
        self.create_subscription(Int32, 'robot/command/linear_speed', self.cb_linear_speed, 10)
        self.create_subscription(Int32, 'robot/command/speed_ratio', self.cb_speed_ratio, 10)
        # 조그는 레지스터가 아니라 30001 스크립트로 처리한다.
        # 값의 부호가 방향, 절댓값이 축 번호(1~6)이며 0은 정지다.
        self.create_subscription(Int32, 'robot/command/jog_joint', self.cb_jog_joint, 10)
        self.create_subscription(Int32, 'robot/command/jog_tcp', self.cb_jog_tcp, 10)
        # 30001 스크립트 중계. 픽앤플레이스 시퀀스 노드처럼 임의의 자세로
        # 움직여야 하는 쪽이 쓴다. 소켓은 이 노드가 혼자 들고 있어야
        # 명령이 섞이지 않으므로, 바깥에서는 문자열만 보내고 전송은 여기서 한다.
        self.create_subscription(String, 'robot/command/script', self.cb_script, 10)

        # 기준 위치는 6개 레지스터에 한 번에 쓴다.
        self.create_subscription(Float32MultiArray, 'robot/command/home_joint', self.cb_home_joint, 10)
        self.create_subscription(Float32MultiArray, 'robot/command/start_pose', self.cb_start_pose, 10)

        # 작업 영역: [너비, 높이, 스캐너높이, 겹침] mm. MQTT job_cmd의 grid
        # 블록이 UI를 거쳐 여기로 온다. 256~259에 쓰고 266(param_src)을
        # 1로 세워야 태스크가 이 값을 읽는다.
        self.create_subscription(Float32MultiArray, 'robot/command/work_area', self.cb_work_area, 10)
        # 스캔 시작 허가(267). 로봇이 원점에서 멈춰 기다리는 것을 풀어 준다.
        self.create_subscription(Int32, 'robot/command/scan_go', self.cb_scan_go, 10)
        # 차량 고정 확인(309). 차량이 서고 아웃트리거가 고정되고 리프트가
        # 멈췄다고 RCS 가 확인해 주면 1, 하나라도 움직이면 0 이다. 로봇
        # 태스크는 이 값이 1 이어야 움직인다(dus5_init / dus5_goto_zero).
        self.create_subscription(
            Int32, 'robot/command/vehicle_ready', self.cb_vehicle_ready, 10)
        # 마킹 자리 [u, v] mm (268~269). 마킹 태스크를 틀기 전에 쓴다.
        self.create_subscription(
            Float32MultiArray, 'robot/command/mark_target', self.cb_mark_target, 10)
        # 태스크 바꿔 끼우기. 마킹은 스캔과 다른 태스크라 29999 로 불러온다.
        # 경로는 컨트롤러 안의 실제 위치라 현장에서 한 번 확인해야 한다.
        self.declare_parameter('mark_task_path', 'Dusan/dusan_v4/dusan_v4_mark.task')
        self.declare_parameter('scan_task_path', 'Dusan/dusan_v4/dusan_v4.task')
        self.create_service(Trigger, 'robot/dashboard/load_mark_task', self.cb_load_mark_task)
        self.create_service(Trigger, 'robot/dashboard/load_scan_task', self.cb_load_scan_task)

        # 10Hz 주기로 모드버스 데이터 갱신 및 30001 알람 수집
        self.timer = self.create_timer(0.1, self.update_robot_loop)
        self.get_logger().info("[DEBUG]] ELITE Robot 제어 ROS2 노드가 활성화되었습니다.")

        # 기동 시 한 번 붙어 본다. 실패해도 노드는 살아 있어야 운영 UI에서
        # 연결 버튼을 쓸 수 있다. auto_connect 가 False 면 붙지 않고 기다린다
        # — 배경 재연결도 운영자가 [연결] 을 누른 뒤에야 돈다.
        if self.get_parameter('auto_connect').value:
            if not self.connect_all_servers(robot_ip):
                self.get_logger().warn(
                    f"[WARN] 로봇에 연결하지 못했습니다 — {self.RECONNECT_S:g}초마다 다시 시도합니다."
                )
        else:
            self._want_connected = False
            self.publish_connected()
            self.get_logger().info("[connect] UI 에서 로봇 IP 를 넣고 [연결] 을 누르면 붙습니다.")
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop, name="robot-reconnect", daemon=True)
        self._reconnect_thread.start()

    #: Modbus 읽기가 이만큼 연속 실패하면 끊긴 것으로 본다(주기 0.1 s + 읽기 대기).
    LINK_FAIL_LIMIT = 3
    #: 끊긴 뒤 다시 붙어 보는 간격 [s].
    RECONNECT_S = 2.0

    def _reconnect_loop(self):
        """끊겨 있고 운영자가 연결을 원하면 주기적으로 다시 붙는다.

        타이머 콜백(update_robot_loop)에서 붙으면 연결 시도 동안(소켓마다
        수 초) 노드의 다른 서비스·토픽이 모두 멈춘다. 그래서 따로 돈다.
        """
        while rclpy.ok():
            time.sleep(self.RECONNECT_S)
            self._try_reconnect()

    def _try_reconnect(self):
        """끊겨 있고 연결을 원하면 한 번 붙어 본다. 붙으면 True."""
        if self.connected or not self._want_connected:
            return False
        if not self._connect_lock.acquire(blocking=False):
            return False            # 운영자가 누른 connect 가 이미 붙는 중
        try:
            if self.connected or not self._want_connected:
                return False
            self.disconnect_all_servers(quiet=True)
            if self._connect_unlocked():
                self.get_logger().info(f"[connect] {self.robot_ip} 다시 연결됨")
                return True
            return False
        finally:
            self._connect_lock.release()

    def _param_str(self, name, fallback):
        """파라미터를 못 읽어도 연결 자체는 진행한다(기존 주소를 그대로 쓴다)."""
        try:
            return self.get_parameter(name).get_parameter_value().string_value or fallback
        except Exception:
            return fallback

    def _param_int(self, name, fallback):
        try:
            return self.get_parameter(name).get_parameter_value().integer_value or fallback
        except Exception:
            return fallback

    def connect_all_servers(self, robot_ip=None):
        with self._connect_lock:
            return self._connect_unlocked(robot_ip)

    def _connect_unlocked(self, robot_ip=None):
        """세 채널을 모두 연결한다. 하나라도 실패하면 연결로 보지 않는다.

        연결할 때마다 `robot_ip` 파라미터를 다시 읽는다 — 운영 UI가 화면에서
        주소를 고친 뒤 이 파라미터를 바꾸고 connect 를 부르면 **그 주소로**
        붙게 하기 위해서다. 예전에는 노드를 띄울 때 읽은 주소로 소켓을
        만들어 두고 인자도 무시해서, UI에서 IP를 아무리 바꿔도 노드는 계속
        옛 주소로 붙었다(화면 표시와 실제 연결이 어긋나던 원인).
        """
        if robot_ip is None:
            robot_ip = self._param_str('robot_ip', self.robot_ip)
        if robot_ip and robot_ip != self.robot_ip:
            self.get_logger().info(f"[connect] 주소 변경: {self.robot_ip} -> {robot_ip}")
            self.disconnect_all_servers()
            modbus_port = self._param_int('modbus_port', 502)
            self.robot_ip = robot_ip
            self.robot_dash = Robot_29999(robot_ip, 29999)
            self.robot_primary = Robot_30001(robot_ip, 30001)
            self.robot_modbus = Robot_modbus(robot_ip, modbus_port)
        try:
            d_ok = self.robot_dash.connect_29999()
            p_ok = self.robot_primary.connect_30001()
            m_ok = self.robot_modbus.connect()
            self.connected = bool(d_ok and p_ok and m_ok)
            self._link_fails = 0
        except Exception as e:
            self.get_logger().error(f"[ERROR] 소켓 연결 중 예외 발생: {e}")
            self.connected = False
        self.publish_connected()
        return self.connected

    def disconnect_all_servers(self, quiet=False):
        """세 채널을 정리한다. 개별 실패는 남은 채널 정리를 막지 않는다."""
        for close in (self.robot_dash.disconnect_29999,
                      self.robot_primary.disconnect_30001,
                      self.robot_modbus.disconnect):
            try:
                close()
            except Exception as e:
                if not quiet:
                    self.get_logger().warn(f"[WARN] 연결 해제 중 예외: {e}")
        self.connected = False
        self.publish_connected()
        return True

    def publish_connected(self):
        """세 채널 연결 여부를 알린다. 운영 UI가 이 값으로 상태를 표시한다."""
        self.pub_connected.publish(Bool(data=self.connected))

    def cb_connect(self, req, res):
        # 운영자가 연결을 원한다 — 지금 실패해도 배경에서 계속 다시 붙는다.
        self._want_connected = True
        ok = self.connect_all_servers()
        res.success = ok
        res.message = (
            f"[connect] {self.robot_ip} 연결됨" if ok
            else f"[connect] {self.robot_ip} 연결 실패"
        )
        if not ok:
            self.get_logger().warn(res.message)
        return res

    def cb_disconnect(self, req, res):
        # 운영자가 끊었다 — 자동으로 다시 붙지 않는다.
        self._want_connected = False
        self.disconnect_all_servers()
        res.success = True
        res.message = "[disconnect] 연결을 해제했습니다."
        return res

    def update_robot_loop(self):
        # 매 주기 발행한다. 상태가 바뀔 때만 보내면 UI가 나중에 구독했을 때
        # (또는 재시작했을 때) 이미 지나간 값을 영영 받지 못한다.
        self.publish_connected()
        # 연결 전에 소켓을 건드리면 예외가 난다.
        if not self.connected:
            return

        # 첫 읽기를 생존 확인으로 쓴다. 실패하면 이번 주기의 나머지 읽기는
        # 건너뛴다 — 끊긴 채로 열 번을 다 읽으면 번마다 타임아웃(1 s)을 기다려
        # 노드가 몇 초씩 멈추고, 그 사이 '연결됨'을 계속 내보내게 된다.
        if not self.publish_code('robot_mode', self.pub_robot_mode):
            self._link_fails += 1
            if self._link_fails >= self.LINK_FAIL_LIMIT:
                self._link_lost()
            return
        self._link_fails = 0
        self.publish_code('control_method', self.pub_control_method)
        self.publish_code('operation_mode', self.pub_op_mode)
        self.publish_pose('tcp_absolute', self.pub_tcp_pose)
        self.publish_pose('tcp_zero_relative', self.pub_tcp_pose_zero)
        self.publish_pose('joint_position', self.pub_joint_position)
        self.publish_raw('scan_state', self.pub_scan_state)
        self.publish_code('task_state', self.pub_task_state)
        self.publish_code('home_flag', self.pub_home_flag)
        self.publish_code('speed_scale', self.pub_speed_scale)

        # 30001 포트 비동기 백그라운드 실시간 알람 스트림 처리
        self.robot_primary.get_data()
        while not self.robot_primary.alarm_queue.empty():
            alarm = self.robot_primary.alarm_queue.get()
            if self.alarm_mgr.process(alarm):
                alarm_msg = String()
                alarm_msg.data = f"[ALARM] {alarm.msg}" if alarm.msg else f"[ALARM CODE] E{alarm.code} S{alarm.sub}"
                self.pub_alarm.publish(alarm_msg)

    def _link_lost(self):
        """로봇 응답이 끊겼다. 연결 끊김을 알리고 소켓을 정리한다(재연결은 배경 스레드)."""
        self.get_logger().warn(
            f"[connect] {self.robot_ip} 응답 없음({self._link_fails}회 연속) — 연결 끊김. "
            f"{self.RECONNECT_S:g}초마다 다시 연결합니다.")
        self._link_fails = 0
        self.disconnect_all_servers(quiet=True)

    def publish_code(self, name, publisher):
        """레지스터 한 개를 읽어 그대로 발행한다. 읽었으면 True."""
        entry = self.registers.read_entry(name)
        if not entry.available:
            return True
        try:
            value = self.robot_modbus.get_register(entry.address)
        except Exception:
            value = None
        if value is None:
            return False
        publisher.publish(Int32(data=value))
        return True

    def publish_pose(self, name, publisher):
        """자세 레지스터 6개를 읽어 [X, Y, Z, Rx, Ry, Rz]로 발행한다.

        get_all_registers가 부호 있는 16비트로 변환해 주므로 여기서는
        단위 환산만 한다. X, Y, Z는 mm, Rx, Ry, Rz는 mrad이다.
        """
        entry = self.registers.read_entry(name)
        if not entry.available:
            return

        regs = self.robot_modbus.get_all_registers(entry.address, entry.count)
        if not regs or len(regs) != entry.count:
            return

        scales = self.registers.scales_for(entry)
        pose_msg = Float32MultiArray()
        pose_msg.data = [value * scale for value, scale in zip(regs, scales)]
        publisher.publish(pose_msg)

    def publish_raw(self, name, publisher):
        """레지스터 여러 개를 환산 없이 정수 그대로 발행한다.

        스캔 진행 상태처럼 단위가 없는 값(상태 코드, 개수, 플래그)에 쓴다.
        환산을 거치면 부동소수 오차로 == 비교가 어긋날 수 있다.
        """
        entry = self.registers.read_entry(name)
        if not entry.available:
            return

        regs = self.robot_modbus.get_all_registers(entry.address, entry.count)
        if not regs or len(regs) != entry.count:
            return

        publisher.publish(Int32MultiArray(data=[int(value) for value in regs]))

    def write_register(self, name, value):
        """쓰기 레지스터에 값을 넣는다. (성공여부, 안내문구)를 돌려준다.

        주소가 정해지지 않은 항목은 시도하지 않는다. 엉뚱한 레지스터에
        쓰면 로봇이 예기치 않게 움직일 수 있다.
        """
        entry = self.registers.write_entry(name)
        if not entry.available:
            return False, f"[{name}] Modbus 주소가 설정되지 않았습니다."
        try:
            ok = self.robot_modbus.set_register(entry.address, value)
        except Exception as exc:
            return False, f"[{name}] 레지스터 쓰기 실패: {exc}"
        if not ok:
            return False, f"[{name}] 레지스터 {entry.address} 쓰기를 확인하지 못했습니다."
        return True, f"[{name}] 레지스터 {entry.address} <- {value}"

    def _write_service(self, name, value, response):
        """Trigger 서비스 응답에 쓰기 결과를 채운다."""
        success, message = self.write_register(name, value)
        if not success:
            self.get_logger().warn(message)
        response.success = success
        response.message = message
        return response

    def _write_topic(self, name, msg):
        """토픽으로 받은 값을 레지스터에 쓰고 실패만 기록한다."""
        success, message = self.write_register(name, int(msg.data))
        if not success:
            self.get_logger().warn(message)

    def cb_linear_speed(self, msg):
        self._write_topic('linear_speed', msg)

    def cb_speed_ratio(self, msg):
        """로봇 전체 동작 속도 비율[%]을 바꾼다. 2~100.

        29999로 실시간 반영하는 것이 본 경로다. 레지스터 307은 태스크가
        시작할 때 읽어 가는 값이라 함께 써 둔다(태스크 재시작 후에도 유지).
        조그 속도도 이 값을 그대로 곱해 줄인다.
        """
        self.speed_ratio = max(2, min(100, int(msg.data)))
        # 실시간 반영: 돌고 있는 동작에도 바로 먹는다.
        result = self.robot_dash.robot_set_speed(self.speed_ratio)
        if result is None:
            self.get_logger().warn(
                f"[speed] 속도 비율 {self.speed_ratio} % 전송 실패")
        elif any(word in str(result).lower() for word in self._DASH_REFUSALS):
            # 답은 왔는데 거절한 경우(명령 형식이 틀렸거나 로컬 모드 등).
            self.get_logger().warn(
                f"[speed] 컨트롤러가 속도 비율 {self.speed_ratio} % 를 거절했습니다: {result}")
        self._write_topic('speed_ratio', Int32(data=self.speed_ratio))

    def _jog_scale(self):
        """속도 비율(2~100 %)을 배율로 바꾼다."""
        return self.speed_ratio / 100.0

    @staticmethod
    def _round6(values):
        """부동소수 잡음을 없앤다. 스크립트 문자열로 나가기 때문이다."""
        return [round(float(v), 6) for v in values]

    def write_pose(self, name, values):
        """자세 6개를 연속 레지스터에 쓴다. 단위 환산은 읽기와 반대로 한다."""
        entry = self.registers.write_entry(name)
        if not entry.available:
            return False, f"[{name}] Modbus 주소가 설정되지 않았습니다."
        if len(values) != entry.count:
            return False, f"[{name}] 값이 {entry.count}개가 아닙니다: {len(values)}개"

        scales = self.registers.scales_for(entry)
        for offset, (value, scale) in enumerate(zip(values, scales)):
            raw = int(round(float(value) / scale))
            ok = self.robot_modbus.set_register(entry.address + offset, raw)
            if not ok:
                return False, f"[{name}] 레지스터 {entry.address + offset} 쓰기 실패"
        return True, f"[{name}] 레지스터 {entry.address}~{entry.address + entry.count - 1} 갱신"

    def _write_pose_topic(self, name, msg, flag_name='pose_src'):
        """값 N개를 레지스터에 쓰고, 태스크가 읽도록 플래그 레지스터를 1로 세운다.

        home_joint/start_pose는 pose_src(308)를, work_area는 param_src(266)를
        쓴다. 플래그 이름이 다를 뿐 동작은 같다.
        """
        success, message = self.write_pose(name, list(msg.data))
        if not success:
            self.get_logger().warn(message)
            return
        flag_entry = self.registers.write_entry(flag_name)
        if flag_entry.available:
            self.robot_modbus.set_register(flag_entry.address, 1)
        else:
            self.get_logger().warn(f"[{name}] {flag_name} 주소가 설정되지 않았습니다.")

    def cb_home_joint(self, msg):
        self._write_pose_topic('home_joint', msg)

    def cb_start_pose(self, msg):
        self._write_pose_topic('start_pose', msg)

    def cb_work_area(self, msg):
        # [너비, 높이, 스캐너높이, 겹침] mm -> 256~259, 성공하면 266=1.
        self._write_pose_topic('work_area', msg, flag_name='param_src')

    def cb_mark_target(self, msg):
        """마킹 자리 [u, v] (mm)를 268~269 에 쓴다."""
        success, message = self.write_pose('mark_target', list(msg.data))
        if not success:
            self.get_logger().warn(message)

    def _load_task(self, param_name, response):
        """29999 `task -p <경로>` 로 태스크를 불러온다.

        경로는 파라미터(mark_task_path / scan_task_path)다. 센서가 없을 때는
        scan_task_path 를 dusan_v4_nosensor_seq.task, mark_task_path 를
        dusan_v4_nosensor_mark.task 로 바꿔 띄운다.
        """
        path = self._param_str(param_name, '')
        if not path:
            response.success = False
            response.message = f"[task -p] {param_name} 파라미터가 비어 있습니다."
            return response
        return self._execute_dash_cmd(
            lambda: self.robot_dash.send_command_29999(f"task -p {path}"),
            f"task -p {path}", response)

    def cb_load_mark_task(self, req, res):
        return self._load_task('mark_task_path', res)

    def cb_load_scan_task(self, req, res):
        return self._load_task('scan_task_path', res)

    def cb_scan_go(self, msg):
        """스캔 시작 허가(267)를 쓴다.

        로봇은 원점에 도착하면 state(290)를 7 로 두고 이 값이 1 이 될
        때까지 멈춰 선다. 통과하면서 로봇이 스스로 0 으로 되돌리므로
        여기서 지울 필요는 없다.
        """
        self._write_topic('scan_go', msg)

    # ---------------------------------------------------------------- 조그
    def _jog_vector(self, code, count=6):
        """부호는 방향, 절댓값은 축 번호(1~6)인 코드를 속도 벡터로 바꾼다."""
        axis = abs(int(code)) - 1
        if axis < 0 or axis >= count:
            return None
        vector = [0.0] * count
        vector[axis] = 1.0 if code > 0 else -1.0
        return vector

    def _jog_stop(self):
        """29999 stop으로 즉시 멈춘다. 조그는 눌린 동안만 움직여야 한다."""
        self.robot_dash.robot_stop()

    def cb_vehicle_ready(self, msg):
        """차량 고정 확인을 레지스터 309 에 그대로 전한다(1 고정 / 0 아님)."""
        self._write_topic('vehicle_ready', Int32(data=1 if int(msg.data) else 0))

    def _leaving_home(self):
        """로봇을 홈 밖으로 움직이기 직전에 홈 플래그를 내린다.

        태스크가 돌 때는 태스크의 발행 스레드가 제어주기마다 276 을
        갱신하지만, 조그·홈 이동은 태스크 **밖**에서 30001 스크립트로
        돌기 때문에 그동안은 아무도 값을 갱신하지 않는다. 그대로 두면
        홈에서 조그로 빠져나가도 플래그가 1 로 남아, RCS 가 로봇이 홈에
        있는 줄 알고 차량을 움직인다.
        """
        if self.connected:
            self.write_register('home_flag', 0)

    def cb_jog_joint(self, msg):
        if not self.connected:
            return
        if int(msg.data) == 0:
            self._jog_stop()
            return
        unit = self._jog_vector(msg.data)
        if unit is None:
            self.get_logger().warn(f"[jog_joint] 축 번호가 범위를 벗어났습니다: {msg.data}")
            return
        scale = self._jog_scale()
        speed = self.get_parameter('jog_joint_speed_max').value * scale
        accel = self.get_parameter('jog_accel_max').value * scale
        qd = self._round6([value * speed for value in unit])
        hold = self.get_parameter('jog_hold_time').value
        self._leaving_home()
        self.robot_primary.send_script(f"speedj({qd}, {round(accel, 6)}, {hold})")

    def cb_jog_tcp(self, msg):
        if not self.connected:
            return
        if int(msg.data) == 0:
            self._jog_stop()
            return
        unit = self._jog_vector(msg.data)
        if unit is None:
            self.get_logger().warn(f"[jog_tcp] 축 번호가 범위를 벗어났습니다: {msg.data}")
            return
        # 앞 3개는 직선 속도, 뒤 3개는 회전 속도라 단위가 다르다.
        scale = self._jog_scale()
        linear = self.get_parameter('jog_tcp_speed_max').value * scale
        angular = self.get_parameter('jog_tcp_rot_speed_max').value * scale
        accel = self.get_parameter('jog_tcp_accel_max').value * scale
        xd = self._round6([
            value * (linear if index < 3 else angular)
            for index, value in enumerate(unit)
        ])
        hold = self.get_parameter('jog_hold_time').value
        self._leaving_home()
        self.robot_primary.send_script(f"speedl({xd}, {round(accel, 6)}, {hold})")

    def cb_script(self, msg):
        """받은 스크립트를 그대로 30001 로 보낸다.

        로봇을 움직이는 명령이므로 연결되어 있을 때만 보내고, 홈 플래그를
        먼저 내린다(조그·홈 이동과 같은 이유 — 태스크 밖에서 움직이는
        동안에는 아무도 276 을 갱신하지 않는다).
        """
        script = str(msg.data).strip()
        if not script:
            return
        if not self.connected:
            self.get_logger().warn("[script] 로봇에 연결되어 있지 않아 보내지 않았습니다.")
            return
        self._leaving_home()
        if not self.robot_primary.send_script(script):
            self.get_logger().warn(f"[script] 전송 실패: {script}")

    # get_variable이 실패했을 때만 쓰는 마지막 안전망. 정상 경로에서는
    # 항상 로봇에서 29999로 직접 읽은 값을 쓴다.
    _HOME_JOINT_FALLBACK = [0.79941, -1.55029, -2.70026, 1.10839, 0.81203, -3.14139]
    _HOME_POSE_FALLBACK = [0.40795, 0.05529, 0.40515, 1.57121, 0.00018, 1.61144]

    def cb_move_home(self, req, res):
        """안전 높이까지 movel로 올린 뒤 movej로 홈 관절값에 간다.

        30001 소켓(실시간 명령 채널)에 if/else 를 최상위(top-level)로
        그냥 보내면 한 줄씩 별개 명령으로 읽혀 실패한다("No 'if' command").
        def 이름(): ... end 로 감싼 하나의 프로그램으로 보내야 하고,
        보내고 나면 파싱이 끝나는 대로 바로 실행되므로 따로 호출하면
        안 된다("move_home_now 정의 안 됨"). 이 프로그램은 태스크
        preamble 밖에서 독립적으로 도는 별개 컨텍스트라 태스크가 저장해
        둔 Home_joint/Home_pose 전역도 그냥은 안 보여서("정의되지
        않았습니다") 쓸 수 없다. 대신 29999의 `variable -get`으로 로봇에
        저장된 실제 값을 그때그때 읽어와, global 선언 뒤 이 스크립트
        안에서 그 값을 직접 대입해 준다 — 하드코딩해 두면 태스크 쪽
        값이 바뀔 때 둘이 어긋날 수 있어서다.

        레지스터 308(pose_src)이 1일 때만 310~315 override 값을 쓰고,
        평소(0)에는 위에서 읽어온 Home_joint/Home_pose 를 쓴다. 예전에는
        항상 310~315를 읽었는데, 운영자가 '위치 저장'으로 한 번도 값을
        넣지 않으면 그 레지스터가 0으로 남아 있어 movej([0,0,0,0,0,0],
        ...) 처럼 전혀 엉뚱한 관절로 가 버렸다 — 그게 '고장 홈 위치로
        간다'로 보인 원인이다.
        """
        if not self.connected:
            res.success = False
            res.message = "[move_home] 로봇에 연결되어 있지 않습니다."
            return res

        home_joint = self.robot_dash.get_variable("Home_joint")
        home_pose = self.robot_dash.get_variable("Home_pose")
        # 벽에서 먼저 물러날 거리 [m]. 태스크가 쓰는 probe_dist(mm)와 같다.
        probe_dist = self.robot_dash.get_variable("probe_dist")
        if not isinstance(probe_dist, (int, float)) or probe_dist <= 0:
            probe_dist = self._HOME_RETREAT_FALLBACK_MM
        retreat_m = round(float(probe_dist) / 1000.0, 4)
        if not isinstance(home_joint, list) or len(home_joint) != 6:
            self.get_logger().warn(
                f"[move_home] Home_joint를 로봇에서 못 읽었습니다({home_joint!r}). "
                "고정값으로 대신합니다.")
            home_joint = self._HOME_JOINT_FALLBACK
        if not isinstance(home_pose, list) or len(home_pose) != 6:
            self.get_logger().warn(
                f"[move_home] Home_pose를 로봇에서 못 읽었습니다({home_pose!r}). "
                "고정값으로 대신합니다.")
            home_pose = self._HOME_POSE_FALLBACK

        # p, j 는 pose/joint 리터럴(p[..], j[..])에 쓰는 내장 이름과
        # 충돌한다("변수 p가 내장 함수의 이름과 충돌"). 겹치지 않는
        # 이름으로 바꾼다.
        # 30001은 def 이름(): ... end 블록 자체를 프로그램으로 받아
        # 파싱이 끝나면 바로 실행한다 — 따로 move_home_now() 를 호출하면
        # 그 호출을 다시 "정의 안 된 변수"로 오해해서 실패한다.
        script = (
            "def move_home_now():\n"
            "  global Home_joint\n"
            f"  Home_joint = {home_joint}\n"
            "  global Home_pose\n"
            f"  Home_pose = {home_pose}\n"
            "  if (read_port_register(308, True) == 1):\n"
            "    tgt_j = [read_port_register(310, True) / 1000.0,\n"
            "             read_port_register(311, True) / 1000.0,\n"
            "             read_port_register(312, True) / 1000.0,\n"
            "             read_port_register(313, True) / 1000.0,\n"
            "             read_port_register(314, True) / 1000.0,\n"
            "             read_port_register(315, True) / 1000.0]\n"
            "    tgt_h = get_forward_kin(tgt_j)\n"
            "  else:\n"
            "    tgt_j = Home_joint\n"
            "    tgt_h = Home_pose\n"
            "  end\n"
            # 벽에 붙어 있을 수 있다(스캔·마킹 도중 멈춘 경우). 곧장 올라가면
            # 프로브가 곡면을 긁고 올라가므로 **먼저 TCP -Z 로 물러난다.**
            # 다만 홈 자세보다 더 뒤로는 가지 않는다 — 홈이 지금 자리에서
            # TCP 축으로 얼마나 뒤에 있는지(hm_rel[2])만큼만, 최대 probe_dist.
            # 이미 홈보다 뒤에 있거나 홈에 있으면 물러나지 않는다.
            "  cur_pose = get_actual_tcp_pose()\n"
            "  hm_rel = pose_trans(pose_inv(cur_pose), tgt_h)\n"
            "  hm_back = -hm_rel[2]\n"
            f"  if (hm_back > {retreat_m}):\n"
            f"    hm_back = {retreat_m}\n"
            "  end\n"
            "  if (hm_back > 0.001):\n"
            "    movel(pose_trans(cur_pose, [0, 0, -hm_back, 0, 0, 0]), a=0.4, v=0.1)\n"
            "  end\n"
            "  cur_pose = get_actual_tcp_pose()\n"
            "  if (cur_pose[2] < tgt_h[2] - 0.001):\n"
            "    cur_pose[2] = tgt_h[2]\n"
            "    movel(cur_pose, a=0.4, v=0.1)\n"
            "  end\n"
            "  movej(tgt_j, a=1.4, v=0.5)\n"
            # 홈에 닿았다 — 레지스터 276 을 세운다. 태스크가 돌지 않는
            # 동안에는 이 스크립트 말고 아무도 이 값을 갱신하지 않는다.
            "  write_port_register(276, 1)\n"
            "end"
        )
        # 돌고 있는 태스크(스캔·마킹)를 **먼저 세운다.** 태스크가 도는 채로
        # 30001 로 스크립트를 보내면 컨트롤러가 거부하거나 태스크와 뒤섞인다.
        # 홈 이동은 언제 불러도(ERUT·RCS) 따로 돌아야 한다.
        stop_reply = self.robot_dash.robot_stop()
        if stop_reply is None:
            self.get_logger().warn(
                f"[move_home] 태스크 정지 응답 없음 — {self.robot_dash.last_error}")
        time.sleep(self._HOME_AFTER_STOP_S)
        # 홈으로 가는 **동안**은 홈이 아니다. 태스크를 세운 뒤에 내린다 —
        # 먼저 내리면 아직 도는 태스크의 발행 스레드가 다시 덮어쓴다.
        self._leaving_home()
        ok = self.robot_primary.send_script(script)
        res.success = bool(ok)
        res.message = "[move_home] 홈 이동을 요청했습니다." if ok else "[move_home] 스크립트 전송 실패"
        return res

    #: probe_dist 를 로봇에서 못 읽었을 때 쓰는 후퇴 한도 [mm].
    _HOME_RETREAT_FALLBACK_MM = 100.0

    #: 태스크를 세운 뒤 스크립트를 보내기까지 기다리는 시간 [s].
    #: 정지가 끝나기 전에 보내면 컨트롤러가 "실행 중"으로 거부한다.
    _HOME_AFTER_STOP_S = 0.5

    #: 대시보드가 **답은 했지만 거절한** 응답에 들어가는 말들.
    #: 예) "Failed to execute: play", "... not supported in local control mode"
    _DASH_REFUSALS = ("fail", "not supported", "error", "not allowed", "can not", "cannot")

    def _execute_dash_cmd(self, func, name, response):
        """대시보드 명령을 보내고 결과를 서비스 응답에 담는다.

        예전에는 응답이 **오기만 하면** 성공으로 쳤다 — 컨트롤러가
        "Failed to execute: play" 로 거절해도 RCS 는 성공으로 받았다.
        그리고 연결이 끊겨 None 이 오면 "Result: None" 만 남아 원인을
        알 수 없었다. 이제 거절 문구면 실패로, None 이면 끊긴 사유를
        함께 돌려준다.
        """
        res_str = func()
        if res_str is None:
            reason = getattr(self.robot_dash, "last_error", "") or "응답 없음"
            response.success = False
            response.message = f"[{name}] 대시보드 응답 없음 — {reason}"
            self.get_logger().warn(response.message)
            return response
        refused = any(word in res_str.lower() for word in self._DASH_REFUSALS)
        response.success = not refused
        response.message = f"[{name}] Result: {res_str}"
        if refused:
            self.get_logger().warn(f"[{name}] 컨트롤러가 거절: {res_str}")
        return response

    def cb_dash_mode(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_mode, "robotMode", res)
    def cb_dash_status(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_status, "status", res)
    def cb_dash_power_on(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_power_on, "robotControl -on", res)
    def cb_dash_power_off(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_power_off, "robotControl -off", res)
    def cb_dash_brake(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_brakeRelease, "brakeRelease", res)
    def cb_dash_play(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_play, "play", res)

    def cb_dash_remote_on(self, req, res):
        """원격 제어 모드를 켠다. 이걸 켜야 play/stop 이 먹는다."""
        return self._execute_dash_cmd(
            self.robot_dash.robot_remote_control_on, "remoteControl -on", res)
    def cb_dash_pause(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_pause, "pause", res)
    def cb_dash_stop(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_stop, "stop", res)
    def cb_dash_task_status(self, req, res): return self._execute_dash_cmd(self.robot_dash.robot_task_status, "task -s", res)

    def destroy_node(self):
        self.robot_dash.disconnect_29999()
        self.robot_primary.disconnect_30001()
        self.robot_modbus.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotControlNode()
        rclpy.spin(node)
    except SystemExit: 
        pass
    except KeyboardInterrupt: pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()