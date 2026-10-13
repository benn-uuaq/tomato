import struct
import socket
import select
import time
import queue
import threading
from pyModbusTCP.client import ModbusClient

DEFAULT_TIMEOUT = 10.0
MESSAGE_TYPE_ROBOT_STATE = 16
MESSAGE_TYPE_ROBOT_MESSAGE = 20

FMT_HEADER = 'IB'
FMT_ROBOT_MODE = 'IBQ???????BBdddB??I'
FMT_JOINT_HEADER = 'IB'     
FMT_JOINT_DATA = 'dddiiiffffBI'
FMT_CARTESIAN = 'IBdddddddddddd'
FMT_CONFIG = 'IB'+'dd'*6+'dd'*6+'ddddd'+'d'*6+'d'*6+'d'*6+'d'*6+'IIIBBBB'
FMT_MASTERBOARD = 'IBIIBBBdddBBBdddffffB???B'
FMT_ADDITIONAL = 'IB????B'
FMT_TOOL = 'IBBBddfBffB'
FMT_SAFETY = 'IBIbBdddd'
FMT_TOOL_COMM = 'IB?III?Bff'

class RobotDataConfig():
    def __init__(self):
        self.names_pre = [
            'total_message_len', 'total_message_type',
            'mode_sub_len', 'mode_sub_type', 'timestamp', 'reserved_1', 'reserved_2',
            'is_robot_power_on', 'is_emergency_stopped', 'is_robot_protective_stopped',
            'is_task_running', 'is_task_paused', 'robot_mode', 'robot_control_mode',
            'target_speed_fraction', 'speed_scaling', 'target_speed_fraction_limit',
            'get_robot_speed_mode', 'reserved_3', 'is_in_package_mode', 'reserved_4',
            'joint_sub_len', 'joint_sub_type'
        ]
        self.names_joint = [
            'actual_joint', 'target_joint', 'actual_velocity', 
            'joint_reserved_1', 'joint_reserved_2', 'joint_reserved_3',
            'current', 'voltage', 'temperature', 'torques', 'mode', 'joint_reserved_4'
        ]
        self.names_post = [
            'cartesial_sub_len', 'cartesial_sub_type',
            'tcp_x', 'tcp_y', 'tcp_z', 'rot_x', 'rot_y', 'rot_z',
            'offset_px', 'offset_py', 'offset_pz', 'offset_rotx', 'offset_roty', 'offset_rotz',
            'configuration_sub_len', 'configuration_sub_type',
            'limit_min_joint_0', 'limit_max_joint_0', 'limit_min_joint_1', 'limit_max_joint_1',
            'limit_min_joint_2', 'limit_max_joint_2', 'limit_min_joint_3', 'limit_max_joint_3',
            'limit_min_joint_4', 'limit_max_joint_4', 'limit_min_joint_5', 'limit_max_joint_5',
            'max_velocity_joint_0', 'max_acc_joint_0', 'max_velocity_joint_1', 'max_acc_joint_1',
            'max_velocity_joint_2', 'max_acc_joint_2', 'max_velocity_joint_3', 'max_acc_joint_3',
            'max_velocity_joint_4', 'max_acc_joint_4', 'max_velocity_joint_5', 'max_acc_joint_5',
            'default_velocity_joint', 'default_acc_joint', 'default_tool_velocity', 'default_tool_acc', 'internal_use',
            'dh_a_joint_0', 'dh_a_joint_1', 'dh_a_joint_2', 'dh_a_joint_3', 'dh_a_joint_4', 'dh_a_joint_5',
            'dh_d_joint_0', 'dh_d_joint_1', 'dh_d_joint_2', 'dh_d_joint_3', 'dh_d_joint_4', 'dh_d_joint_5',
            'dh_alpha_joint_0', 'dh_alpha_joint_1', 'dh_alpha_joint_2', 'dh_alpha_joint_3', 'dh_alpha_joint_4', 'dh_alpha_joint_5',
            'dh_theta_joint_0', 'dh_theta_joint_1', 'dh_theta_joint_2', 'dh_theta_joint_3', 'dh_theta_joint_4', 'dh_theta_joint_5',
            'masterboard_version', 'control_box_type', 'robot_type', 'robot_structure', 'tool_io_type', 'reserved_cfg2', 'reserved_cfg3',
            'masterboard_sub_len', 'masterboard_sub_type',
            'digital_input_bits', 'digital_output_bits',
            'standard_analog_input_domain0', 'standard_analog_input_domain1', 'tool_analog_input_domain',
            'standard_analog_input_value0', 'standard_analog_input_value1', 'tool_analog_input_value',
            'standard_analog_output_domain0', 'standard_analog_output_domain1', 'tool_analog_output_domain',
            'standard_analog_output_value0', 'standard_analog_output_value1', 'tool_analog_output_value',
            'masterrbord_temperature', 'robot_voltage', 'robot_current', 'io_current',
            'safety_mode', 'is_robot_in_reduced_mode', 'operational_mode_selector_input',
            'threeposition_enabling_device_input', 'internal_use_mb',
            'additional_sub_len', 'additional_sub_type',
            'is_freedrive_button_pressed', 'reserved_add', 'is_freedrive_io_enabled', 'is_dynamic_collision_detect_enabled', 'reserved_add2',
            'tool_sub_len', 'tool_sub_type',
            'tool_analog_output_domain', 'tool_analog_input_domain', 'tool_analog_output_value', 'tool_analog_input_value',
            'tool_voltage', 'tool_output_voltage', 'tool_current', 'tool_temperature', 'tool_mode',
            'safe_sub_len', 'safe_sub_type',
            'safety_crc_num', 'safety_operational_mode', 'reserved_safe',
            'current_elbow_position_x', 'current_elbow_position_y', 'current_elbow_position_z', 'elbow_radius',
            'tool_comm_sub_len', 'tool_comm_sub_type',
            'is_enable', 'baudrate', 'parity', 'stopbits', 'tci_modbus_status', 'tci_usage', 'reserved_tc1', 'reserved_tc2'
        ]
        self.fmt = (
            '>' +
            FMT_HEADER + FMT_ROBOT_MODE +
            FMT_JOINT_HEADER + (FMT_JOINT_DATA * 6) +
            FMT_CARTESIAN + FMT_CONFIG + FMT_MASTERBOARD +
            FMT_ADDITIONAL + FMT_TOOL + FMT_SAFETY + FMT_TOOL_COMM
        )

class RobotHeader():
    __slots__ = ['type', 'size',]
    @staticmethod
    def unpack(buf):
        rmd = RobotHeader()
        (rmd.size, rmd.type) = struct.unpack_from('>iB', buf)
        return rmd

class RobotData():
    @staticmethod
    def unpack(buf, config):
        data = RobotData()
        try:
            unpacked = struct.unpack(config.fmt, buf)
            it = iter(unpacked)
            for name in config.names_pre: setattr(data, name, next(it))
            for name in config.names_joint: setattr(data, name, [])
            for _ in range(6):
                for name in config.names_joint:
                    getattr(data, name).append(next(it))
            for name in config.names_post: setattr(data, name, next(it))
            return data
        except (struct.error, StopIteration):
            return None
    
class AlarmData:
    def __init__(self, code=None, sub=None, level=None, msg=None):
        self.code = code
        self.sub = sub
        self.level = level
        self.msg = msg
        self.timestamp = time.time()
        self.active = True
    
class ReadAlarm():
    @staticmethod
    def unpack(buf):
        data_length = struct.unpack(">i", buf[0:4])[0]
        data = buf[0:data_length]
        msg_type = data[14]
        if msg_type == 10:
            msg = bytearray(data[23:data_length-1]).decode()
            return AlarmData(msg=msg)
        if msg_type == 6:
            error_code = struct.unpack(">i", data[15:19])[0]
            sub_error_code = struct.unpack(">i", data[19:23])[0]
            level = struct.unpack(">i", data[23:27])[0]
            return AlarmData(code=error_code, sub=sub_error_code, level=level)
        return None
        
class Robot_30001():
    def __init__(self, ip, port2) -> None:
        self.__data_config = RobotDataConfig()
        self.ip = ip
        self.port2 = port2
        self.alarm_queue = queue.Queue()
        self.__sock = None
        self.__buf = b""

    def connect_30001(self):
        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.settimeout(0.5)
            self.__sock.connect((self.ip, self.port2))
            self.__sock.settimeout(10.0) 
            print(f"[DEBUG][Robot Primary] Connected to {self.ip} on port {self.port2}")
            self.__buf = b""
            return self.__sock
        except Exception as e:
            print(f"[ERROR][Robot Primary] Connecting to {self.ip} on port {self.port2}: {e}")
            self.__sock = None 
            return None 
        
    def disconnect_30001(self):
        if self.__sock:
            self.__sock.close()
            self.__sock = None
            print("[Robot Primary] Disconnect")

    def send_script(self, script):
        """스크립트 본문을 30001로 보낸다. 응답은 없다.

        조그(speedl/speedj)와 픽앤플레이스 이동처럼 태스크 밖에서 로봇을
        움직일 때 쓴다. 여러 줄이면 그대로 이어 보내고 마지막에 줄바꿈을 붙인다.

        **끊긴 소켓은 다시 붙여 한 번 더 보낸다.** 컨트롤러는 29999 `stop`
        이나 전원 조작 때 이 연결을 끊는데, 상태 읽기(`get_data`)는 조용히
        실패만 하고 넘어가므로 아무도 끊긴 걸 눈치채지 못했다. 그 결과
        그 뒤의 모든 이동 명령이 영원히 "전송 실패" 로 흘러갔다.
        """
        body = script if script.endswith("\n") else script + "\n"
        for attempt in (1, 2):
            if self.__sock is None and self.connect_30001() is None:
                return False
            try:
                self.__sock.sendall(body.encode("utf-8"))
                return True
            except Exception as e:
                print(f"[ERROR][Robot Primary] Sending script: {e} (시도 {attempt}/2)")
                self.disconnect_30001()
        return False

    def get_data(self):
        return self.__recv()

    def __recv(self):
        try:
            self.__read_socket_no_wait()
        except Exception:
            return None
        last_valid_data = None
        while len(self.__buf) >= 5:
            try:
                head = RobotHeader.unpack(self.__buf)
            except:
                self.__buf = b""
                break
            if len(self.__buf) < head.size:
                break
            payload = self.__buf[:head.size]
            self.__buf = self.__buf[head.size:]
            if head.type == MESSAGE_TYPE_ROBOT_MESSAGE:
                try:
                    alarm = ReadAlarm.unpack(payload)
                    if alarm: self.alarm_queue.put(alarm)
                except: pass
                continue
            if head.type == MESSAGE_TYPE_ROBOT_STATE:
                try:
                    last_valid_data = RobotData.unpack(payload, self.__data_config)
                except: pass
        return last_valid_data

    def __read_socket_no_wait(self):
        while True:
            readable, _, _ = select.select([self.__sock], [], [], 0)
            if not readable: break
            try:
                more = self.__sock.recv(4096)
                if not more: raise ConnectionError("Socket closed")
                self.__buf += more
            except BlockingIOError:
                break
            except Exception:
                break
            
class AlarmManager:
    """같은 알람을 짧은 시간 안에 중복으로만 걸러내고, 다시 발생하면 다시 알린다.

    예전에는 (code, sub, msg) 키가 한 번이라도 나오면 영원히 dict 에 남아 있어서,
    같은 알람이 나중에 또 발생해도(예: 같은 원인으로 반복되는 역기구학 실패)
    프로세스가 떠 있는 동안은 다시는 안 알려졌다 — RCS 운영자가 재발을 놓치는
    문제였다. 이제는 최근 본 시각만 기억해 두고, 그 시각으로부터
    `dedup_window` 초가 지나면 같은 알람도 새로 온 것으로 다시 알린다.
    (같은 물리적 이벤트가 한 번에 패킷 여러 개로 쪼개져 들어오는 것만 눌러 준다.)
    """

    def __init__(self, dedup_window=2.0):
        self.dedup_window = dedup_window
        self._last_seen = {}

    def process(self, alarm):
        key = (alarm.code, alarm.sub, alarm.msg)
        now = time.time()
        last = self._last_seen.get(key)
        if last is not None and (now - last) < self.dedup_window:
            return False
        self._last_seen[key] = now
        print("================================")
        print("[ALARM TRIGGERED]")
        if alarm.msg:
            print(f"[ALARM MSG] {alarm.msg}")
        else:
            print(f"[ALARM CODE] E{alarm.code} S{alarm.sub} (level={alarm.level})")
        print("================================")
        return True


class Robot_29999():
    """로봇 대시보드(29999) 클라이언트.

    **끊긴 소켓을 다시 붙인다.** 예전에는 명령 하나가 예외로 실패하면
    망가진 소켓을 그대로 들고 있어서, 그 뒤 모든 명령이 영원히 None 을
    돌려줬다(`[play] Result: None`). 컨트롤러는 펜던트에서 태스크를 다시
    불러오거나 전원을 껐다 켜면 이 연결을 끊는데, 그걸 다시 잇는 코드가
    없었다. 이제는 실패하면 소켓을 버리고 새로 붙여 **한 번 더** 보낸다.

    실패 사유는 `last_error` 에 남긴다 — None 만 돌려주면 받는 쪽이
    "Result: None" 말고는 보여줄 게 없다.
    """

    #: 명령 하나의 응답을 기다리는 시간 [s]. 대시보드는 보통 즉시 답한다.
    #: 너무 길면 노드(단일 스레드 실행기)가 그동안 Modbus 폴링까지 멈춘다.
    RECV_TIMEOUT_S = 3.0
    #: 붙을 때 기다리는 시간 [s].
    CONNECT_TIMEOUT_S = 2.0

    def __init__(self, ip, port1):
        self.sock = None
        self.ip = ip
        self.port1 = port1
        self.last_error = ""
        # 명령·응답 한 쌍이 다른 명령과 섞이지 않게 한다. 하나의 소켓을
        # 속도 변경·play·stop 이 같이 쓴다.
        self._lock = threading.Lock()

    def connect_29999(self):
        # 이미 붙어 있던 소켓은 닫고 새로 붙는다(새는 소켓이 없게).
        self._close_quietly()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.CONNECT_TIMEOUT_S)
            sock.connect((self.ip, self.port1))
            sock.settimeout(self.RECV_TIMEOUT_S)
            print(f"[DEBUG][Robot Dashboard] Connected to {self.ip} on port {self.port1}")
            # 접속 인사말을 먼저 비워 둔다. 안 비우면 첫 명령이 인사말을
            # 자기 응답으로 읽는다.
            try:
                sock.recv(4096)
            except Exception:
                pass
            self.sock = sock
            self.last_error = ""
            return self.sock
        except Exception as e:
            self.last_error = f"대시보드({self.ip}:{self.port1}) 연결 실패: {e}"
            print(f"[ERROR][Robot Dashboard] {self.last_error}")
            self.sock = None
            return None

    def send_command_29999(self, command):
        """명령을 보내고 응답 한 줄을 돌려준다. 실패하면 None.

        실패하면 소켓을 버리고 다시 붙여 한 번 더 시도한다. 그래도 안 되면
        None 을 돌려주고 사유를 `last_error` 에 남긴다.
        """
        with self._lock:
            for attempt in (1, 2):
                if self.sock is None and self.connect_29999() is None:
                    return None
                try:
                    self._drain()
                    self.sock.sendall(f"{command}\n".encode("utf-8"))
                    raw = self.sock.recv(4096)
                    if not raw:
                        # 컨트롤러가 연결을 닫았다. 다시 붙어 한 번 더.
                        raise ConnectionError("대시보드가 연결을 닫았습니다")
                    self.last_error = ""
                    return raw.decode("utf-8", errors="replace").strip()
                except Exception as e:
                    self.last_error = f"'{command}' 전송 실패: {e}"
                    print(f"[ERROR][Robot Dashboard] {self.last_error} "
                          f"(시도 {attempt}/2)")
                    self._close_quietly()
            return None

    def _drain(self):
        """지난 명령의 늦은 응답이 남아 있으면 버린다.

        남겨 두면 이번 명령이 그걸 자기 응답으로 읽어, 이후 응답이 한 칸씩
        밀린다(stop 의 응답을 play 가 읽는 식).
        """
        if self.sock is None:
            return
        try:
            self.sock.setblocking(False)
            while True:
                if not self.sock.recv(4096):
                    break
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            pass
        finally:
            if self.sock is not None:
                self.sock.settimeout(self.RECV_TIMEOUT_S)

    def _close_quietly(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def disconnect_29999(self):
        with self._lock:
            if self.sock:
                self._close_quietly()
                print("[Robot Dashboard] Disconnect")

    def robot_mode(self): return self.send_command_29999("robotMode")
    def robot_status(self): return self.send_command_29999("status")
    def robot_power_on(self): return self.send_command_29999("robotControl -on")
    def robot_power_off(self): return self.send_command_29999("robotControl -off")
    def robot_brakeRelease(self): return self.send_command_29999("brakeRelease")
    # play/stop 은 원격 제어 모드에서만 받는다. 로컬 제어 모드면 컨트롤러가
    # "not supported in local control mode" 로 거부하므로 먼저 켜 줘야 한다.
    def robot_remote_control_on(self): return self.send_command_29999("remoteControl -on")
    def robot_remote_control_off(self): return self.send_command_29999("remoteControl -off")

    def robot_set_speed(self, percent):
        """로봇 전체 동작 속도 비율[%]을 실시간으로 바꾼다. 2~100.

        대시보드 명령표(robot_task/29999 대쉬보드 command.png)의 형식은
        `speed -v 50` 이다("Set the robot speed to 50% (Range: 2%~100%)").
        예전에는 표에 없는 `speed -set N` 을 보내서 컨트롤러가 비율을
        제대로 받지 못했다. 레지스터 17에 직접 써도 같은 값이 되지만,
        제어 경로를 29999 한 곳으로 모아 둔다. 읽기는 레지스터 17을 쓴다.
        """
        percent = max(2, min(100, int(percent)))
        return self.send_command_29999(f"speed -v {percent}")

    def robot_play(self): return self.send_command_29999("play")
    def robot_pause(self): return self.send_command_29999("pause")
    def robot_stop(self): return self.send_command_29999("stop")

    def robot_task_status(self):
        """지금 로봇에 올라가 있는 태스크 상태를 29999로 물어본다.

        태스크는 펜던트/로봇 쪽에서 고정이라 운영 UI가 고르지 않는다 —
        이 응답을 그대로 보여주기만 한다. 응답 형식은 컨트롤러가 주는
        그대로다(예: "Task is running"); 잘라서 가공하지 않는다 — 잘못
        파싱해 정보를 지우는 것보다 원문 그대로 보여주는 쪽이 안전하다.
        """
        return self.send_command_29999("task -s")

    def set_variable(self, name, value):
        """로봇 쪽 전역 변수를 29999로 설정한다."""
        return self.send_command_29999(f"variable -set {name} {value}")

    def get_variable(self, var_name):
        """로봇 쪽 전역 변수 값을 29999로 읽어 파이썬 타입으로 돌려준다.

        - 리스트("[1.1, 2.2]") -> list[float]
        - 불리언("True"/"False") -> bool
        - 숫자 -> int/float
        - 그 외 -> str
        값이 없거나 오류면 None, 변수를 못 찾으면 "NOT_FOUND"를 돌려준다.
        """
        try:
            response = self.send_command_29999(f"variable -get {var_name}")
            if response is None:
                return None
            if "Error" in response or "undefined" in response or "Can not find" in response:
                return "NOT_FOUND"

            if "[" in response and "]" in response:
                content = response[response.find("[") + 1:response.find("]")]
                result = []
                for item in content.split(","):
                    item = item.strip()
                    if not item:
                        continue
                    try:
                        result.append(float(item))
                    except ValueError:
                        result.append(item)
                return result

            raw = response.split(",")[-1].strip() if "," in response else response.strip()
            if raw.lower() == "true":
                return True
            if raw.lower() == "false":
                return False
            try:
                return int(raw)
            except ValueError:
                pass
            try:
                return float(raw)
            except ValueError:
                pass
            if (raw.startswith('"') and raw.endswith('"')) or \
               (raw.startswith("'") and raw.endswith("'")):
                return raw[1:-1]
            return raw
        except Exception as e:
            print(f"[ERROR][Robot Dashboard] variable -get {var_name}: {e}")
            return None
    
class Robot_modbus():       
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = None
        self.is_running = False

    def connect(self):
        self.client = ModbusClient(host=self.host, port=self.port, unit_id=255, timeout=1.0)
        
        if self.client.is_open:
            self.client.close()
        
        is_open = self.client.open()
        if is_open:
            self.is_running = True
            print(f"[DEBUG][Robot Modbus] Connected to {self.host} on port {self.port}")
            return True
        else:
            print(f"[ERROR][Robot Modbus] Connecting to {self.host} on port {self.port}: timed out")
            return False

    def disconnect(self):
        if self.client:
            self.client.close()
        self.is_running = False
        print("[Robot Modbus] Disconnect")

    def set_register(self, address, value):
        try:
            write_data = int(float(value))
        except ValueError:
            write_data = 1 if str(value).strip().lower() in ['true', '1'] else 0
        modbus_write_val = write_data & 0xFFFF
        is_success = self.client.write_single_register(address, modbus_write_val)
        if not is_success:
            return False
        import time
        time.sleep(0.05) 
        
        signed_read_val = self.get_register(address)
        if signed_read_val is not None and write_data == signed_read_val:
            return True
        return False

    def get_all_registers(self, start_address, count) -> list:
        regs = self.client.read_holding_registers(start_address, count)
        if regs is None:
            regs = self.client.read_input_registers(start_address, count)
        if regs:
            return [val - 65536 if val > 32767 else val for val in regs]
        return []
    
    def get_register(self, address) -> int:
        result = self.client.read_holding_registers(address, 1)
        if result is None:
            result = self.client.read_input_registers(address, 1)
        if result and len(result) > 0:
            val = result[0]
            return val - 65536 if val > 32767 else val
        return None