"""대시보드(29999) 클라이언트가 끊긴 연결에서 다시 붙는지 확인한다.

예전에는 명령 하나가 예외로 실패하면 망가진 소켓을 그대로 들고 있어서,
그 뒤 모든 명령이 영원히 None 을 돌려줬다(RCS 화면의 `[play] Result: None`).
가짜 대시보드 서버를 띄워 연결을 일부러 끊어 보고, 다음 명령이 새로 붙어
제대로 답을 받는지 본다.
"""

import socket
import threading

import pytest

from elite_robot_controller.robot.robot_driver import Robot_29999


class FakeDashboard:
    """Elite 대시보드 흉내. 접속하면 인사말, 명령마다 한 줄로 답한다.

    `drop_after` 개의 명령에 답한 뒤에는 연결을 닫는다 — 펜던트에서 태스크를
    다시 불러오거나 컨트롤러가 재시작했을 때와 같다.
    """

    def __init__(self, drop_after=None, replies=None):
        self.drop_after = drop_after
        self.replies = replies or {}
        self.received = []
        self.connections = 0
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(4)
        self.port = self._srv.getsockname()[1]
        self._stop = False
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        self._srv.settimeout(0.2)
        while not self._stop:
            try:
                conn, _ = self._srv.accept()
            except OSError:
                continue
            self.connections += 1
            with conn:
                conn.sendall(b"Connected: Elite Dashboard Server\n")
                answered = 0
                buf = b""
                conn.settimeout(0.2)
                while not self._stop:
                    try:
                        chunk = conn.recv(1024)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        cmd = line.decode().strip()
                        self.received.append(cmd)
                        reply = self.replies.get(cmd, f"OK {cmd}")
                        conn.sendall(f"{reply}\n".encode())
                        answered += 1
                    if self.drop_after is not None and answered >= self.drop_after:
                        break   # 연결을 끊는다

    def close(self):
        self._stop = True
        self._srv.close()
        self._thread.join(timeout=1.0)


@pytest.fixture
def server():
    srv = FakeDashboard(drop_after=1)
    yield srv
    srv.close()


def test_command_after_a_dropped_connection_reconnects(server):
    """연결이 끊겨도 다음 명령은 새로 붙어 답을 받는다."""
    dash = Robot_29999("127.0.0.1", server.port)
    assert dash.connect_29999() is not None

    assert dash.send_command_29999("stop") == "OK stop"
    # 서버가 방금 연결을 닫았다. 예전에는 여기서부터 영원히 None 이었다.
    assert dash.send_command_29999("play") == "OK play"
    assert server.connections >= 2, "다시 붙지 않았다"
    dash.disconnect_29999()


def test_unreachable_dashboard_reports_why():
    """못 붙으면 None 과 함께 사유를 남긴다 — 'Result: None' 만으로는 모른다."""
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()           # 이 포트엔 아무도 없다

    dash = Robot_29999("127.0.0.1", port)
    assert dash.send_command_29999("play") is None
    assert "연결 실패" in dash.last_error


def test_stale_reply_is_not_taken_as_the_next_answer():
    """늦게 온 지난 응답을 다음 명령의 답으로 읽지 않는다."""
    srv = FakeDashboard()
    try:
        dash = Robot_29999("127.0.0.1", srv.port)
        dash.connect_29999()
        # 서버 쪽에서 요청 없이 한 줄을 흘려보낸 상황을 흉내 낸다.
        assert dash.send_command_29999("robotMode") == "OK robotMode"
        dash.sock.sendall(b"")  # 연결이 살아 있는지만 확인
        assert dash.send_command_29999("play") == "OK play"
        dash.disconnect_29999()
    finally:
        srv.close()
