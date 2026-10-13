"""Orin 패널이 카메라·비전·그리퍼·브리지를 각각 켜고 끄는 데 쓴다.

README.odt 의 수동 절차(터미널 여러 개를 열어 launch / run 을 따로 치던 것)를
프로세스 하나씩으로 옮긴 것이다. ROS 와 무관하게 짜 두어 단독으로 시험할 수
있다.

끄는 순서가 중요하다. **SIGINT -> SIGTERM -> SIGKILL** 순으로 프로세스 그룹
전체에 보낸다. `ros2 launch` 는 SIGINT 를 받아야 자식 노드를 정리하고, 특히
RealSense 는 프로세스가 완전히 죽어야 USB 장치를 놓는다 — 덜 죽은 채로 다시
켜면 "Device or resource busy" 로 실패한다.
"""

from __future__ import annotations

import collections
import os
import re
import signal
import subprocess
import threading
import time

#: 로그에서 지울 색 코드(ROS 로그가 터미널용 색을 붙인다).
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


class ManagedProcess:
    """명령 하나를 띄우고, 로그를 모으고, 확실히 내린다."""

    def __init__(self, name: str, label: str, command: list, env: dict = None,
                 log_lines: int = 400):
        self.name = name
        self.label = label
        self.command = list(command)
        self.env = env
        self.lines = collections.deque(maxlen=log_lines)
        self._proc = None
        self._lock = threading.Lock()
        self.started_at = None
        self.exit_code = None
        self.stopping = False

    # ------------------------------------------------------------ 상태
    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def pid(self):
        return self._proc.pid if self.running else None

    def uptime(self) -> float:
        return time.monotonic() - self.started_at if self.running and self.started_at else 0.0

    def tail(self, count: int = 30) -> list:
        return list(self.lines)[-count:]

    def contains(self, needle: str) -> bool:
        """이번 실행의 로그에 이 문구가 나왔는지(켤 때 로그를 비운다)."""
        return any(needle in line for line in self.lines)

    # ------------------------------------------------------------ 켜기
    def start(self) -> tuple:
        with self._lock:
            if self.running:
                return False, f"{self.label}: 이미 실행 중입니다."
            self.lines.clear()
            self.exit_code = None
            self.stopping = False
            env = dict(os.environ)
            if self.env:
                env.update(self.env)
            # 줄 단위로 바로 받기 위해. 색 코드도 끈다.
            env.setdefault("PYTHONUNBUFFERED", "1")
            env["RCUTILS_COLORIZED_OUTPUT"] = "0"
            try:
                self._proc = subprocess.Popen(
                    self.command, env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, text=True, bufsize=1,
                    errors="replace",
                    # 자기 프로세스 그룹을 갖게 해서, 끌 때 자식까지 한 번에 보낸다.
                    start_new_session=True,
                )
            except OSError as exc:
                self._proc = None
                self.lines.append(f"[패널] 실행 실패: {exc}")
                return False, f"{self.label}: 실행 실패 — {exc}"
            self.started_at = time.monotonic()
            threading.Thread(target=self._pump, args=(self._proc,),
                             name=f"log-{self.name}", daemon=True).start()
            return True, f"{self.label}: 시작했습니다."

    def _pump(self, proc):
        for raw in proc.stdout:
            line = _ANSI.sub("", raw.rstrip("\n"))
            if line.strip():
                self.lines.append(line)
        code = proc.wait()
        self.exit_code = code
        if not self.stopping:
            self.lines.append(f"[패널] 프로세스가 끝났습니다 (종료 코드 {code}).")

    # ------------------------------------------------------------ 끄기
    def stop(self, grace_int: float = 8.0, grace_term: float = 3.0) -> tuple:
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                return False, f"{self.label}: 실행 중이 아닙니다."
            self.stopping = True
            for sig, grace in ((signal.SIGINT, grace_int), (signal.SIGTERM, grace_term),
                               (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    proc.wait(timeout=grace)
                    break
                except subprocess.TimeoutExpired:
                    continue
            self.lines.append("[패널] 중지했습니다.")
            return True, f"{self.label}: 중지했습니다."


class ProcessManager:
    """여러 ManagedProcess 를 이름으로 묶는다."""

    def __init__(self):
        self.items = collections.OrderedDict()

    def add(self, process: ManagedProcess):
        self.items[process.name] = process

    def get(self, name: str):
        return self.items.get(name)

    def stop_all(self):
        # 띄운 반대 순서로 내린다.
        for process in reversed(list(self.items.values())):
            if process.running:
                process.stop()
