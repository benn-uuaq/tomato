"""작업 UI 의 [비전 시작] / [비전 정지] — 비전 패널(:8081)을 부른다.

한 대에서 다 돌리면(기본) 패널은 같은 장비에 있다(`robot_run.sh` 가 같이 띄운다).

    [비전 시작]
      비전 패널이 떠 있다  -> 패널에 "전체 시작" 을 HTTP 로 요청 (카메라·YOLO·그리퍼)
      안 떠 있다           -> vision_run.sh 를 띄운다 (같은 장비면 그냥, 다른 장비면 SSH)
    [비전 정지]
      패널에 "전체 정지" 를 요청 — 패널이 띄운 것만 내린다(한양대 쪽에서 켠 것은 그대로)

두 대로 나눠 쓸 때(로봇 PC -> Orin)는 SSH 를 **키 인증만** 쓴다(BatchMode). 비밀번호를
물으면 그대로 실패로 알린다 — 화면이 멈춘 채 비밀번호를 기다리지 않게. 키가 없으면 한 번만:

    ssh-copy-id nano@<Orin IP>
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import urllib.error
import urllib.request


def workspace_cd(path: str) -> str:
    """원격 셸에서 쓸 `cd ...` — `~/` 는 원격 홈으로 풀리게 두고 나머지는 따옴표로."""
    path = str(path or "~/tomato_ws").strip()
    if path == "~":
        return "cd ~"
    if path.startswith("~/"):
        return "cd ~/" + shlex.quote(path[2:])
    return "cd " + shlex.quote(path)


def ssh_command(user: str, host: str, workspace: str, remote: str) -> list:
    """키 인증만 쓰는 ssh 명령 목록. 비밀번호를 묻는 대신 실패한다."""
    return [
        "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
        "-o", "StrictHostKeyChecking=accept-new",
        f"{user}@{host}", f"{workspace_cd(workspace)} && {remote}",
    ]


#: 같은 장비로 보는 주소 — 이때는 SSH 없이 바로 띄운다.
LOCAL_HOSTS = ("127.0.0.1", "localhost")


def is_local(host: str) -> bool:
    return str(host).strip().lower() in LOCAL_HOSTS


#: 비전 쪽을 띄우는 명령. 끝나지 않는 프로세스라 떼어 놓고(setsid nohup)
#: 바로 돌아온다. 로그는 띄운 장비의 /tmp/vision_run.log.
#: `{ ... & }` 로 이 명령 하나만 뒤로 보낸다 — `cd ... && cmd &` 로 쓰면 `cd && cmd`
#: 전체가 서브셸로 뒤에 남아 SSH 채널을 쥐고 있어 ssh 가 돌아오지 않는다.
START_REMOTE = ("{ setsid nohup ./vision_run.sh --no-window > /tmp/vision_run.log 2>&1 "
                "< /dev/null & } && sleep 1 && echo started")


class VisionRemote:
    """비전 패널(HTTP) 과 vision_run.sh 띄우기를 묶은 작은 클라이언트."""

    def __init__(self, user: str = "nano", workspace: str = "~/tomato_ws",
                 panel_port: int = 8081, local_workspace: str = ""):
        self.user = user
        self.workspace = workspace              # 다른 장비(SSH)의 워크스페이스
        self.local_workspace = (local_workspace or os.environ.get("TOMATO_WS")
                                or os.path.expanduser("~/tomato_ws"))
        self.panel_port = int(panel_port)

    @staticmethod
    def where(host: str) -> str:
        return "이 PC" if is_local(host) else f"Orin({host})"

    def launch_command(self, host: str) -> list:
        """vision_run.sh 를 떼어 놓고 띄우는 명령 — 같은 장비면 bash, 아니면 ssh."""
        if is_local(host):
            return ["bash", "-c", f"cd {shlex.quote(self.local_workspace)} && {START_REMOTE}"]
        return ssh_command(self.user, host, self.workspace, START_REMOTE)

    # ------------------------------------------------------------ 패널
    def _url(self, host: str, path: str) -> str:
        return f"http://{host}:{self.panel_port}{path}"

    def panel_status(self, host: str, timeout: float = 2.0):
        """패널의 /status. 닿지 않으면 None."""
        try:
            with urllib.request.urlopen(self._url(host, "/status"), timeout=timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def _panel_post(self, host: str, path: str, timeout: float = 30.0) -> dict:
        req = urllib.request.Request(self._url(host, path), data=b"{}", method="POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    # ------------------------------------------------------------ 조작
    def start(self, host: str) -> dict:
        """비전 쪽을 켠다. 패널이 있으면 패널로, 없으면 vision_run.sh 를 띄운다."""
        where = self.where(host)
        if self.panel_status(host) is not None:
            try:
                result = self._panel_post(host, "/all/start")
            except (urllib.error.URLError, OSError, ValueError) as exc:
                return {"success": False, "message": f"비전 패널 요청 실패: {exc}"}
            return {"success": bool(result.get("success")),
                    "message": "비전 패널: " + str(result.get("message", ""))}
        try:
            done = subprocess.run(self.launch_command(host),
                                  capture_output=True, text=True, timeout=20)
        except FileNotFoundError:
            return {"success": False, "message": "이 PC 에 ssh 가 없습니다."}
        except subprocess.TimeoutExpired:
            # 이미 떴을 수 있다 — 패널이 보이면 성공으로 본다.
            if self.panel_status(host) is not None:
                return {"success": True, "message": f"{where} 비전 패널이 떴습니다."}
            return {"success": False, "message": f"{where} 에서 응답이 없습니다."}
        if done.returncode != 0 or "started" not in done.stdout:
            reason = (done.stderr or done.stdout).strip().splitlines()
            reason = reason[-1] if reason else f"종료 코드 {done.returncode}"
            hint = ""
            if "Permission denied" in reason or "publickey" in reason:
                hint = f" — 이 PC 의 SSH 키를 Orin 에 한 번 등록하세요: ssh-copy-id {self.user}@{host}"
            return {"success": False, "message": f"{where} 에서 vision_run.sh 를 못 띄웠습니다: {reason}{hint}"}
        return {"success": True,
                "message": f"{where} 에서 vision_run.sh 를 시작했습니다 — 카메라·모델이 올라오는 데 "
                           "20~60 초 걸립니다."}

    def stop(self, host: str) -> dict:
        """패널이 띄운 것만 내린다. 한양대 쪽에서 직접 켠 노드는 그대로 둔다."""
        if self.panel_status(host) is None:
            return {"success": False,
                    "message": f"{self.where(host)} 비전 패널에 닿지 않습니다 — 이미 꺼져 있습니다."}
        try:
            result = self._panel_post(host, "/all/stop")
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return {"success": False, "message": f"비전 패널 요청 실패: {exc}"}
        return {"success": bool(result.get("success")),
                "message": "비전 패널: " + str(result.get("message", ""))}


def summarize(status) -> dict:
    """패널 /status 를 로봇 PC 화면용으로 줄인다."""
    if not status:
        return {"reachable": False, "ready": False,
                "text": "비전 패널이 꺼져 있습니다 — [비전 시작] 을 누르세요", "components": []}
    components = [{"label": c.get("label"), "state": c.get("state"),
                   "detail": c.get("detail"), "external": c.get("external", False)}
                  for c in status.get("components", [])]
    text = status.get("overall", {}).get("text", "")
    if components and all(c["state"] == "stopped" for c in components):
        # 패널 문구는 패널 화면의 [전체 시작] 을 가리킨다 — 여기서는 작업 UI 의 버튼으로.
        text = "비전이 꺼져 있습니다 — [비전 시작] 을 누르세요"
    return {
        "reachable": True,
        "ready": bool(status.get("overall", {}).get("ready")),
        "text": text,
        "components": components,
    }
