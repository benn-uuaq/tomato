"""로봇 PC 에서 Orin 의 비전 쪽을 켜고 끄고 상태를 본다 — UI 의 [비전 시작] 버튼.

이 버튼 하나면 Orin 에 가서 `vision_run.sh` 를 따로 칠 필요가 없다.

    [비전 시작]
      Orin 비전 패널(:8081) 이 떠 있다  -> 패널에 "전체 시작" 을 HTTP 로 요청
      안 떠 있다                        -> SSH 로 Orin 의 vision_run.sh 를 띄운다
    [비전 정지]
      패널에 "전체 정지" 를 요청 — 패널이 띄운 것만 내린다(한양대 쪽에서 켠 것은 그대로)

SSH 는 **키 인증만** 쓴다(BatchMode). 비밀번호를 물으면 그대로 실패로 알린다 —
화면이 멈춘 채 비밀번호를 기다리지 않게. 키가 없으면 한 번만:

    ssh-copy-id nano@<Orin IP>

Orin 쪽에 미리 설치하거나 띄워 둘 것은 없다 — 전원과 sshd 만 있으면 된다.
"""

from __future__ import annotations

import json
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


#: Orin 에서 비전 쪽을 띄우는 명령. 끝나지 않는 프로세스라 떼어 놓고(setsid nohup)
#: 바로 돌아온다. 로그는 Orin 의 /tmp/vision_run.log.
#: `{ ... & }` 로 이 명령 하나만 뒤로 보낸다 — `cd ... && cmd &` 로 쓰면 `cd && cmd`
#: 전체가 서브셸로 뒤에 남아 SSH 채널을 쥐고 있어 ssh 가 돌아오지 않는다.
START_REMOTE = ("{ setsid nohup ./vision_run.sh --no-window > /tmp/vision_run.log 2>&1 "
                "< /dev/null & } && sleep 1 && echo started")


class VisionRemote:
    """Orin 비전 패널(HTTP) 과 SSH 를 묶은 작은 클라이언트."""

    def __init__(self, user: str = "nano", workspace: str = "~/tomato_ws",
                 panel_port: int = 8081):
        self.user = user
        self.workspace = workspace
        self.panel_port = int(panel_port)

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
        """비전 쪽을 켠다. 패널이 있으면 패널로, 없으면 SSH 로 vision_run.sh."""
        if self.panel_status(host) is not None:
            try:
                result = self._panel_post(host, "/all/start")
            except (urllib.error.URLError, OSError, ValueError) as exc:
                return {"success": False, "message": f"Orin 패널 요청 실패: {exc}"}
            return {"success": bool(result.get("success")),
                    "message": "Orin 패널: " + str(result.get("message", ""))}
        try:
            done = subprocess.run(ssh_command(self.user, host, self.workspace, START_REMOTE),
                                  capture_output=True, text=True, timeout=20)
        except FileNotFoundError:
            return {"success": False, "message": "이 PC 에 ssh 가 없습니다."}
        except subprocess.TimeoutExpired:
            # 원격에서는 이미 떴을 수 있다 — 패널이 보이면 성공으로 본다.
            if self.panel_status(host) is not None:
                return {"success": True, "message": f"Orin({host}) 비전 패널이 떴습니다."}
            return {"success": False, "message": f"Orin({host}) SSH 응답이 없습니다."}
        if done.returncode != 0 or "started" not in done.stdout:
            reason = (done.stderr or done.stdout).strip().splitlines()
            reason = reason[-1] if reason else f"종료 코드 {done.returncode}"
            hint = ""
            if "Permission denied" in reason or "publickey" in reason:
                hint = f" — 이 PC 의 SSH 키를 Orin 에 한 번 등록하세요: ssh-copy-id {self.user}@{host}"
            return {"success": False, "message": f"Orin 에서 vision_run.sh 를 못 띄웠습니다: {reason}{hint}"}
        return {"success": True,
                "message": f"Orin({host}) 에서 vision_run.sh 를 시작했습니다 — 카메라·모델이 올라오는 데 "
                           "20~60 초 걸립니다."}

    def stop(self, host: str) -> dict:
        """패널이 띄운 것만 내린다. 한양대 쪽에서 직접 켠 노드는 그대로 둔다."""
        if self.panel_status(host) is None:
            return {"success": False, "message": f"Orin({host}) 비전 패널에 닿지 않습니다 — 이미 꺼져 있습니다."}
        try:
            result = self._panel_post(host, "/all/stop")
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return {"success": False, "message": f"Orin 패널 요청 실패: {exc}"}
        return {"success": bool(result.get("success")),
                "message": "Orin 패널: " + str(result.get("message", ""))}


def summarize(status) -> dict:
    """패널 /status 를 로봇 PC 화면용으로 줄인다."""
    if not status:
        return {"reachable": False, "ready": False,
                "text": "Orin 비전 패널이 꺼져 있습니다 — [비전 시작] 을 누르세요", "components": []}
    components = [{"label": c.get("label"), "state": c.get("state"),
                   "detail": c.get("detail"), "external": c.get("external", False)}
                  for c in status.get("components", [])]
    text = status.get("overall", {}).get("text", "")
    if components and all(c["state"] == "stopped" for c in components):
        # 패널 문구는 Orin 화면의 [전체 시작] 을 가리킨다 — 여기서는 이 화면의 버튼으로.
        text = "Orin 패널은 떠 있고 부품은 꺼져 있습니다 — [비전 시작] 을 누르세요"
    return {
        "reachable": True,
        "ready": bool(status.get("overall", {}).get("ready")),
        "text": text,
        "components": components,
    }
