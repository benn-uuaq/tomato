"""대시보드(29999) 속도 비율 명령 형식.

명령표(robot_task/29999 대쉬보드 command.png): `speed -v 50` = 속도 50 %
(범위 2 ~ 100 %). 예전에 보내던 `speed -set N` 은 표에 없는 형식이라
실장비에서 응답만 오고 비율(레지스터 17)이 바뀌지 않았다.
"""

from elite_robot_controller.robot.robot_driver import Robot_29999


def _sent(percent):
    dash = Robot_29999.__new__(Robot_29999)
    sent = []
    dash.send_command_29999 = lambda command: sent.append(command) or "ok"
    dash.robot_set_speed(percent)
    return sent


def test_speed_ratio_uses_the_documented_command():
    assert _sent(50) == ["speed -v 50"]


def test_speed_ratio_is_kept_in_range():
    assert _sent(0) == ["speed -v 2"]
    assert _sent(150) == ["speed -v 100"]
