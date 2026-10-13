"""AlarmManager 회귀 테스트.

중복 억제가 '영구 차단'이 아니라 '짧은 시간 창'인지 검증한다. 예전에는
(code, sub, msg) 키가 한 번이라도 나오면 영원히 dict에 남아 있어서, 같은
알람이 나중에 다시 발생해도(예: 같은 원인의 반복되는 역기구학 실패)
프로세스가 떠 있는 동안은 다시는 RCS로 안 알려졌다.
"""

from unittest.mock import patch

from elite_robot_controller.robot.robot_driver import AlarmData, AlarmManager


def test_same_alarm_reported_again_after_dedup_window():
    mgr = AlarmManager(dedup_window=2.0)
    alarm = AlarmData(msg="PROBE_R: NO IK SOLUTION")

    with patch("elite_robot_controller.robot.robot_driver.time.time", return_value=1000.0):
        assert mgr.process(alarm) is True

    # 창 안에서 같은 알람이 또 오면(패킷이 쪼개져 들어온 경우 등) 눌러야 한다.
    with patch("elite_robot_controller.robot.robot_driver.time.time", return_value=1000.5):
        assert mgr.process(alarm) is False

    # 창이 지난 뒤 같은 알람이 다시 발생하면 새로 알려야 한다 — 예전 버그는
    # 여기서 영원히 False였다.
    with patch("elite_robot_controller.robot.robot_driver.time.time", return_value=1003.0):
        assert mgr.process(alarm) is True


def test_different_alarms_are_never_deduped_against_each_other():
    mgr = AlarmManager(dedup_window=2.0)
    with patch("elite_robot_controller.robot.robot_driver.time.time", return_value=1000.0):
        assert mgr.process(AlarmData(msg="A")) is True
        assert mgr.process(AlarmData(msg="B")) is True
