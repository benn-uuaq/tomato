#!/usr/bin/env python3
"""로봇이 보고하는 TCP 회전이 RPY 인지 회전 벡터인지 확인한다 — **로봇을 움직이지 않는다.**

    python3 tools/check_pose_convention.py 192.168.1.200

30001 상태 패킷 하나를 받아
  1. 컨트롤러가 실어 보내는 DH 파라미터와 현재 관절값으로 정기구학을 직접 계산하고
  2. 컨트롤러가 보고한 TCP 회전을 RPY 로 읽은 것 / 회전 벡터로 읽은 것과 비교한다.
가까운 쪽이 이 로봇의 표기다. TCP 에 회전 오프셋이 설정돼 있으면 둘 다 안 맞을
수 있다(그때는 TCP 설정을 펜던트에서 확인).

결과가 RPY 가 아니면 config/tomato_pick.yaml 의 `orientation_format` 을 rotvec 으로.
"""

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from elite_robot_controller.robot.robot_driver import Robot_30001  # noqa: E402
from elite_robot_controller.tomato import geometry  # noqa: E402


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.200"
    primary = Robot_30001(ip, 30001)
    if primary.connect_30001() is None:
        print(f"30001 에 연결하지 못했습니다: {ip}")
        return 2
    data = None
    deadline = time.time() + 5
    while data is None and time.time() < deadline:
        data = primary.get_data()
        time.sleep(0.1)
    primary.disconnect_30001()
    if data is None:
        print("상태 패킷을 받지 못했습니다.")
        return 2

    a = [getattr(data, f"dh_a_joint_{i}") for i in range(6)]
    d = [getattr(data, f"dh_d_joint_{i}") for i in range(6)]
    alpha = [getattr(data, f"dh_alpha_joint_{i}") for i in range(6)]
    theta = [getattr(data, f"dh_theta_joint_{i}") for i in range(6)]
    joints = list(data.actual_joint)
    reported = [data.rot_x, data.rot_y, data.rot_z]
    tcp = [data.tcp_x, data.tcp_y, data.tcp_z]

    rot_fk, pos_fk = geometry.fk_dh(joints, a, d, alpha, theta)
    err_rpy = geometry.rotation_angle_deg(rot_fk, geometry.rpy_to_matrix(*reported))
    err_rotvec = geometry.rotation_angle_deg(rot_fk, geometry.rotvec_to_matrix(*reported))

    print(f"관절 [deg]      : {[round(v * 57.29578, 2) for v in joints]}")
    print(f"보고된 TCP [m]  : {[round(v, 4) for v in tcp]}")
    print(f"보고된 회전     : {[round(v, 4) for v in reported]}")
    print(f"정기구학 플랜지 : {[round(v, 4) for v in pos_fk]}  (TCP 오프셋만큼 다를 수 있다)")
    print(f"RPY 로 읽었을 때 오차      : {err_rpy:7.2f} deg")
    print(f"회전 벡터로 읽었을 때 오차 : {err_rotvec:7.2f} deg")
    if min(err_rpy, err_rotvec) > 3.0:
        print("판정: 둘 다 맞지 않습니다 — TCP 에 회전 오프셋이 있거나 DH 해석이 다릅니다.")
        return 1
    verdict = "rpy" if err_rpy < err_rotvec else "rotvec"
    print(f"판정: {verdict}  (config 의 orientation_format 을 이 값으로)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
