#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
토마토 좌표 검증 노드 (eye-in-hand)

목적:
  카메라가 로봇 끝단에 장착된 상태에서, 토마토 노드가 발행한 카메라 기준 X, Y 만큼
  로봇을 상대 이동시킨 뒤 다시 측정했을 때 X, Y가 0에 가까워지는지 확인한다.
  Z는 건드리지 않는다.

동작 모드 (파라미터 mode):
  'raw'   : 로봇을 움직이지 않고 카메라 X, Y 원시값만 출력. 축 매핑 확인용.
  'move'  : 홈 이동 후, 오차가 tolerance_mm 이하가 될 때까지 측정과 상대 이동을 반복.

실행:
  ros2 run ...  또는  python3 tomato_align_test_node.py
  측정/이동 시작은 /align_test/start (std_msgs/Empty) 로 트리거.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty, Float32MultiArray
from geometry_msgs.msg import PoseArray

from dsr_msgs2.srv import MoveLine


# =====================================================================
# ★★★ 축 매핑 — 여기만 고치면 됩니다 ★★★
# ---------------------------------------------------------------------
# 카메라 optical frame:  X = 오른쪽, Y = 아래, Z = 피사체 방향
# 로봇 base frame     :  X, Y, Z (로봇 설정에 따름)
#
# 아래 함수는 "카메라에서 본 토마토 위치(mm)"를 "로봇이 움직여야 할 양(mm)"으로
# 바꿉니다. 부호와 축 대응은 실제 장착 방향에 따라 달라지므로,
# raw 모드로 확인한 뒤 이 함수만 수정하십시오.
#
# 예시)
#   return (cam_y, cam_x)      # 카메라 Y -> 로봇 X, 카메라 X -> 로봇 Y
#   return (-cam_y, -cam_x)    # 부호 반전이 필요한 경우
# =====================================================================
def camera_to_robot_delta(cam_x_mm, cam_y_mm):
    """카메라 X, Y (mm) -> 로봇 dx, dy (mm)"""
    dx = cam_y_mm      # 카메라 +Y -> 베이스 +X
    dy = cam_x_mm      # 카메라 +X -> 베이스 +Y
    return dx, dy


class TomatoAlignTestNode(Node):
    def __init__(self):
        super().__init__('tomato_align_test_node')

        self.group = ReentrantCallbackGroup()

        # ------------------------------------------------------------
        # 파라미터
        # ------------------------------------------------------------
        self.declare_parameter('mode', 'raw')          # 'raw' 또는 'move'
        self.declare_parameter('max_step_mm', 80.0)    # 1회 이동량 상한
        self.declare_parameter('settle_sec', 1.5)      # 이동 후 안정화 대기
        self.declare_parameter('tolerance_mm', 1.0)    # 수렴 판정 기준
        self.declare_parameter('max_iter', 5)          # 최대 반복 횟수
        self.mode = self.get_parameter('mode').value
        self.max_step = float(self.get_parameter('max_step_mm').value)
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.tolerance = float(self.get_parameter('tolerance_mm').value)
        self.max_iter = int(self.get_parameter('max_iter').value)

        # 딸기 코드와 동일한 홈 위치
        self.home_pos = [-550.0, 250.0, 400, 180.0, 180.0, 315.0]

        # ------------------------------------------------------------
        # 토마토 노드 연동
        # ------------------------------------------------------------
        self.capture_pub = self.create_publisher(Empty, '/tomato/capture', 10)
        self.pose_sub = self.create_subscription(
            PoseArray, '/tomato/center_positions', self.pose_callback, 10,
            callback_group=self.group)
        self.weight_sub = self.create_subscription(
            Float32MultiArray, '/tomato/estimated_weights', self.weight_callback, 10,
            callback_group=self.group)

        # 검증 시작 트리거
        self.start_sub = self.create_subscription(
            Empty, '/align_test/start', self.start_callback, 10,
            callback_group=self.group)

        self.latest_poses = None       # 마지막 수신 좌표 목록 (m)
        self.latest_weights = []
        self.pose_seq = 0              # 수신 카운터 (새 데이터 판별용)
        self.busy = False

        # ------------------------------------------------------------
        # 로봇 서비스 (raw 모드에서는 사용하지 않음)
        # ------------------------------------------------------------
        self.cli = None
        if self.mode == 'move':
            self.cli = self.create_client(
                MoveLine, '/dsr01/motion/move_line', callback_group=self.group)
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('로봇 서비스 대기 중...')
            self.get_logger().info('로봇 서비스 연결 완료')

        self.get_logger().info(
            f"=== 토마토 좌표 검증 노드 시작 (mode: {self.mode}) ===\n"
            f"    시작: ros2 topic pub --once /align_test/start std_msgs/msg/Empty \"{{}}\"")

    # ================================================================
    # 수신 콜백
    # ================================================================
    def pose_callback(self, msg):
        self.latest_poses = msg.poses
        self.pose_seq += 1

    def weight_callback(self, msg):
        self.latest_weights = list(msg.data)

    # ================================================================
    # 촬영 요청 후 결과 대기
    # ================================================================
    def capture_and_wait(self, timeout=5.0):
        """트리거를 보내고 새 결과가 올 때까지 대기. (poses, weights) 반환"""
        prev_seq = self.pose_seq
        self.capture_pub.publish(Empty())

        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.pose_seq != prev_seq:
                return self.latest_poses, self.latest_weights
            time.sleep(0.05)

        self.get_logger().error('촬영 결과 수신 시간 초과')
        return None, None

    # ================================================================
    # 시작 트리거
    # ================================================================
    def start_callback(self, msg):
        if self.busy:
            self.get_logger().warn('이미 작업 중입니다.')
            return
        self.busy = True
        try:
            if self.mode == 'raw':
                self.run_raw()
            else:
                self.run_move()
        except Exception as e:
            import traceback
            self.get_logger().error(f'작업 중 오류:\n{traceback.format_exc()}')
        finally:
            self.busy = False

    # ================================================================
    # raw 모드 — 로봇을 움직이지 않고 값만 출력
    # ================================================================
    def run_raw(self):
        poses, weights = self.capture_and_wait()
        if poses is None:
            return
        if not poses:
            self.get_logger().warn('검출된 토마토가 없습니다.')
            return

        self.get_logger().info(f'--- 카메라 기준 원시 좌표 ({len(poses)}개) ---')
        for i, p in enumerate(poses):
            x_mm = p.position.x * 1000.0
            y_mm = p.position.y * 1000.0
            z_mm = p.position.z * 1000.0
            w = weights[i] if i < len(weights) else float('nan')
            dx, dy = camera_to_robot_delta(x_mm, y_mm)
            self.get_logger().info(
                f'  [{i}] 카메라  X: {x_mm:+8.1f} mm | Y: {y_mm:+8.1f} mm | Z: {z_mm:8.1f} mm | 무게 {w:.1f} g\n'
                f'       현재 매핑 적용 시 로봇 이동량 -> dx: {dx:+8.1f} mm | dy: {dy:+8.1f} mm')

        self.get_logger().info(
            '토마토를 화면 상하좌우로 옮겨가며 X, Y 부호를 확인한 뒤 '
            'camera_to_robot_delta() 를 수정하십시오.')

    # ================================================================
    # move 모드 — 홈 이동 -> 측정 -> 상대 이동 -> 재측정
    # ================================================================
    def run_move(self):
        # ---------- 홈 이동 ----------
        self.get_logger().info('① 홈 위치로 이동')
        self.send_move_line(self.home_pos)
        time.sleep(self.settle_sec)

        # 누적 방식: 현재 목표 좌표를 코드에서 추적
        cur_pos = list(self.home_pos)
        history = []          # (회차, cam_x, cam_y, cam_z, 거리, dx, dy)
        converged = False

        for it in range(1, self.max_iter + 1):
            # ---------- 측정 ----------
            poses, weights = self.capture_and_wait()
            if poses is None:
                return
            if not poses:
                self.get_logger().warn(f'[{it}회] 검출된 토마토가 없습니다. 중단합니다.')
                return
            if len(poses) > 1 and it == 1:
                self.get_logger().warn(
                    f'토마토가 {len(poses)}개 검출되었습니다. 첫 번째 개체로 진행합니다.')

            p = poses[0]
            cam_x = p.position.x * 1000.0
            cam_y = p.position.y * 1000.0
            cam_z = p.position.z * 1000.0
            err = (cam_x ** 2 + cam_y ** 2) ** 0.5
            w = weights[0] if weights else float('nan')

            self.get_logger().info(
                f'[{it}회 측정] 카메라  X: {cam_x:+7.1f} | Y: {cam_y:+7.1f} | '
                f'Z: {cam_z:7.1f} mm | 거리 {err:6.1f} mm | 무게 {w:.1f} g')

            # ---------- 수렴 판정 ----------
            if err <= self.tolerance:
                history.append((it, cam_x, cam_y, cam_z, err, 0.0, 0.0))
                converged = True
                self.get_logger().info(
                    f'✅ {it}회 만에 수렴했습니다 (기준 {self.tolerance:.1f} mm)')
                break

            if it == self.max_iter:
                history.append((it, cam_x, cam_y, cam_z, err, 0.0, 0.0))
                self.get_logger().warn(
                    f'최대 반복({self.max_iter}회)에 도달했습니다. 잔차 {err:.1f} mm')
                break

            # ---------- 축 매핑 + 상한 클램프 ----------
            dx, dy = camera_to_robot_delta(cam_x, cam_y)
            dx_c = max(-self.max_step, min(self.max_step, dx))
            dy_c = max(-self.max_step, min(self.max_step, dy))
            if dx_c != dx or dy_c != dy:
                self.get_logger().warn(
                    f'  이동량이 상한({self.max_step:.0f} mm)을 초과해 제한했습니다. '
                    f'dx {dx:+.1f} -> {dx_c:+.1f} | dy {dy:+.1f} -> {dy_c:+.1f}')

            history.append((it, cam_x, cam_y, cam_z, err, dx_c, dy_c))

            # ---------- 상대 이동 (누적) ----------
            cur_pos[0] += dx_c
            cur_pos[1] += dy_c
            self.get_logger().info(
                f'  -> 이동  dx: {dx_c:+7.1f} | dy: {dy_c:+7.1f} mm  '
                f'| 목표 X: {cur_pos[0]:.1f} | Y: {cur_pos[1]:.1f} (Z 고정 {cur_pos[2]:.1f})')
            self.send_move_line(cur_pos)
            time.sleep(self.settle_sec)

        # ---------- 결과 요약 ----------
        lines = ['========== 반복 정렬 결과 ==========',
                 '  회차 | 카메라 X | 카메라 Y |   거리 | 카메라 Z |     이동 dx |     이동 dy']
        for (it, cx, cy, cz, e, dx, dy) in history:
            move = f'{dx:+11.1f} | {dy:+11.1f}' if (dx or dy) else f'{"-":>11} | {"-":>11}'
            lines.append(
                f'  {it:4d} | {cx:+8.1f} | {cy:+8.1f} | {e:6.1f} | {cz:8.1f} | {move}')

        first_err = history[0][4]
        last_err = history[-1][4]
        z_drift = history[-1][3] - history[0][3]
        lines.append('  ---------------------------------')
        lines.append(f'  시작 오차 {first_err:.1f} mm -> 최종 오차 {last_err:.1f} mm '
                     f'| Z 변화 {z_drift:+.1f} mm')

        # 1회차 이동 효율 (스케일 진단용)
        if len(history) >= 2 and history[0][4] > 0:
            ratio = 1.0 - history[1][4] / history[0][4]
            lines.append(f'  1회차 보정률: {ratio * 100:.1f} %  '
                         f'(100%에 가까울수록 스케일이 정확)')
        lines.append('===================================')
        self.get_logger().info('\n'.join(lines))

        if converged:
            self.get_logger().info('판정: 양호. 좌표와 축 매핑이 맞습니다.')
        elif last_err < first_err * 0.5:
            self.get_logger().info(
                '판정: 수렴 중이나 기준에 미달했습니다. 반복 횟수를 늘리거나 기준을 완화해 보십시오.')
        else:
            self.get_logger().warn(
                '판정: 수렴하지 않았습니다. camera_to_robot_delta() 의 축 대응 또는 부호를 재확인하십시오.')

    # ================================================================
    # 로봇 이동
    # ================================================================
    def send_move_line(self, pos):
        req = MoveLine.Request()
        req.pos = [float(v) for v in pos]
        req.vel, req.acc = [100.0, 100.0], [100.0, 100.0]   # 검증용으로 느리게
        future = self.cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = TomatoAlignTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
