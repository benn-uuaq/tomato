#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
import os
import time
from scipy.optimize import least_squares

class TomatoVolumeEstimator(Node):
    def __init__(self):
        super().__init__('tomato_volume_estimator')
        
        # 📂 탐색할 PCD 파일 경로 및 모니터링 설정
        self.pcd_dir = "/home/smartfarm/Desktop/pcd files"
        self.processed_files = set(os.listdir(self.pcd_dir)) # 시작 시 기존 파일들은 제외
        
        # ⏱️ 2초마다 폴더를 감시하는 타이머 생성
        self.timer = self.create_timer(2.0, self.watch_directory_callback)
        self.get_logger().info(f'🍅 부피 추정 노드가 가동되었습니다. 감시 폴더: {self.pcd_dir}')

    def watch_directory_callback(self):
        """폴더를 감시하다가 새 PCD 파일이 들어오면 부피 계산을 트리거합니다."""
        if not os.path.exists(self.pcd_dir):
            return

        current_files = set(os.listdir(self.pcd_dir))
        new_files = current_files - self.processed_files

        for file_name in new_files:
            if file_name.endswith('.pcd'):
                full_path = os.path.join(self.pcd_dir, file_name)
                self.get_logger().info(f'🆕 새 토마토 데이터 감지: {file_name}')
                
                # 파일 쓰기가 완전히 끝날 때까지 잠시 대기 (0.2초)
                time.sleep(0.2) 
                
                # 부피 계산 실행
                self.estimate_volume(full_path)
                
        self.processed_files = current_files

    def estimate_volume(self, pcd_path):
        try:
            # 1. PCD 파일 로드
            pcd = o3d.io.read_point_cloud(pcd_path)
            points = np.asarray(pcd.points)

            if len(points) < 20:
                self.get_logger().warn('❌ 포인트 수가 너무 적어 부피를 계산할 수 없습니다.')
                return

            # 2. 데이터 중심점 정렬 (수학적 연산 안정화)
            centroid = np.mean(points, axis=0)
            pts_centered = points - centroid

            # 3. 타원체 피팅용 잔차(Residual) 함수 정의
            def ellipsoid_residuals(params, pts):
                a, b, c = params
                return (pts[:, 0]**2 / a**2) + (pts[:, 1]**2 / b**2) + (pts[:, 2]**2 / c**2) - 1

            # 4. 초기 예측값(Initial Guess) 설정 (경계상자 크기 기반)
            bbox = pcd.get_axis_aligned_bounding_box()
            extent = bbox.get_extent()
            
            # 리얼센스 단위(m) 기준 초기값 세팅 (가로, 세로, 높이의 절반 수준)
            initial_guess = [max(extent[0]/2, 0.01), max(extent[1]/2, 0.01), max(extent[2], 0.01)]

            # 5. 최적화 알고리즘 가동 (타원체 피팅)
            # 💡 args=(pts_centered,)를 통해 목적 함수에 중심 정렬된 포인트를 정상 전달합니다.
            result = least_squares(
                ellipsoid_residuals, 
                initial_guess, 
                args=(pts_centered,), 
                bounds=((0.001, 0.001, 0.001), (0.2, 0.2, 0.2))
            )
            a, b, c = result.x

            # 6. 타원체 공식 기반 부피 계산 (m³ -> cm³ 변환)
            # 💡 리얼센스는 기본 단위가 미터(m)이므로 결과에 1,000,000을 곱해야 cm³(cc, ml) 단위가 됩니다.
            volume_m3 = (4.0 / 3.0) * np.pi * a * b * c
            volume_cm3 = volume_m3 * 1e6

            # 7. 결과 출력
            self.get_logger().info('=' * 40)
            self.get_logger().info(f'📊 [계산 결과] 파일명: {os.path.basename(pcd_path)}')
            self.get_logger().info(f'   - 가로 반경(a): {a*100:.2f} cm (지름: {a*200:.2f} cm)')
            self.get_logger().info(f'   - 세로 반경(b): {b*100:.2f} cm (지름: {b*200:.2f} cm)')
            self.get_logger().info(f'   - 높이 반경(c): {c*100:.2f} cm (지름: {c*200:.2f} cm)')
            self.get_logger().info(f'   🔥 추정 부피: {volume_cm3:.2f} cm³ (ml)')
            self.get_logger().info('=' * 40)

        except Exception as e:
            self.get_logger().error(f'❌ 부피 계산 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TomatoVolumeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()