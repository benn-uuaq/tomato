import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO


class YoloSegNodeD435i(Node):
    """학습한 딸기 segmentation 모델을 D435i 컬러 영상에 실시간으로 돌려
    마스크 품질을 눈으로 확인하기 위한 검증 전용 노드."""

    def __init__(self):
        super().__init__('yolo_seg_node_d435i')

        self.declare_parameter('model_path', '')
        self.declare_parameter('yolo_conf', 0.4)
        self.declare_parameter('show_window', True)
        self.declare_parameter('mask_alpha', 0.45)

        model_path = self.get_parameter('model_path').value
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.mask_alpha = float(self.get_parameter('mask_alpha').value)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 인스턴스별로 구분되는 색상 팔레트 (붙어있는 딸기가 분리되는지 확인용)
        rng = np.random.default_rng(0)
        self.palette = rng.integers(60, 255, size=(64, 3), dtype=np.uint8)

        self.count_buffer = []
        self.last_print_time = time.time()

        self.get_logger().info(f"D435i Segmentation 검증 노드 실행 중... (model: {model_path})")

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(cv_image, verbose=False, conf=self.yolo_conf)

        n_masks = 0
        overlay = cv_image.copy()

        for r in results:
            if r.masks is None:
                continue

            polygons = r.masks.xy
            confs = r.boxes.conf.cpu().numpy() if r.boxes is not None else []

            for i, poly in enumerate(polygons):
                if poly is None or len(poly) < 3:
                    continue

                color = tuple(int(c) for c in self.palette[i % len(self.palette)])
                pts = poly.astype(np.int32).reshape(-1, 1, 2)

                # 마스크 내부를 인스턴스별 색으로 채우고, 외곽선은 진하게
                cv2.fillPoly(overlay, [pts], color)
                cv2.polylines(cv_image, [pts], True, color, 2)

                conf = float(confs[i]) if i < len(confs) else 0.0
                cx, cy = poly.mean(axis=0).astype(int)
                cv2.putText(cv_image, f"{conf:.2f}", (cx - 20, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                n_masks += 1

        cv_image = cv2.addWeighted(overlay, self.mask_alpha, cv_image, 1 - self.mask_alpha, 0)

        cv2.putText(cv_image, f"count: {n_masks}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        self.count_buffer.append(n_masks)

        # 1초마다 평균 검출 개수 출력 (프레임마다 흔들리는지 확인용)
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            if self.count_buffer:
                avg = sum(self.count_buffer) / len(self.count_buffer)
                self.get_logger().info(
                    f"[1s Avg | Frames: {len(self.count_buffer)}] "
                    f"detected: {avg:.1f} (min {min(self.count_buffer)}, max {max(self.count_buffer)})"
                )
                self.count_buffer.clear()
            self.last_print_time = current_time

        if self.show_window:
            cv2.imshow("D435i YOLO Segmentation Visualizer", cv_image)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloSegNodeD435i()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
