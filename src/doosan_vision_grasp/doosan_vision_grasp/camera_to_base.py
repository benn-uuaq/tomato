
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from std_msgs.msg import Float64, Bool  # Bool 추가
import tf2_ros
import math
import numpy as np

class CameraToBaseTF(Node):
    def __init__(self):
        super().__init__('camera_to_base')
        
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publish_static_transform()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.window_size = 30  # 1초 분량 데이터      
        self.pos_buffer = []         
        self.ang_buffer = []         
        self.weight_buffer = []  # [추가] 무게 버퍼
        self.pos_var_threshold = 0.0001  
        self.ang_var_threshold = 1.0     
        
        self.sub_point = self.create_subscription(PointStamped, '/object_in_camera', self.callback, 10)
        self.sub_angle = self.create_subscription(Float64, '/object_angle', self.angle_callback, 10)
        self.sub_weight = self.create_subscription(Float64, '/object_weight_raw', self.weight_callback, 10)
        
        # [추가] YOLO 스위치 구독 및 버퍼 초기화
        self.reset_sub = self.create_subscription(Bool, '/enable_yolo', self.reset_callback, 10)
        
        self.pub = self.create_publisher(Point, '/target_xyz', 10)
        self.angle_pub = self.create_publisher(Float64, '/filtered_angle', 10)
        self.weight_pub = self.create_publisher(Float64, '/filtered_weight', 10)
        
        self.current_strawberry_angle = 0.0
        self.current_strawberry_weight = 0.0
        self.get_logger().info('🚀 스위치 기반 버퍼 리셋 필터 노드 가동')

    def weight_callback(self, msg):
        self.current_strawberry_weight = msg.data

    def reset_callback(self, msg):
        # [추가] YOLO가 다시 가동될 때(True) 버퍼를 완전히 비웁니다.
        if msg.data is True:
            self.pos_buffer = []
            self.ang_buffer = []
            self.weight_buffer = [] # [추가]
            self.get_logger().info("🧹 버퍼를 비웠습니다. 새로운 데이터를 수집합니다.")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_6'       
        t.child_frame_id = 'camera_link'  
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = -0.009, -0.055, 0.0313
        t.transform.rotation.z, t.transform.rotation.w = 0.0, 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def angle_callback(self, msg):
        self.current_strawberry_angle = msg.data

    def filter_outliers(self, data_list):
        data = np.array(data_list)
        if len(data) < 10: return data
        if data.ndim == 1:
            lower = np.percentile(data, 15)
            upper = np.percentile(data, 85)
            return data[(data >= lower) & (data <= upper)]
        else:
            filtered = []
            for i in range(3):
                col = data[:, i]
                lower = np.percentile(col, 15)
                upper = np.percentile(col, 85)
                filtered_col = col[(col >= lower) & (col <= upper)]
                filtered.append(np.mean(filtered_col) if len(filtered_col) > 0 else np.mean(col))
            return np.array(filtered)

    def callback(self, msg):
        try:
            self.pos_buffer.append([msg.point.x, msg.point.y, msg.point.z])
            self.ang_buffer.append(self.current_strawberry_angle)
            self.weight_buffer.append(self.current_strawberry_weight)
            
            if len(self.pos_buffer) > self.window_size:
                self.pos_buffer.pop(0)
                self.ang_buffer.pop(0)
                self.weight_buffer.pop(0) # [추가]
            
            if len(self.pos_buffer) < self.window_size: return

            pos_data = np.array(self.pos_buffer)
            ang_data = np.array(self.ang_buffer)
            clean_ang = self.filter_outliers(ang_data)
            clean_pos_avg = self.filter_outliers(pos_data) 
            clean_weight = self.filter_outliers(self.weight_buffer)

            if len(clean_ang) > 0:
                avg_pos = clean_pos_avg 
                avg_ang = np.mean(clean_ang) 
                avg_weight = np.mean(clean_weight) # [추가] 필터링된 무게 평균

                st = TransformStamped()
                st.header.stamp = self.get_clock().now().to_msg()
                st.header.frame_id = 'camera_link'
                st.child_frame_id = 'strawberry'
                st.transform.translation.x, st.transform.translation.y, st.transform.translation.z = avg_pos
                
                yaw_rad = math.radians(avg_ang)
                st.transform.rotation.z, st.transform.rotation.w = math.sin(yaw_rad/2), math.cos(yaw_rad/2)
                self.tf_broadcaster.sendTransform(st)

                transform = self.tf_buffer.lookup_transform('base_link', 'strawberry', rclpy.time.Time())
                
                target_msg = Point()
                target_msg.x, target_msg.y, target_msg.z = transform.transform.translation.x*1000, transform.transform.translation.y*1000, transform.transform.translation.z*1000
                self.pub.publish(target_msg)
                self.angle_pub.publish(Float64(data=float(avg_ang)))
                self.weight_pub.publish(Float64(data=float(avg_weight))) # [추가]

        except Exception as e:
            pass

def main():
    rclpy.init()
    node = CameraToBaseTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()