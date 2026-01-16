import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor

class DualLidarMerger(Node):
    def __init__(self):
        # [QUAN TRỌNG] Tên node phải khớp với mục trong YAML
        super().__init__('scan_merger_360') 
        
        # =========================================================
        # 1. KHAI BÁO THAM SỐ (Mapping với file YAML)
        # =========================================================
        
        # Topic Configuration
        self.declare_parameter('front_topic', '/scan_front')
        self.declare_parameter('rear_topic', '/scan_rear')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('output_frame', 'base_link')

        # Geometry Configuration (Vị trí Lidar trên xe)
        # Cập nhật mặc định 0.42 cho khớp robot của bạn
        self.declare_parameter('front_tx', 0.42) 
        self.declare_parameter('front_ty', 0.29)
        self.declare_parameter('front_yaw', 3.14159) # Lidar trước quay ngược
        
        self.declare_parameter('rear_tx', -0.42)
        self.declare_parameter('rear_ty', -0.29)
        self.declare_parameter('rear_yaw', 0.0)      # Lidar sau quay xuôi

        # Filter Configuration (Khoảng cách quét)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('angle_increment', 0.01745) # ~1 độ (Nhẹ hơn 0.0058 cũ)

        # =========================================================
        # 2. LẤY GIÁ TRỊ PARAM
        # =========================================================
        front_topic = self.get_parameter('front_topic').value
        rear_topic = self.get_parameter('rear_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.front_tx = self.get_parameter('front_tx').value
        self.front_ty = self.get_parameter('front_ty').value
        self.front_yaw = self.get_parameter('front_yaw').value

        self.rear_tx = self.get_parameter('rear_tx').value
        self.rear_ty = self.get_parameter('rear_ty').value
        self.rear_yaw = self.get_parameter('rear_yaw').value

        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.target_increment = self.get_parameter('angle_increment').value

        # =========================================================
        # 3. KHỞI TẠO PUB/SUB
        # =========================================================
        self.sub_front = self.create_subscription(LaserScan, front_topic, self.front_callback, 10)
        self.sub_rear = self.create_subscription(LaserScan, rear_topic, self.rear_callback, 10)
        self.pub_merged = self.create_publisher(LaserScan, self.output_topic, 10)

        # Cache dữ liệu
        self.scan_front = None
        self.scan_rear = None

        self.get_logger().info(f'Scan Merger Started. Output: {self.output_topic}')

    def front_callback(self, msg: LaserScan):
        self.scan_front = msg
        # Chỉ merge khi đã có đủ cả 2 (hoặc cơ chế lỏng hơn tùy bạn)
        if self.scan_rear is not None:
            self.merge_and_publish()

    def rear_callback(self, msg: LaserScan):
        self.scan_rear = msg
        if self.scan_front is not None:
            self.merge_and_publish()

    def transform_point(self, r, angle, tx, ty, yaw):
        # Chuyển đổi tọa độ cực (r, angle) -> Cartesian (x, y) trên base_link
        x_s = r * math.cos(angle)
        y_s = r * math.sin(angle)
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        x_b = tx + cos_yaw * x_s - sin_yaw * y_s
        y_b = ty + sin_yaw * x_s + cos_yaw * y_s
        return x_b, y_b

    def merge_and_publish(self):
        front = self.scan_front
        rear = self.scan_rear
        
        # Cấu hình Output Scan (Quét 360 độ)
        angle_min = -math.pi
        angle_max = math.pi
        angle_inc = self.target_increment
        
        # Số lượng điểm quét
        num_bins = int(round((angle_max - angle_min) / angle_inc))
        merged_ranges = [float('inf')] * num_bins

        # Hàm nội bộ để xử lý từng Lidar
        def process_scan(scan, tx, ty, yaw):
            if not scan: return
            
            for i, r in enumerate(scan.ranges):
                # 1. Lọc dữ liệu rác
                if not math.isfinite(r) or r < self.range_min or r > self.range_max:
                    continue

                # 2. Tính tọa độ trên Base Link
                angle_sensor = scan.angle_min + i * scan.angle_increment
                x_b, y_b = self.transform_point(r, angle_sensor, tx, ty, yaw)
                
                # 3. Tính lại khoảng cách & góc so với tâm xe
                r_b = math.hypot(x_b, y_b)
                angle_b = math.atan2(y_b, x_b)

                # 4. Gán vào mảng merged (Tìm vị trí index tương ứng)
                if angle_b < angle_min or angle_b > angle_max: continue
                
                idx = int((angle_b - angle_min) / angle_inc)
                
                # 5. Logic gộp: Lấy điểm GẦN NHẤT (Min range)
                if 0 <= idx < num_bins:
                    if r_b < merged_ranges[idx]:
                        merged_ranges[idx] = r_b

        # Xử lý cả 2 Lidar
        process_scan(front, self.front_tx, self.front_ty, self.front_yaw)
        process_scan(rear, self.rear_tx, self.rear_ty, self.rear_yaw)

        # Tạo gói tin Output
        out = LaserScan()
        out.header.frame_id = self.output_frame
        out.header.stamp = self.get_clock().now().to_msg() # Lấy giờ hiện tại để tránh trễ TF
        
        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = angle_inc
        out.time_increment = 0.0
        out.scan_time = 0.1 # 10Hz
        out.range_min = self.range_min
        out.range_max = self.range_max
        out.ranges = merged_ranges

        self.pub_merged.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()