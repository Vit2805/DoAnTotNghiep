import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor

class ScanMerger360(Node):
    def __init__(self):
        super().__init__('scan_merger_360')
        
        # ==================== PARAMETERS ====================
        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('front_topic', '/scan_front', descriptor=desc)
        self.declare_parameter('rear_topic', '/scan_rear', descriptor=desc)
        self.declare_parameter('output_topic', '/scan_360', descriptor=desc)
        self.declare_parameter('target_frame', 'base_link', descriptor=desc)

        # Cấu hình vị trí Lidar (Khớp với TF Static trong Launch file)
        self.declare_parameter('front_tx', 0.42)
        self.declare_parameter('front_ty', 0.29)
        self.declare_parameter('front_yaw', 3.14159)
        self.declare_parameter('rear_tx', -0.42)
        self.declare_parameter('rear_ty', -0.29)
        self.declare_parameter('rear_yaw', 0.0)

        # Cấu hình Masking (Lọc vỏ xe) - QUAN TRỌNG
        self.declare_parameter('front_mask_box', [0.0, 0.40, -0.1, 0.30], descriptor=desc)
        self.declare_parameter('rear_mask_box', [0.0, 0.40, -0.1, 0.30], descriptor=desc)
        self.declare_parameter('body_polygon', [], descriptor=desc)

        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('angle_increment', 0.01745) # ~1 độ
        self.declare_parameter('scan_frequency', 15.0)

        # ==================== GET VALUES ====================
        self.front_topic = self.get_parameter('front_topic').value
        self.rear_topic = self.get_parameter('rear_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('target_frame').value

        self.f_tx = self.get_parameter('front_tx').value
        self.f_ty = self.get_parameter('front_ty').value
        self.f_yaw = self.get_parameter('front_yaw').value

        self.r_tx = self.get_parameter('rear_tx').value
        self.r_ty = self.get_parameter('rear_ty').value
        self.r_yaw = self.get_parameter('rear_yaw').value

        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_inc = self.get_parameter('angle_increment').value
        freq = self.get_parameter('scan_frequency').value

        self.front_mask = self.parse_box(self.get_parameter('front_mask_box').value)
        self.rear_mask = self.parse_box(self.get_parameter('rear_mask_box').value)
        self.body_polygon = self.parse_polygon(self.get_parameter('body_polygon').value)

        # ==================== INIT ====================
        self.create_subscription(LaserScan, self.front_topic, self.front_cb, 10)
        self.create_subscription(LaserScan, self.rear_topic, self.rear_cb, 10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.scan_front = None
        self.scan_rear = None

        # Tính toán trước các bins
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.num_bins = int(np.ceil((self.angle_max - self.angle_min) / self.angle_inc))
        
        # Timer chạy độc lập để ổn định FPS
        self.create_timer(1.0 / freq, self.timer_cb)
        self.get_logger().info('Numpy Merger V2 (With Masking) Started')

    # --- Tiện ích Parse Param ---
    def parse_box(self, val):
        if isinstance(val, (list, tuple)) and len(val) == 4: return val
        return None

    def parse_polygon(self, val):
        # Hỗ trợ list phẳng [x1, y1, x2, y2...]
        if isinstance(val, (list, tuple)) and len(val) >= 6: return val
        return []

    def front_cb(self, msg): self.scan_front = msg
    def rear_cb(self, msg): self.scan_rear = msg

    # --- Xử lý Numpy (Đã thêm Masking) ---
    def process_lidar_numpy(self, scan_msg, tx, ty, yaw, mask_box):
        if scan_msg is None: return None, None
        
        ranges = np.array(scan_msg.ranges)
        # Tạo mảng góc
        angles = np.linspace(scan_msg.angle_min, 
                             scan_msg.angle_min + scan_msg.angle_increment * (len(ranges)-1), 
                             len(ranges))
        
        # 1. Lọc dữ liệu rác
        valid = (ranges >= self.range_min) & (ranges <= self.range_max) & (np.isfinite(ranges))
        r_v = ranges[valid]
        a_v = angles[valid]
        
        # 2. Tính tọa độ Sensor Frame (để lọc Mask Box)
        cos_a = np.cos(a_v)
        sin_a = np.sin(a_v)
        x_s = r_v * cos_a
        y_s = r_v * sin_a

        # 3. ÁP DỤNG MASK BOX (Lọc điểm trúng vỏ xe)
        if mask_box:
            xmin, xmax, ymin, ymax = mask_box
            # Giữ lại những điểm NẰM NGOÀI hộp
            mask_keep = ~((x_s >= xmin) & (x_s <= xmax) & (y_s >= ymin) & (y_s <= ymax))
            x_s = x_s[mask_keep]
            y_s = y_s[mask_keep]
            r_v = r_v[mask_keep] # Cập nhật lại r để dùng sau

        if len(x_s) == 0: return None, None

        # 4. Transform sang Base Frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_b = tx + x_s * cos_yaw - y_s * sin_yaw
        y_b = ty + x_s * sin_yaw + y_s * cos_yaw
        
        # 5. Đổi lại sang Polar (Base Frame)
        # Lưu ý: Ta tính lại góc angle_b nhưng giữ nguyên r_v (khoảng cách thực)
        # Hoặc tính lại r_b từ x_b, y_b (chính xác hơn cho map)
        r_b = np.sqrt(x_b**2 + y_b**2)
        angle_b = np.arctan2(y_b, x_b)
        
        return r_b, angle_b

    def point_in_polygon_fast(self, r_arr, ang_arr, poly):
        # Lọc Polygon đơn giản trên Base Frame
        if not poly: return
        x = r_arr * np.cos(ang_arr)
        y = r_arr * np.sin(ang_arr)
        
        # Ray casting algorithm (phiên bản đơn giản cho list điểm)
        n = len(poly) // 2
        inside = np.zeros(len(x), dtype=bool)
        
        px = poly[0::2]
        py = poly[1::2]
        
        j = n - 1
        for i in range(n):
            xi, yi = px[i], py[i]
            xj, yj = px[j], py[j]
            
            intersect = ((yi > y) != (yj > y)) & \
                        (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            inside ^= intersect
            j = i
            
        # Gán vô cực cho điểm nằm trong polygon
        r_arr[inside] = np.inf

    def timer_cb(self):
        merged_ranges = np.full(self.num_bins, np.inf, dtype=np.float32)
        
        # --- FIX TIMESTAMP: Lấy giờ của Lidar thay vì giờ hệ thống ---
        out_stamp = self.get_clock().now().to_msg()
        if self.scan_front: out_stamp = self.scan_front.header.stamp
        if self.scan_rear and self.scan_front:
            if self.scan_rear.header.stamp.sec > self.scan_front.header.stamp.sec:
                out_stamp = self.scan_rear.header.stamp

        # Xử lý Front
        r_f, a_f = self.process_lidar_numpy(self.scan_front, self.f_tx, self.f_ty, self.f_yaw, self.front_mask)
        if r_f is not None:
            idx = ((a_f - self.angle_min) / self.angle_inc).astype(int)
            mask = (idx >= 0) & (idx < self.num_bins)
            np.minimum.at(merged_ranges, idx[mask], r_f[mask])

        # Xử lý Rear
        r_r, a_r = self.process_lidar_numpy(self.scan_rear, self.r_tx, self.r_ty, self.r_yaw, self.rear_mask)
        if r_r is not None:
            idx = ((a_r - self.angle_min) / self.angle_inc).astype(int)
            mask = (idx >= 0) & (idx < self.num_bins)
            np.minimum.at(merged_ranges, idx[mask], r_r[mask])

        # Lọc Body Polygon (Nếu có)
        if self.body_polygon:
            # Tạo mảng góc cho merged_ranges
            merged_angles = self.angle_min + np.arange(self.num_bins) * self.angle_inc
            # Chỉ check những điểm hữu hạn
            finite_mask = np.isfinite(merged_ranges)
            if np.any(finite_mask):
                self.point_in_polygon_fast(merged_ranges, merged_angles, self.body_polygon)

        # Publish
        msg = LaserScan()
        msg.header.stamp = out_stamp
        msg.header.frame_id = self.output_frame
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_inc
        msg.scan_time = 1.0 / 15.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = merged_ranges.tolist()
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger360()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()