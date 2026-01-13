import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class DualLidarMerger(Node):
    def __init__(self):
        super().__init__('dual_lidar_merger')
        
        # Topic input
        front_topic = self.declare_parameter('front_topic', '/scan_front').get_parameter_value().string_value
        rear_topic = self.declare_parameter('rear_topic', '/scan_rear').get_parameter_value().string_value
        
        # Frame output
        self.output_frame = self.declare_parameter('output_frame', 'base_link').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('output_topic', '/scan_merged').get_parameter_value().string_value
        
        # Vị trí lidar (phải đồng bộ với TF trong launch)
        # Lidar front
        self.front_tx = self.declare_parameter('front_tx', 0.3).get_parameter_value().double_value
        self.front_ty = self.declare_parameter('front_ty', 0.0).get_parameter_value().double_value
        self.front_yaw = self.declare_parameter('front_yaw', 0.0).get_parameter_value().double_value
        
        # Lidar rear
        self.rear_tx = self.declare_parameter('rear_tx', -0.3).get_parameter_value().double_value
        self.rear_ty = self.declare_parameter('rear_ty', 0.0).get_parameter_value().double_value
        self.rear_yaw = self.declare_parameter('rear_yaw', math.pi).get_parameter_value().double_value

        # Subscriber
        self.sub_front = self.create_subscription(LaserScan, front_topic, self.front_callback, 10)
        self.sub_rear = self.create_subscription(LaserScan, rear_topic, self.rear_callback, 10)

        # Publisher
        self.pub_merged = self.create_publisher(LaserScan, self.output_topic, 10)

        # Lưu scan mới nhất
        self.scan_front = None
        self.scan_rear = None
        self.get_logger().info(f'Merger listening on {front_topic} and {rear_topic}, publishing {self.output_topic}')

    def front_callback(self, msg: LaserScan):
        self.scan_front = msg
        if self.scan_rear is not None:
            self.merge_and_publish()

    def rear_callback(self, msg: LaserScan):
        self.scan_rear = msg
        if self.scan_front is not None:
            self.merge_and_publish()

    def transform_point(self, r, angle, tx, ty, yaw):
        """
        Biến 1 điểm (r, angle) trong frame của sensor sang toạ độ (x, y) trong base_link.
        """
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
        if front is None or rear is None:
            return

        # Giả sử 2 lidar cùng cấu hình góc
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = front.angle_increment if front.angle_increment > 0.0 else 0.0058
        num_bins = int(round((angle_max - angle_min) / angle_increment)) + 1
        
        # Khởi tạo ranges = inf
        merged_ranges = [float('inf')] * num_bins
        range_min = min(front.range_min, rear.range_min)
        range_max = max(front.range_max, rear.range_max)

        # Helper để nạp 1 scan vào merged_ranges
        def accumulate_scan(scan: LaserScan, tx, ty, yaw):
            for i, r in enumerate(scan.ranges):
                if not math.isfinite(r):
                    continue
                if r < scan.range_min or r > scan.range_max:
                    continue
                
                angle_sensor = scan.angle_min + i * scan.angle_increment
                x_b, y_b = self.transform_point(r, angle_sensor, tx, ty, yaw)
                r_b = math.hypot(x_b, y_b)
                
                if r_b < range_min or r_b > range_max:
                    continue
                
                angle_b = math.atan2(y_b, x_b)
                
                # Bỏ qua nếu ngoài [-pi, pi]
                if angle_b < angle_min or angle_b > angle_max:
                    continue
                
                idx = int(round((angle_b - angle_min) / angle_increment))
                if 0 <= idx < num_bins:
                    if r_b < merged_ranges[idx]:
                        merged_ranges[idx] = r_b

        # Tích luỹ front + rear
        accumulate_scan(front, self.front_tx, self.front_ty, self.front_yaw)
        accumulate_scan(rear, self.rear_tx, self.rear_ty, self.rear_yaw)

        # Tạo LaserScan output
        out = LaserScan()
        out.header.frame_id = self.output_frame
        
        # Lấy timestamp mới nhất
        if front.header.stamp.sec > rear.header.stamp.sec or \
           (front.header.stamp.sec == rear.header.stamp.sec and front.header.stamp.nanosec >= rear.header.stamp.nanosec):
            out.header.stamp = front.header.stamp
        else:
            out.header.stamp = rear.header.stamp
            
        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = angle_increment
        out.time_increment = front.time_increment
        out.scan_time = front.scan_time
        out.range_min = range_min
        out.range_max = range_max
        out.ranges = merged_ranges
        
        self.pub_merged.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
