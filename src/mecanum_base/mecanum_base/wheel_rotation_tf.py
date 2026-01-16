#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class WheelRotationTF(Node):
    def __init__(self):
        super().__init__("wheel_rotation_tf")

        # =========================================================
        # 1. LOAD THAM SỐ
        # =========================================================
        self.declare_parameter('robot.geometry.lx', 0.240)
        self.declare_parameter('robot.geometry.ly', 0.235)
        self.declare_parameter('robot.geometry.base_z', 0.0) 
        self.declare_parameter('robot.wheels.ticks_per_rev', 6864.0)
        self.declare_parameter('ticks_topic', '/wheel_ticks_debug')

        self.lx = self.get_parameter('robot.geometry.lx').value
        self.ly = self.get_parameter('robot.geometry.ly').value
        self.z  = self.get_parameter('robot.geometry.base_z').value
        self.TPR = self.get_parameter('robot.wheels.ticks_per_rev').value
        topic_name = self.get_parameter('ticks_topic').value

        self.get_logger().info(f"Visualizer Fixed Order: FL-FR-RR-RL")

        # =========================================================
        # 2. SỬA LỖI VỊ TRÍ (MAPPING CHUẨN: FL - FR - RR - RL)
        # =========================================================
        self.parent_frame = "base_link"
        
        # Thứ tự Data nhận được: [0]=FL, [1]=FR, [2]=RR, [3]=RL
        # Quy tắc tọa độ ROS: 
        # - X Dương: Phía Trước
        # - Y Dương: Bên Trái
        self.wheel_positions = [
            ( self.lx,  self.ly, self.z),  # [0] FL (Trước Trái): X+, Y+
            ( self.lx, -self.ly, self.z),  # [1] FR (Trước Phải): X+, Y-
            (-self.lx, -self.ly, self.z),  # [2] RR (Sau Phải):   X-, Y-
            (-self.lx,  self.ly, self.z)   # [3] RL (Sau Trái):   X-, Y+
        ]
        
        # Tên tương ứng (Phải khớp thứ tự trên)
        self.child_frames = [
            "wheel_fl_visual",
            "wheel_fr_visual",
            "wheel_rr_visual",
            "wheel_rl_visual", 
        ]

        # Sửa chiều quay (Thử để 1.0 trước, nếu ngược thì đổi thành -1.0)
        self.encoder_direction = 1.0

        # =========================================================
        # 3. SỬA LỖI TRỤC QUAY NGƯỢC
        # =========================================================
        # Nếu bạn thấy trục X (Màu đỏ) quay về phía sau, ta cần đảo dấu
        # Hoặc xoay trục 180 độ. Ở đây mình đảo chiều quay encoder.
        self.encoder_direction = 1.0 # Đổi thành 1.0 nếu muốn quay ngược lại

        self.prev_ticks = [None, None, None, None]
        self.theta = [0.0, 0.0, 0.0, 0.0]

        self.tf_br = TransformBroadcaster(self)
        self.sub_ticks = self.create_subscription(Int32MultiArray, topic_name, self.ticks_cb, 10)

    def ticks_cb(self, msg):
        current_ticks = msg.data
        if len(current_ticks) < 4: return

        if self.prev_ticks[0] is None:
            self.prev_ticks = current_ticks
            return

        for i in range(4):
            delta = current_ticks[i] - self.prev_ticks[i]
            
            # Tính góc quay (có nhân thêm chiều direction)
            d_angle = (delta / self.TPR) * (2 * math.pi) * self.encoder_direction
            self.theta[i] += d_angle
            
        self.prev_ticks = current_ticks
        self.publish_wheel_tf(self.get_clock().now())

    def publish_wheel_tf(self, now):
        def make_tf(child, x, y, z, angle):
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = child
            
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            
            # Xoay quanh trục Y (Pitch)
            cy = math.cos(angle * 0.5)
            sy = math.sin(angle * 0.5)
            
            # Quaternion chuẩn (X-Forward, Y-Left, Z-Up)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = sy
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = cy
            return t

        tfs = []
        for i in range(4):
            wx, wy, wz = self.wheel_positions[i]
            tfs.append(make_tf(self.child_frames[i], wx, wy, wz, self.theta[i]))
            
        self.tf_br.sendTransform(tfs)

def main(args=None):
    rclpy.init(args=args)
    node = WheelRotationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()