#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class WheelTFNode(Node):
    def __init__(self):
        super().__init__("wheel_tf_node")
        # Vị trí 4 bánh xe (so với base_link)
        # Bạn có thể chỉnh lại tọa độ x, y cho đúng kích thước thật robot của bạn
        self.wheel_positions = {
            "wheel_fl": (0.25,  0.24, 0.0),   # Trước Trái
            "wheel_fr": (0.25, -0.24, 0.0),   # Trước Phải
            "wheel_rl": (-0.25,  0.24, 0.0),  # Sau Trái
            "wheel_rr": (-0.25, -0.24, 0.0),  # Sau Phải
        }
        self.br = StaticTransformBroadcaster(self)
        self.publish_static_tfs()
        self.get_logger().info("Wheel TF Node started (static transforms).")

    def publish_static_tfs(self):
        tf_list = []
        now = self.get_clock().now().to_msg()
        for name, (x, y, z) in self.wheel_positions.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "base_link"
            t.child_frame_id = name
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            t.transform.rotation.w = 1.0 # Không xoay
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            tf_list.append(t)
        self.br.sendTransform(tf_list)

def main(args=None):
    rclpy.init(args=args)
    node = WheelTFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
