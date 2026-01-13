#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class WheelRotationTF(Node):
    def __init__(self):
        super().__init__("wheel_rotation_tf")
        # Tham số cấu hình
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.07).value
        self.base_width = self.declare_parameter("base_width", 0.50).value
        # Lưu ý: topic phải khớp với cái mà robot_driver publish
        self.odom_topic = self.declare_parameter("odom_encoder_topic", "/wheel/odom").value

        # Tên các frame tĩnh (Parent)
        self.fl_parent = "wheel_fl"
        self.fr_parent = "wheel_fr"
        self.rl_parent = "wheel_rl"
        self.rr_parent = "wheel_rr"

        # Tên các frame động (Child - quay tròn)
        self.fl_child = "wheel_fl_spin"
        self.fr_child = "wheel_fr_spin"
        self.rl_child = "wheel_rl_spin"
        self.rr_child = "wheel_rr_spin"

        # Góc tích lũy của bánh xe
        self.theta_fl = 0.0
        self.theta_fr = 0.0
        self.theta_rl = 0.0
        self.theta_rr = 0.0
        self.last_time = None

        self.tf_br = TransformBroadcaster(self)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.get_logger().info(f"WheelRotationTF started listening on {self.odom_topic}")

    def odom_cb(self, msg):
        now = msg.header.stamp
        # Đổi giây và nano giây ra float
        t = now.sec + now.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time = t
            return
        
        dt = t - self.last_time
        if dt <= 0: return
        self.last_time = t

        # Lấy vận tốc dài và vận tốc góc từ Odom
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        # Tính vận tốc riêng từng bên bánh (Differential Drive)
        v_left = vx - wz * self.base_width / 2.0
        v_right = vx + wz * self.base_width / 2.0

        # Tính vận tốc góc quay của bánh xe (rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Cộng dồn góc quay
        self.theta_fl += omega_left * dt
        self.theta_rl += omega_left * dt
        self.theta_fr += omega_right * dt
        self.theta_rr += omega_right * dt

        self.publish_wheel_tf(now)

    def publish_wheel_tf(self, stamp):
        def make_tf(parent, child, theta):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            # Quay quanh trục Y (trục ngang của bánh xe)
            cy = math.cos(theta * 0.5)
            sy = math.sin(theta * 0.5)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = sy
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = cy
            return t

        tfs = [
            make_tf(self.fl_parent, self.fl_child, self.theta_fl),
            make_tf(self.fr_parent, self.fr_child, self.theta_fr),
            make_tf(self.rl_parent, self.rl_child, self.theta_rl),
            make_tf(self.rr_parent, self.rr_child, self.theta_rr)
        ]
        self.tf_br.sendTransform(tfs)

def main(args=None):
    rclpy.init(args=args)
    node = WheelRotationTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
