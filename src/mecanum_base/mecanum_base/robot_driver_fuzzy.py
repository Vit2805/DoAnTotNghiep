#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
import serial
import math
import time

# Import bộ Fuzzy PID (Đảm bảo file fuzzy_pid.py đã có cùng thư mục)
from mecanum_base.fuzzy_pid import FuzzyPID

class RobotDriverFuzzy(Node):  # Đổi tên Class
    def __init__(self):
        super().__init__('robot_driver_fuzzy') # Đổi tên Node

        # --- 1. LOAD THAM SỐ ---
        self.declare_parameter('robot.geometry.R', 0.075)
        self.declare_parameter('robot.geometry.lx', 0.240)
        self.declare_parameter('robot.geometry.ly', 0.235)
        self.declare_parameter('robot.wheels.ticks_per_rev', 6864.0)
        self.declare_parameter('robot.comms.bridge.serial_port', '/dev/ttyACM0')
        self.declare_parameter('robot.comms.bridge.baud', 115200)
        self.declare_parameter('robot.odom.publish_tf', True) 
        self.declare_parameter('robot.frames.odom', 'odom')
        self.declare_parameter('robot.frames.base_link', 'base_link')

        self.R = self.get_parameter('robot.geometry.R').value
        self.Lx = self.get_parameter('robot.geometry.lx').value
        self.Ly = self.get_parameter('robot.geometry.ly').value
        self.L_sum = self.Lx + self.Ly
        self.TPR = self.get_parameter('robot.wheels.ticks_per_rev').value
        self.port = self.get_parameter('robot.comms.bridge.serial_port').value
        self.baud = self.get_parameter('robot.comms.bridge.baud').value
        self.should_pub_tf = self.get_parameter('robot.odom.publish_tf').value
        self.frame_odom = self.get_parameter('robot.frames.odom').value
        self.frame_base = self.get_parameter('robot.frames.base_link').value

        # --- 2. KHỞI TẠO FUZZY PID ---
        self.pid_vx = FuzzyPID(base_kp=1.2, base_ki=0.5, base_kd=0.01, max_out=1.0, error_map_max=0.5)
        self.pid_vy = FuzzyPID(base_kp=1.2, base_ki=0.5, base_kd=0.01, max_out=1.0, error_map_max=0.5)
        self.pid_wz = FuzzyPID(base_kp=1.5, base_ki=0.5, base_kd=0.02, max_out=2.0, error_map_max=1.0)

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_control_time = self.get_clock().now()

        # --- 3. KẾT NỐI ---
        self.ser = None
        self.connect_serial()
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_ticks = self.create_publisher(Int32MultiArray, '/wheel_ticks_debug', 10)
        
        if self.should_pub_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Biến Odom
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_ticks = [None]*4
        self.last_stamp = None
        self.MAX_TICK_JUMP = 50000 

        self.create_timer(0.02, self.update_loop) 
        self.get_logger().info(f"Fuzzy PID Driver Started!")

    def connect_serial(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial fail: {e}")

    def cmd_vel_callback(self, msg: Twist):
        # Lưu mục tiêu để PID xử lý sau
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    def update_loop(self):
        if not self.ser or not self.ser.is_open: return
        try:
            data_block = self.ser.read_all().decode('utf-8', errors='ignore')
            lines = data_block.split('\n')
            last_valid_line = None
            for line in reversed(lines):
                if "T=" in line:
                    last_valid_line = line.strip()
                    break 
            if last_valid_line:
                self.process_encoder_and_control(last_valid_line)
        except Exception: pass

    def process_encoder_and_control(self, line):
        try:
            # --- XỬ LÝ ENCODER ---
            parts = line.split()
            ticks = []
            for p in parts:
                if p.startswith("T="):
                    ticks = [int(x) for x in p.split('=')[1].split(',')]
                    break
            if len(ticks) < 4: return

            # Fix chiều quay
            ticks = [-t for t in ticks] 
            
            self.pub_ticks.publish(Int32MultiArray(data=ticks))

            if self.last_ticks[0] is None:
                self.last_ticks = ticks; self.last_stamp = self.get_clock().now(); return

            now = self.get_clock().now()
            dt = (now - self.last_stamp).nanoseconds / 1e9
            if dt < 0.001: dt = 0.001 

            d_ticks = [c - l for c, l in zip(ticks, self.last_ticks)]
            if any(abs(x) > self.MAX_TICK_JUMP for x in d_ticks):
                self.last_ticks = ticks; self.last_stamp = now; return

            self.last_ticks = ticks; self.last_stamp = now

            # Tính Odom
            m_per_tick = (2 * math.pi * self.R) / self.TPR
            d_meters = [d * m_per_tick for d in d_ticks]
            d_fl, d_fr, d_rr, d_rl = d_meters

            dx_r = (d_fl + d_fr + d_rr + d_rl) / 4.0
            dy_r = (-d_fl + d_fr - d_rr + d_rl) / 4.0 
            dth  = (-d_fl + d_fr + d_rr - d_rl) / (4.0 * self.L_sum)

            self.th += dth
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))
            self.x += dx_r * math.cos(self.th) - dy_r * math.sin(self.th)
            self.y += dx_r * math.sin(self.th) + dy_r * math.cos(self.th)

            current_vx = dx_r / dt
            current_vy = dy_r / dt
            current_wz = dth / dt

            self.publish_odom(current_vx, current_vy, current_wz, now)

            # --- CHẠY FUZZY PID ---
            dt_pid = (now - self.last_control_time).nanoseconds / 1e9
            self.last_control_time = now

            if self.target_vx == 0 and self.target_vy == 0 and self.target_wz == 0:
                if abs(current_vx) < 0.01:
                    self.pid_vx.reset(); self.pid_vy.reset(); self.pid_wz.reset()

            out_vx = self.pid_vx.compute(self.target_vx, current_vx, dt_pid)
            out_vy = self.pid_vy.compute(self.target_vy, current_vy, dt_pid)
            out_wz = self.pid_wz.compute(self.target_wz, current_wz, dt_pid)

            cmd_str = f"V {out_vx:.3f} {out_vy:.3f} {out_wz:.3f}\n"
            self.ser.write(cmd_str.encode('utf-8'))

        except ValueError: pass

    def publish_odom(self, vx, vy, wz, now_time):
        # (Giữ nguyên logic publish cũ)
        qz = math.sin(self.th / 2.0); qw = math.cos(self.th / 2.0)
        if self.should_pub_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now_time.to_msg(); t.header.frame_id = self.frame_odom; t.child_frame_id = self.frame_base
            t.transform.translation.x = self.x; t.transform.translation.y = self.y
            t.transform.rotation.z = qz; t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        msg = Odometry()
        msg.header.stamp = now_time.to_msg(); msg.header.frame_id = self.frame_odom; msg.child_frame_id = self.frame_base
        msg.pose.pose.position.x = self.x; msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = qz; msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx; msg.twist.twist.linear.y = vy; msg.twist.twist.angular.z = wz
        self.pub_odom.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverFuzzy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()