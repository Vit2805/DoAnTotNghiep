#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster # Thêm thư viện TF
import serial
import math
import time

class RobotDriverMecanum(Node):
    def __init__(self):
        super().__init__('robot_driver_mecanum')

        # =========================================================
        # 1. LOAD THAM SỐ TỪ YAML (Cấu trúc robot.xxx)
        # =========================================================
        
        # Nhóm Geometry (Kích thước vật lý)
        self.declare_parameter('robot.geometry.R', 0.075)
        self.declare_parameter('robot.geometry.lx', 0.240)
        self.declare_parameter('robot.geometry.ly', 0.235)
        self.declare_parameter('robot.wheels.ticks_per_rev', 6864.0)
        
        # Nhóm Comms (Kết nối)
        self.declare_parameter('robot.comms.bridge.serial_port', '/dev/ttyACM0')
        self.declare_parameter('robot.comms.bridge.baud', 115200)

        # Nhóm Odom (Cấu hình EKF Logic)
        self.declare_parameter('robot.odom.publish_tf', True) 
        self.declare_parameter('robot.frames.odom', 'odom')
        self.declare_parameter('robot.frames.base_link', 'base_link')

        # --- LẤY GIÁ TRỊ RA BIẾN ---
        self.R = self.get_parameter('robot.geometry.R').value
        self.Lx = self.get_parameter('robot.geometry.lx').value
        self.Ly = self.get_parameter('robot.geometry.ly').value
        self.L_sum = self.Lx + self.Ly
        self.TPR = self.get_parameter('robot.wheels.ticks_per_rev').value
        
        self.port = self.get_parameter('robot.comms.bridge.serial_port').value
        self.baud = self.get_parameter('robot.comms.bridge.baud').value
        
        # Logic EKF: Có publish TF hay không?
        self.should_pub_tf = self.get_parameter('robot.odom.publish_tf').value
        self.frame_odom = self.get_parameter('robot.frames.odom').value
        self.frame_base = self.get_parameter('robot.frames.base_link').value

        # =========================================================
        # 2. KHỞI TẠO KẾT NỐI & PUBLISHERS
        # =========================================================
        self.ser = None
        self.connect_serial()

        # Publisher Odom
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        
        # Publisher Debug Ticks
        self.pub_ticks = self.create_publisher(Int32MultiArray, '/wheel_ticks_debug', 10)
        
        # TF Broadcaster (Chỉ khởi tạo nếu cần phát TF)
        if self.should_pub_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info(f"TF Broadcasting ENABLED (Mode: No EKF)")
        else:
            self.tf_broadcaster = None
            self.get_logger().warn(f"TF Broadcasting DISABLED (Mode: EKF Active)")

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- BIẾN ODOMETRY ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_ticks = [None, None, None, None]
        self.last_stamp = None
        self.MAX_TICK_JUMP = 50000 

        self.create_timer(0.02, self.update_loop) # 50Hz
        self.get_logger().info(f"Mecanum Driver Started. Port: {self.port}")

    def connect_serial(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")

    def update_loop(self):
        if not self.ser or not self.ser.is_open: return
        try:
            # Đọc sạch buffer để chống lag (Kỹ thuật Anti-Lag)
            data_block = self.ser.read_all().decode('utf-8', errors='ignore')
            lines = data_block.split('\n')
            
            # Tìm dòng hợp lệ mới nhất
            last_valid_line = None
            for line in reversed(lines):
                if "T=" in line:
                    last_valid_line = line.strip()
                    break 
            
            if last_valid_line:
                self.process_encoder(last_valid_line)
                
        except Exception: pass

    def process_encoder(self, line):
        try:
            # Parse dữ liệu: T=100,200,300,400
            parts = line.split()
            ticks = []
            for p in parts:
                if p.startswith("T="):
                    val_str = p.split('=')[1]
                    ticks = [int(x) for x in val_str.split(',')]
                    break
            
            if len(ticks) < 4: return

            # --- FIX CHIỀU QUAY (Tùy chỉnh theo thực tế robot) ---
            # Nếu robot đi ngược, thêm dấu trừ vào đây
            ticks[0] = -ticks[0] 
            ticks[1] = -ticks[1]
            ticks[2] = -ticks[2]
            ticks[3] = -ticks[3]
            
            self.pub_ticks.publish(Int32MultiArray(data=ticks))

            # Khởi tạo lần đầu
            if self.last_ticks[0] is None:
                self.last_ticks = ticks
                self.last_stamp = self.get_clock().now()
                return

            # Tính dt
            now = self.get_clock().now()
            dt = (now - self.last_stamp).nanoseconds / 1e9
            if dt < 0.001: dt = 0.001 

            # Tính delta ticks
            d_ticks = [c - l for c, l in zip(ticks, self.last_ticks)]
            
            # Lọc nhiễu đột biến
            if any(abs(x) > self.MAX_TICK_JUMP for x in d_ticks):
                self.last_ticks = ticks
                self.last_stamp = now
                return

            self.last_ticks = ticks
            self.last_stamp = now

            # 1. Đổi ticks -> mét
            m_per_tick = (2 * math.pi * self.R) / self.TPR
            d_meters = [d * m_per_tick for d in d_ticks]
            d_fl, d_fr, d_rr, d_rl = d_meters

            # 2. Mecanum Kinematics
            dx_r = (d_fl + d_fr + d_rr + d_rl) / 4.0
            dy_r = (-d_fl + d_fr - d_rr + d_rl) / 4.0 
            dth  = (-d_fl + d_fr + d_rr - d_rl) / (4.0 * self.L_sum)

            # 3. Tích phân vị trí (Odom)
            self.th += dth
            # Chuẩn hóa góc -pi đến pi
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))

            self.x += dx_r * math.cos(self.th) - dy_r * math.sin(self.th)
            self.y += dx_r * math.sin(self.th) + dy_r * math.cos(self.th)

            # 4. Vận tốc
            vx = dx_r / dt
            vy = dy_r / dt
            wz = dth / dt

            self.publish_odom(vx, vy, wz, now)

        except ValueError: pass

    def publish_odom(self, vx, vy, wz, now_time):
        # 1. Chuẩn bị Quaternion từ góc Theta
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # 2. Gửi TF (Nếu được phép)
        if self.should_pub_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now_time.to_msg()
            t.header.frame_id = self.frame_odom
            t.child_frame_id = self.frame_base
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            
            self.tf_broadcaster.sendTransform(t)

        # 3. Gửi Odom Message
        msg = Odometry()
        msg.header.stamp = now_time.to_msg()
        msg.header.frame_id = self.frame_odom
        msg.child_frame_id = self.frame_base
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        
        # Covariance (Độ tin cậy) - Mecanum trượt ngang nên Y cov cao hơn chút
        cov = [0.01 if i in [0, 7, 14, 21, 28, 35] else 0.0 for i in range(36)]
        msg.pose.covariance = cov
        msg.twist.covariance = cov
        
        self.pub_odom.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        if not self.ser or not self.ser.is_open: return
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Gửi lệnh xuống STM32: "V vx vy wz"
        cmd_str = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverMecanum()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()