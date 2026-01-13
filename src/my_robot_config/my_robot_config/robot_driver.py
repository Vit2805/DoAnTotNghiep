#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import serial
import math
import time

class RobotDriverMecanum(Node):
    def __init__(self):
        super().__init__('robot_driver_mecanum')

        # --- CẤU HÌNH THAM SỐ MECANUM ---
        self.port = self.declare_parameter('port', '/dev/stm32_base').value
        self.baud = self.declare_parameter('baud', 115200).value
        
        self.R = self.declare_parameter('wheel_radius', 0.075).value
        self.Lx = self.declare_parameter('track_width_x', 0.24).value 
        self.Ly = self.declare_parameter('track_width_y', 0.24).value 
        self.L_sum = self.Lx + self.Ly
        
        self.TPR = self.declare_parameter('ticks_per_rev', 6864.0).value

        # --- KẾT NỐI SERIAL ---
        self.ser = None
        self.connect_serial()

        # --- PUBLISHER & SUBSCRIBER ---
        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom', 20)
        #self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_ticks = self.create_publisher(Int32MultiArray, '/wheel_ticks_debug', 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- BIẾN ODOMETRY ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_ticks = [None, None, None, None]
        self.last_stamp = None
        self.MAX_TICK_JUMP = 50000 

        self.create_timer(0.02, self.update_loop)
        self.get_logger().info("MECANUM DRIVER FIXED: NaN Protection Active.")

    def connect_serial(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial fail: {e}")

    def update_loop(self):
        if not self.ser or not self.ser.is_open: return
        try:
            # --- CÁCH CŨ (DỄ GÂY LAG) ---
            # if self.ser.in_waiting:
            #     line = self.ser.readline()... 
            
            # --- CÁCH MỚI (CHỐNG LAG TUYỆT ĐỐI) ---
            # 1. Đọc sạch sành sanh dữ liệu đang có trong buffer
            data_block = self.ser.read_all().decode('utf-8', errors='ignore')
            
            # 2. Tách thành các dòng
            lines = data_block.split('\n')
            
            # 3. Tìm dòng hợp lệ CUỐI CÙNG (Mới nhất)
            last_valid_line = None
            
            # Duyệt ngược từ dưới lên cho nhanh
            for line in reversed(lines):
                if "T=" in line: # Kiểm tra điều kiện gói tin chuẩn
                    last_valid_line = line.strip()
                    break # Tìm thấy gói mới nhất rồi thì dừng ngay
            
            # 4. Chỉ xử lý gói mới nhất
            if last_valid_line:
                self.process_encoder(last_valid_line)
                
        except Exception: pass

    def process_encoder(self, line):
        try:
            parts = line.split()
            ticks = []
            for p in parts:
                if p.startswith("T="):
                    val_str = p.split('=')[1]
                    ticks = [int(x) for x in val_str.split(',')]
                    break
            
            if len(ticks) < 4: return

            # --- FIX LỖI NGƯỢC CHIỀU (Bánh 1, 2, 3, 4) ---
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

            # --- TÍNH TOÁN THỜI GIAN (DT) AN TOÀN ---
            now = self.get_clock().now()
            dt = (now - self.last_stamp).nanoseconds / 1e9
            
            # [QUAN TRỌNG] Chặn lỗi chia cho 0 gây ra NaN
            if dt < 0.001: 
                dt = 0.001 
            # ----------------------------------------

            d_ticks = [c - l for c, l in zip(ticks, self.last_ticks)]
            
            # Kiểm tra nhiễu (nhảy số quá lớn)
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

            # 2. CÔNG THỨC KINEMATICS
            dx_r = (d_fl + d_fr + d_rr + d_rl) / 4.0
            dy_r = (-d_fl + d_fr - d_rr + d_rl) / 4.0 
            dth  = (-d_fl + d_fr + d_rr - d_rl) / (4.0 * self.L_sum)

            # 3. Tích phân vị trí (Odometry)
            self.th += dth
            # Chuẩn hóa góc về -pi đến pi
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))

            self.x += dx_r * math.cos(self.th) - dy_r * math.sin(self.th)
            self.y += dx_r * math.sin(self.th) + dy_r * math.cos(self.th)

            # 4. Tính vận tốc tức thời
            vx = dx_r / dt
            vy = dy_r / dt
            wz = dth / dt

            self.publish_odom(vx, vy, wz)

        except ValueError: pass

    def publish_odom(self, vx, vy, wz):
        msg = Odometry()
        # [QUAN TRỌNG] Lấy thời gian mới nhất để RViz không báo lỗi trễ
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.th/2)
        msg.pose.pose.orientation.w = math.cos(self.th/2)
        
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        
        # --- SỬA LẠI PHẦN NÀY CHO CHUẨN ---
        # Chỉ điền số vào đường chéo chính (Diagonal), còn lại bằng 0
        cov = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # y (Mecanum đi ngang được nên cần độ tin cậy)
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # yaw
        ]
        
        msg.pose.covariance = cov
        msg.twist.covariance = cov
        # ----------------------------------
        
        self.pub_odom.publish(msg)
    def cmd_vel_callback(self, msg: Twist):
        if not self.ser or not self.ser.is_open: return
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Gửi lệnh dạng: "V 0.3 0.0 0.5" (Dùng dấu cách)
        cmd_str = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"

        # self.get_logger().info(f"Sent: {cmd_str.strip()}")

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
