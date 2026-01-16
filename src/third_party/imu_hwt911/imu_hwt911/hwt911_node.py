#!/usr/bin/env python3
import math
import struct
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

G = 9.80665

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]

class HWT911(Node):
    def __init__(self):
        super().__init__('hwt911')
        # Params: Sửa mặc định thành 57600 cho khớp với sensor của bạn
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 57600).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value

        # Publisher
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(Imu, 'imu/data', qos)

        # Serial Connection
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.02)
            self.get_logger().info(f"Opened {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Cannot open serial: {e}")
            return

        self.buf = bytearray()
        
        # States
        self.acc = None
        self.gyro = None
        self.quat = None

        # Timer 100Hz
        self.create_timer(0.01, self.spin_serial)

    def spin_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
                self.parse()
        except Exception as e:
            pass # Bỏ qua lỗi để không crash (như doc)

    def parse(self):
        while len(self.buf) >= 11:
            if self.buf[0] != 0x55:
                self.buf.pop(0)
                continue
            
            pkt = self.buf[:11]
            if ((sum(pkt[:10]) & 0xFF) != pkt[10]):
                self.buf.pop(0)
                continue

            pid = pkt[1]
            pay = pkt[2:10]

            if pid == 0x51: # Acc
                ax, ay, az, _ = struct.unpack('<hhhh', pay)
                self.acc = (float(ax)/32768.0*16.0*G, float(ay)/32768.0*16.0*G, float(az)/32768.0*16.0*G)
            elif pid == 0x52: # Gyro
                gx, gy, gz, _ = struct.unpack('<hhhh', pay)
                d2r = math.pi/180.0
                self.gyro = (float(gx)/32768.0*2000.0*d2r, float(gy)/32768.0*2000.0*d2r, float(gz)/32768.0*2000.0*d2r)
            elif pid == 0x53: # Angle
                r, p, y, _ = struct.unpack('<hhhh', pay)
                d2r = math.pi/180.0
                self.quat = euler_to_quat((float(r)/32768.0)*180*d2r, (float(p)/32768.0)*180*d2r, (float(y)/32768.0)*180*d2r)

            if (self.acc and self.gyro):
                if self.quat is None: self.quat = [1.0, 0.0, 0.0, 0.0]
                self.publish_imu()
            
            del self.buf[:11]

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.orientation.w = float(self.quat[0])
        msg.orientation.x = float(self.quat[1])
        msg.orientation.y = float(self.quat[2])
        msg.orientation.z = float(self.quat[3])
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        msg.angular_velocity.x = float(self.gyro[0])
        msg.angular_velocity.y = float(self.gyro[1])
        msg.angular_velocity.z = float(self.gyro[2])
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        msg.linear_acceleration.x = float(self.acc[0])
        msg.linear_acceleration.y = float(self.acc[1])
        msg.linear_acceleration.z = float(self.acc[2])
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = HWT911()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
