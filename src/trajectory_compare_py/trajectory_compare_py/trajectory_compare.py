import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import math
import time

class TrajectoryCompare(Node):
    def __init__(self):
        super().__init__('trajectory_compare')

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.x_real, self.y_real = [], []
        self.x_cmd, self.y_cmd = [], []

        self.xr = self.yr = self.thr = 0.0
        self.xc = self.yc = self.thc = 0.0

        self.last_time = time.time()
        self.v = 0.0
        self.w = 0.0

        self.timer = self.create_timer(0.1, self.update_cmd_traj)

    def odom_cb(self, msg):
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.x_real.append(self.xr)
        self.y_real.append(self.yr)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_cmd_traj(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.thc += self.w * dt
        self.xc += self.v * math.cos(self.thc) * dt
        self.yc += self.v * math.sin(self.thc) * dt

        self.x_cmd.append(self.xc)
        self.y_cmd.append(self.yc)

    def plot(self):
        plt.figure()
        plt.plot(self.x_real, self.y_real, 'b', label='Real (odom)')
        plt.plot(self.x_cmd, self.y_cmd, 'r--', label='Commanded')
        plt.axis('equal')
        plt.legend()
        plt.grid()
        plt.show()

def main():
    rclpy.init()
    node = TrajectoryCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.plot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
