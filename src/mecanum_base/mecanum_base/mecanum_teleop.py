#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- CẤU HÌNH ---
# Tốc độ tối đa
MAX_SPEED = 0.5   # m/s
MAX_TURN  = 1.5   # rad/s

# ĐỘ MƯỢT (Gia tốc) - Số càng nhỏ thì càng mượt nhưng càng trễ
# 0.01 = Rất mượt (như lái tàu)
# 0.1  = Bốc (như xe đua)
ACCEL_STEP = 0.05 

msg = """
ĐIỀU KHIỂN ROBOT MECANUM (CÓ GIA TỐC)
---------------------------
        W    
   A    S    D
   
   Q (Xoay) E
   
SPACE: Phanh từ từ
CTRL-C: Thoát
"""

MOVE_BINDINGS = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'q': (0, 0, 0, 1),
    'e': (0, 0, 0, -1),
    ' ': (0, 0, 0, 0),

    'u': (1, 1, 0, 0),   # Tiến + Trái (Forward-Left)
    'o': (1, -1, 0, 0),  # Tiến + Phải (Forward-Right)
    'j': (-1, 1, 0, 0),  # Lùi + Trái (Back-Left)
    'l': (-1, -1, 0, 0), # Lùi + Phải (Back-Right)
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # Timeout 0.1s giúp vòng lặp chạy liên tục để tính toán gia tốc
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) 
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Hàm tạo dốc (Ramp Function)
def ramp_vel(current, target, step):
    if target > current:
        return min(target, current + step)
    elif target < current:
        return max(target, current - step)
    else:
        return target

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = rclpy.create_node('mecanum_teleop_smooth')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print(msg)
    
    # Biến lưu trạng thái mục tiêu (Người dùng muốn đi đâu)
    target_x = 0.0
    target_y = 0.0
    target_th = 0.0

    # Biến lưu trạng thái hiện tại (Robot đang đi thế nào)
    current_x = 0.0
    current_y = 0.0
    current_th = 0.0

    try:
        while True:
            key = getKey(settings)
            
            # 1. ĐỌC LỆNH TỪ BÀN PHÍM (Xác định mục tiêu)
            if key in MOVE_BINDINGS.keys():
                target_x = MOVE_BINDINGS[key][0] * MAX_SPEED
                target_y = MOVE_BINDINGS[key][1] * MAX_SPEED
                target_th = MOVE_BINDINGS[key][3] * MAX_TURN
            
            elif key == '\x03': # Ctrl-C
                break
            
            else:
                # Nếu nhả phím -> Mục tiêu về 0 (để giảm tốc từ từ)
                target_x = 0.0
                target_y = 0.0
                target_th = 0.0

            # 2. TÍNH TOÁN GIA TỐC (Làm mượt)
            # Thay vì gán thẳng, ta nhích dần current về phía target
            current_x = ramp_vel(current_x, target_x, ACCEL_STEP)
            current_y = ramp_vel(current_y, target_y, ACCEL_STEP)
            current_th = ramp_vel(current_th, target_th, ACCEL_STEP)

            # 3. GỬI LỆNH ĐI
            twist = Twist()
            twist.linear.x = current_x
            twist.linear.y = current_y
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = current_th
            pub.publish(twist)

            # In ra trạng thái để debug
            # \r để ghi đè dòng cũ, tránh spam terminal
            print(f"\rSpeed: X={current_x:.2f}, Y={current_y:.2f}, W={current_th:.2f}   ", end="")

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()