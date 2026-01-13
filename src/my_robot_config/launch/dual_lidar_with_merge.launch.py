import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # =========================================================
    # 1. LIDAR FRONT (Có thêm respawn=True)
    # =========================================================
    lidar_front = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_front',
        output='screen',
        # --- CẤU HÌNH QUAN TRỌNG: TỰ KHỞI ĐỘNG LẠI ---
        respawn=True,              # Nếu chết (do sụt nguồn), tự sống lại
        respawn_delay=2.0,         # Chờ 2s rồi mới thử lại
        # ---------------------------------------------
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser_front',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', 'scan_front')]
    )

    # =========================================================
    # 2. LIDAR REAR (Delay 8s - Để Lidar trước quay tít hẳn đã)
    # =========================================================
    lidar_rear = TimerAction(
        period=8.0,  # <--- Tăng hẳn lên 8 giây để nguồn điện hồi phục
        actions=[
            Node(
                package='sllidar_ros2',     
                executable='sllidar_node',   
                name='sllidar_rear',
                output='screen',
                respawn=True,       # Cũng tự khởi động lại nếu lỗi
                respawn_delay=2.0,
                parameters=[{
                    'serial_port': '/dev/ttyUSB2', 
                    'serial_baudrate': 115200,
                    'frame_id': 'laser_rear',
                    'angle_compensate': True,
                    'scan_mode': 'Standard',
                }],
                remappings=[('scan', 'scan_rear')]
            )
        ]
    )

    # =========================================================
    # 3. TF Front
    # =========================================================
    tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_front',
        arguments=['0.43', '0.29', '0.05', '3.14159', '0', '0', 'base_link', 'laser_front']
    )

    # =========================================================
    # 4. TF Rear
    # =========================================================
    tf_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_rear',
        arguments=['-0.43', '-0.29', '0.05', '0', '0', '0', 'base_link', 'laser_rear']
    )

    # =========================================================
    # 5. MERGER (Cập nhật tham số hình học cho khớp Code Python)
    # =========================================================
    merger = TimerAction(
        period=12.0, 
        actions=[
            Node(
                package='dual_lidar_tools', # Tên package chứa file python của bạn
                executable='dual_lidar_merger', # Tên file thực thi (setup trong setup.py)
                name='lidar_merger',
                output='screen',
                parameters=[{
                    'front_topic': '/scan_front',
                    'rear_topic': '/scan_rear',
                    'output_topic': '/scan_merged',
                    'output_frame': 'base_link',
                    
                    # --- THÔNG SỐ VỊ TRÍ (PHẢI KHỚP VỚI TF static_transform_publisher) ---
                    'front_tx': 0.43,     # Khớp với --x 0.43
                    'front_ty': 0.29,     # Khớp với --y 0.29
                    'front_yaw': 3.14159, # Khớp với --yaw 3.14159 (Lidar trước quay ngược?)
                    
                    'rear_tx': -0.43,     # Khớp với --x -0.43
                    'rear_ty': -0.29,     # Khớp với --y -0.29
                    'rear_yaw': 0.0,      # Khớp với --yaw 0
                }],
            )
        ]
    )

    return LaunchDescription([
        lidar_front,
        lidar_rear,
        tf_front,
        tf_rear,
        merger,
    ])