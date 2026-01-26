import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Lấy đường dẫn gói & file config
    pkg_bringup = get_package_share_directory('mecanum_bringup')
    
    ekf_config_file = os.path.join(pkg_bringup, 'config', 'base', 'ekf_params.yaml')
    robot_config_file = os.path.join(pkg_bringup, 'config', 'base', 'robot_params.yaml')

    return LaunchDescription([
        # ---------------------------------------------------------
        # 0. STATIC TF (Rất quan trọng cho EKF)
        # ---------------------------------------------------------
        # Khai báo vị trí IMU so với tâm robot (Ví dụ: nằm giữa tâm)
        # args: x y z yaw pitch roll parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # ---------------------------------------------------------
        # 1. IMU DRIVER
        # ---------------------------------------------------------
        Node(
            package='imu_hwt911',
            executable='hwt911_node',
            name='imu_node',
            output='screen',
            # Nên load từ file YAML để đồng bộ tham số với toàn hệ thống
            parameters=[robot_config_file],
            # Remap topic để EKF nhận được (trong file yaml EKF thường nghe topic này)
            remappings=[('/imu_data', '/imu/data')]
        ),

        # ---------------------------------------------------------
        # 2. ROBOT DRIVER (Đọc Encoder)
        # ---------------------------------------------------------
        Node(
            package='mecanum_base',
            executable='robot_driver',
            name='robot_driver',
            output='screen',
            parameters=[
                robot_config_file,
                # QUAN TRỌNG: Tắt TF của driver cơ sở để không xung đột với EKF
                {'robot.odom.publish_tf': False} 
            ],
            # QUAN TRỌNG: Đổi tên topic output để làm đầu vào cho EKF
            remappings=[('/odom', '/wheel/odom')] 
        ),

        # ---------------------------------------------------------
        # 3. EKF - ROBOT LOCALIZATION
        # ---------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file],
            # Đầu ra của EKF là odom chuẩn nhất
            remappings=[('/odometry/filtered', '/odom')]
        ),

        # ---------------------------------------------------------
        # 4. HIỂN THỊ BÁNH XE (Visual)
        # ---------------------------------------------------------
        Node(
            package='mecanum_base',
            executable='wheel_rotation_tf',
            name='wheel_rotation_tf',
            output='screen',
            parameters=[robot_config_file]
        )
    ])