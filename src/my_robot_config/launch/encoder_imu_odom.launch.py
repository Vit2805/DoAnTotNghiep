import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Lấy đường dẫn file config EKF
    config_dir = os.path.join(get_package_share_directory('my_robot_config'), 'config')
    ekf_config_file = os.path.join(config_dir, 'ekf_params.yaml')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            # Nếu IMU gắn lệch tâm robot, bạn sửa 3 số 0 đầu tiên (đơn vị mét)
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        # ---------------------------------------------------------
        # 1. IMU DRIVER (Python - Theo Doc)
        # ---------------------------------------------------------
        Node(
            package='imu_hwt911',
            executable='hwt911_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 57600,
                'frame_id': 'imu_link',
            }],
            remappings=[('/imu_data', '/imu/data')]
        ),

        # ---------------------------------------------------------
        # 2. ROBOT DRIVER (Mecanum - STM32)
        # ---------------------------------------------------------
        Node(
            package='my_robot_config',
            executable='robot_driver',
            name='robot_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
                'wheel_radius': 0.07,
                'track_width_x': 0.25,  # Lx
                'track_width_y': 0.25,  # Ly
                'ticks_per_rev': 6864.0,
            }]
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
            remappings=[('/odometry/filtered', '/odom')]
        ),

        # ---------------------------------------------------------
        # 4. HIỂN THỊ BÁNH XE (Theo chuẩn Doc)
        # ---------------------------------------------------------
        # 4.1. Static TF (Vị trí 4 bánh)
        Node(
            package='my_robot_config',
            executable='wheel_tf_node',
            name='wheel_tf_node',
            output='screen'
        ),
        
        # 4.2. Rotation TF (Quay bánh theo Odom)
        Node(
            package='my_robot_config',
            executable='wheel_rotation_tf',
            name='wheel_rotation_tf',
            output='screen',
            parameters=[{
                'wheel_radius': 0.07,
                'base_width': 0.50,
                'odom_encoder_topic': '/wheel/odom'
            }]
        )
    ])