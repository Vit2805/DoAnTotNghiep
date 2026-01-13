import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config_pkg_share = get_package_share_directory('my_robot_config')

    # ==================================================
    # 1. HỆ THỐNG LIDAR (T = 0s)
    # ==================================================
    dual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_share, 'launch', 'dual_lidar_with_merge.launch.py')
        )
    )

    # ==================================================
    # 2. KHỞI ĐỘNG DRIVER & ODOM (T = 15s)
    # Ở đây ta làm 2 việc cùng lúc:
    # A. Bật Node 'robot_driver' (để kết nối STM32)
    # B. Bật File 'encoder_imu_odom.launch.py' (để tính toán vị trí)
    # ==================================================
    
    # Việc A: Node Driver (Phần cứng)
    start_hardware_driver = Node(
        package='my_robot_config', 
        executable='robot_driver', # Tên file thực thi node driver
        name='robot_driver',
        output='screen',
        respawn=True, 
        parameters=[{'use_sim_time': False}]
    )

    # Việc B: File Launch tính Odom (Phần mềm)
    start_odom_logic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_share, 'launch', 'encoder_imu_odom.launch.py')
        )
    )

    # Gom cả 2 việc vào chung 1 nhóm delay 15s
    driver_and_odom_group = TimerAction(
        period=15.0, 
        actions=[
            start_hardware_driver, # Chạy Driver
            start_odom_logic       # Chạy Odom calculation
        ]
    )

    # ==================================================
    # 3. SLAM TOOLBOX (T = 30s)
    # Đợi 15s sau khi nhóm Driver+Odom chạy
    # ==================================================
    slam_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node', 
                name='slam_toolbox',
                output='screen',
                remappings=[('scan', '/scan_merged')],
                parameters=[{
                    'use_sim_time': False,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'map_frame': 'map',
                    'mode': 'mapping',
                    'transform_timeout': 1.0, 
                    'tf_buffer_duration': 40.0,
                    'message_filter_queue_size': 20,
                    # ... các param khác ...
                }]
            )
        ]
    )

    return LaunchDescription([
        dual_lidar_launch,
        driver_and_odom_group, # <--- Giờ nó chạy cả 2 thứ bạn cần
        slam_node,
    ])
