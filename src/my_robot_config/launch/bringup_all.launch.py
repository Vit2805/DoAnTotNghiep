import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_config')

    # ==================================================
    # 1. HỆ THỐNG LIDAR (Chạy ngay lập tức)
    # Bên trong file này đã có logic delay 5s cho Lidar sau rồi
    # ==================================================
    dual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'dual_lidar_with_merge.launch.py')
        )
    )

    # ==================================================
    # 2. DRIVER / ODOM (Delay 15s - TRÁNH SỤT ÁP)
    # Phải đợi 2 Lidar quay tít, dòng điện phẳng mới bật STM32
    # ==================================================
    robot_driver = TimerAction(
        period=15.0, 
        actions=[
            Node(
                package='my_robot_config',
                executable='robot_driver', 
                name='robot_driver',
                output='screen',
                # Thêm respawn để nếu lỡ STM32 rớt thì tự kết nối lại
                respawn=True, 
                respawn_delay=2.0
            )
        ]
    )

    # ==================================================
    # 3. SLAM TOOLBOX (Delay 20s - Đợi đủ "đồ chơi")
    # Lúc này đã có: Scan merged + Odom TF
    # ==================================================
    slam_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='slam_toolbox',
                # Đổi sang Async để chạy mượt hơn trên máy cấu hình thấp
                executable='async_slam_toolbox_node', 
                name='slam_toolbox',
                output='screen',
                remappings=[
                    ('scan', '/scan_merged'), 
                ],
                parameters=[{
                    'use_sim_time': False,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'map_frame': 'map',
                    'mode': 'mapping',
                    
                    # --- Cấu hình chi tiết ---
                    'max_laser_range': 12.0,
                    'resolution': 0.05,
                    'minimum_time_interval': 0.2, # Giảm xuống để cập nhật map nhanh hơn
                    'transform_timeout': 0.2,
                    'tf_buffer_duration': 30.,
                    'stack_size_to_use': 40000000,
                    'enable_interactive_mode': True,
                    
                    # --- Cấu hình Loop Closure (Chống lệch map khi đi xa) ---
                    'do_loop_closing': True,
                    'use_scan_matching': True,
                }]
            )
        ]
    )

    return LaunchDescription([
        dual_lidar_launch,
        robot_driver,
        slam_node,
    ])