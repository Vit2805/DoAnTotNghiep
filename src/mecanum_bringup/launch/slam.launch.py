import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. KHAI BÁO GÓI BRINGUP
    pkg_bringup = get_package_share_directory('mecanum_bringup')

    # ======================================================
    # 2. KHỞI ĐỘNG PHẦN CỨNG (Lidar, Motor, Merger...)
    # ======================================================
    # Thay vì gọi file dual_lidar cũ (đã xóa), ta gọi bringup_all
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'bringup_all.launch.py')
        )
    )

    # ======================================================
    # 3. SLAM TOOLBOX (Mapping)
    # ======================================================
    slam_params = os.path.join(pkg_bringup, 'config', 'slam',  'slam_toolbox_merged.yaml')

    slam_node = TimerAction(
        period=15.0,  # [QUAN TRỌNG] Đợi Merger trong bringup_all khởi động xong
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node', # Dùng async mượt hơn sync cho robot thật
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {'use_sim_time': False}
                ],
                remappings=[
                    # Topic đầu vào của SLAM -> Topic gộp của bạn
                    # Hãy chắc chắn topic này khớp với output của Merger
                    ('/scan', '/scan_360'), 
                ]
            )
        ]
    )

    # ======================================================
    # 4. RVIZ (Tùy chọn - Để xem bản đồ ngay lập tức)
    # ======================================================
    # rviz_node = Node(
    #     package='rviz2', executable='rviz2', name='rviz2',
    #     arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'slam.rviz')]
    # )

    return LaunchDescription([
        hardware_launch,
        slam_node,
        # rviz_node
    ])