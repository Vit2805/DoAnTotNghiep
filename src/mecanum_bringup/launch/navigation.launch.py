import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. KHAI BÁO CÁC BIẾN CẤU HÌNH (LaunchConfiguration)
    # Đây là các biến sẽ nhận giá trị từ bên ngoài hoặc dùng mặc định
    map_yaml_config = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # 2. ĐỊNH NGHĨA CÁC ĐƯỜNG DẪN MẶC ĐỊNH (Dựa trên cấu trúc Tree bạn gửi)
    # Lấy đường dẫn đến gói mecanum_bringup
    pkg_mecanum_bringup = FindPackageShare('mecanum_bringup')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # Đường dẫn file Map: mecanum_bringup/maps/my_map.yaml
    default_map_path = PathJoinSubstitution([
        pkg_mecanum_bringup,
        'maps',
        'my_map.yaml'
    ])

    # Đường dẫn file Params: mecanum_bringup/config/nav/nav2_params.yaml
    default_params_path = PathJoinSubstitution([
        pkg_mecanum_bringup,
        'config',
        'nav',
        'nav2_params.yaml'
    ])

    # Đường dẫn file Bringup phần cứng: mecanum_bringup/launch/bringup_all.launch.py
    bringup_all_launch_path = PathJoinSubstitution([
        pkg_mecanum_bringup,
        'launch',
        'bringup_all.launch.py'
    ])

    # 3. KHAI BÁO CÁC THAM SỐ (ARGUMENTS)
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Đường dẫn đầy đủ đến file map (.yaml)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Đường dẫn đầy đủ đến file tham số Nav2'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Sử dụng thời gian mô phỏng (False cho robot thật)'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Tự động khởi động Nav2 sau khi load xong'
    )

    # 4. INCLUDE CÁC FILE LAUNCH KHÁC

    # A. Khởi động phần cứng (Lidar, Driver, EKF, TF...)
    # Gọi file bringup_all.launch.py mà chúng ta đã test ok
    base_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_all_launch_path)
    )

    # B. Khởi động Navigation2 (AMCL, Planner, Controller...)
    # Sử dụng file launch chuẩn của nav2_bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_yaml_config,
            'params_file': params_file_config,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )

    # 5. TRẢ VỀ LAUNCH DESCRIPTION
    return LaunchDescription([
        # Khai báo tham số trước
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        
        # Chạy các tiến trình
        base_bringup_cmd,   # Bật phần cứng robot
        nav2_bringup_cmd    # Bật não dẫn đường
    ])