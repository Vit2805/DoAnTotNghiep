import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # =========================================================
    # 1. KHAI BÁO CÁC "CÔNG TẮC" (ARGUMENTS)
    # =========================================================
    pkg_bringup = get_package_share_directory('mecanum_bringup')
    
    # Argument 1: Có dùng EKF không? (Mặc định: True cho Mecanum)
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Bật bộ lọc Kalman (True) hay chạy Odom thô (False)?'
    )
    
    # Argument 2: Có bật Lidar không? (Tiện khi chỉ muốn test bánh xe)
    start_lidar_arg = DeclareLaunchArgument(
        'start_lidar', default_value='true',
        description='Bật cảm biến Lidar?'
    )

    # Lấy giá trị từ Argument ra biến để dùng
    use_ekf = LaunchConfiguration('use_ekf')
    start_lidar = LaunchConfiguration('start_lidar')
    
    # File cấu hình (Đã trỏ đúng vào thư mục base)
    robot_config = os.path.join(pkg_bringup, 'config', 'base', 'robot_params.yaml')
    ekf_config = os.path.join(pkg_bringup, 'config', 'base',  'ekf_params.yaml')

    # =========================================================
    # 2. STATIC TF (LUÔN BẬT)
    # =========================================================
    static_tf_nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_front',
             arguments=['0.42', '0.29', '0.05', '3.14159', '0', '0', 'base_link', 'lidar_front']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_rear',
             arguments=['-0.42', '-0.29', '0.05', '0', '0', '0', 'base_link', 'lidar_rear']),
    ]

    # =========================================================
    # 3. LIDAR GROUP (KHỞI ĐỘNG TUẦN TỰ)
    # =========================================================
    
    # 3a. Lidar Trước: Chạy NGAY LẬP TỨC (T=0s)
    lidar_front_node = Node(
        package='sllidar_ros2', executable='sllidar_node',
        name='sllidar_front', output='screen', respawn=True,
        parameters=[robot_config], remappings=[('scan', 'scan_front')],
        condition=IfCondition(start_lidar)
    )

    # 3b. Lidar Sau: Delay 2.0s để tránh sụt nguồn (T=2s)
    lidar_rear_node = TimerAction(
        period=5.0, # [QUAN TRỌNG] Delay 2 giây
        actions=[
            Node(
                package='sllidar_ros2', executable='sllidar_node',
                name='sllidar_rear', output='screen', respawn=True,
                parameters=[robot_config], remappings=[('scan', 'scan_rear')],
                condition=IfCondition(start_lidar)
            )
        ]
    )

    # =========================================================
    # 4. ROBOT DRIVER (LOGIC CHUYỂN ĐỔI EKF THÔNG MINH)
    # Delay 5s (Tổng cộng) để đợi cả 2 Lidar ổn định
    # =========================================================
    
    # TRƯỜNG HỢP A: KHÔNG DÙNG EKF (use_ekf = False)
    driver_no_ekf = Node(
        package='mecanum_base', executable='robot_driver',
        name='robot_driver_direct', output='screen', respawn=True,
        parameters=[robot_config, 
                   {'robot.odom.publish_tf': True}], 
        condition=UnlessCondition(use_ekf)
    )

    # TRƯỜNG HỢP B: CÓ DÙNG EKF (use_ekf = True)
    driver_with_ekf = Node(
        package='mecanum_base', executable='robot_driver',
        name='robot_driver_raw', output='screen', respawn=True,
        parameters=[robot_config, 
                   {'robot.odom.publish_tf': False}],
        remappings=[('/odom', '/wheel/odom')],       
        condition=IfCondition(use_ekf)
    )

    # IMU luôn cần thiết
    imu_node = Node(
        package='imu_hwt911', executable='hwt911_node',
        name='imu_node', output='screen',
        parameters=[robot_config], remappings=[('/imu_data', '/imu/data')]
    )

    # Gom nhóm phần cứng (Chạy sau 5s - Lúc này Lidar đã quay tít)
    hardware_group = TimerAction(
        period=5.0,
        actions=[driver_no_ekf, driver_with_ekf, imu_node]
    )

    # =========================================================
    # 5. THUẬT TOÁN (MERGER & EKF) - DELAY 8s
    # =========================================================
    
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')],
        condition=IfCondition(use_ekf)
    )

    merger_node = Node(
        package='mecanum_base', executable='dual_lidar_merger',
        name='scan_merger_360', output='screen',
        parameters=[robot_config],
        condition=IfCondition(start_lidar)
    )
    
    wheel_viz = [
        Node(package='mecanum_base', executable='wheel_rotation_tf', name='wheel_rot', parameters=[robot_config])
    ]

    algo_group = TimerAction(
        period=8.0,
        actions=[ekf_node, merger_node] + wheel_viz
    )

    # =========================================================
    # TRẢ VỀ
    # =========================================================
    return LaunchDescription([
        use_ekf_arg,
        start_lidar_arg,
        *static_tf_nodes,
        lidar_front_node,   # Chạy ngay (0s)
        lidar_rear_node,    # Chạy trễ (2s)
        hardware_group,     # Chạy sau (5s)
        algo_group          # Chạy cuối (8s)
    ])