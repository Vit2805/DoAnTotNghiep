import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # =========================================================
    # 1. KHAI BÁO ĐƯỜNG DẪN GÓI & FILE CẤU HÌNH MẶC ĐỊNH
    # =========================================================
    pkg_bringup = get_package_share_directory('mecanum_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # File cấu hình Robot (Driver, Lidar, Merger, IMU)
    default_robot_config = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'), 'config', 'base', 'robot_params.yaml'
    ])
    
    # File cấu hình EKF
    default_ekf_config = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'), 'config', 'base', 'ekf_params.yaml'
    ])

    # File cấu hình Nav2 (Dùng bản Fuzzy tối ưu)
    default_nav2_config = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'), 'config', 'nav', 'nav2_params_fuzzy.yaml'
    ])
    
    # File Map mặc định
    default_map = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'), 'maps', 'my_map.yaml'
    ])

    # =========================================================
    # 2. KHAI BÁO CÁC LAUNCH ARGUMENTS (CÔNG TẮC)
    # =========================================================
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        
        # Nhóm Hardware
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start Lidar nodes?'),
        DeclareLaunchArgument('start_imu', default_value='true', description='Start IMU node?'),
        
        # Nhóm Localization
        DeclareLaunchArgument('use_ekf', default_value='true', description='Use EKF fusion?'),
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to map file'),
        
        # Nhóm Navigation
        DeclareLaunchArgument('start_nav2', default_value='true', description='Start Nav2 stack?'),
        
        # Config Files
        DeclareLaunchArgument('robot_params', default_value=default_robot_config, description='Robot param file'),
        DeclareLaunchArgument('ekf_params', default_value=default_ekf_config, description='EKF param file'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_config, description='Nav2 param file'),
    ]

    # Lấy giá trị từ Argument ra biến
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_imu = LaunchConfiguration('start_imu')
    use_ekf = LaunchConfiguration('use_ekf')
    map_yaml = LaunchConfiguration('map')
    start_nav2 = LaunchConfiguration('start_nav2')
    
    robot_config = LaunchConfiguration('robot_params')
    ekf_config = LaunchConfiguration('ekf_params')
    nav2_params = LaunchConfiguration('nav2_params')

    # =========================================================
    # 3. ĐỊNH NGHĨA CÁC NODE (DỰA TRÊN CODE CỦA CHÚNG TA)
    # =========================================================
    
    # --- STATIC TF ---
    static_tf_group = GroupAction([
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_front',
             arguments=['0.42', '0.29', '0.05', '3.14159', '0', '0', 'base_link', 'lidar_front']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_rear',
             arguments=['-0.42', '-0.29', '0.05', '0', '0', '0', 'base_link', 'lidar_rear']),
    ])

    # --- LIDAR NODES ---
    lidar_group = GroupAction(condition=IfCondition(start_lidar), actions=[
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_front',
             parameters=[robot_config], remappings=[('scan', 'scan_front')]),
        TimerAction(period=2.0, actions=[
            Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_rear',
                 parameters=[robot_config], remappings=[('scan', 'scan_rear')])
        ]),
        # Node gộp Lidar xịn (ScanMerger360)
        Node(package='mecanum_base', executable='scan_merger_360', name='scan_merger_360',
             parameters=[robot_config])
    ])

    # --- DRIVER FUZZY (CORE) ---
    driver_group = GroupAction(actions=[
        # Chạy Driver Fuzzy (Mode: No EKF -> Tự Pub TF)
        Node(package='mecanum_base', executable='robot_driver_fuzzy', name='robot_driver_fuzzy',
             output='screen', respawn=True,
             parameters=[robot_config, {'robot.odom.publish_tf': True}],
             condition=UnlessCondition(use_ekf)),
             
        # Chạy Driver Fuzzy (Mode: EKF -> Tắt TF)
        Node(package='mecanum_base', executable='robot_driver_fuzzy', name='robot_driver_fuzzy',
             output='screen', respawn=True,
             parameters=[robot_config, {'robot.odom.publish_tf': False}],
             remappings=[('/odom', '/wheel/odom')],
             condition=IfCondition(use_ekf)),
             
        # Wheel Visualization (Để quay bánh xe trên RViz)
        Node(package='mecanum_base', executable='wheel_rotation_tf', name='wheel_rot',
             parameters=[robot_config])
    ])

    # --- IMU ---
    imu_node = Node(
        condition=IfCondition(start_imu),
        package='imu_hwt911', executable='hwt911_node', name='imu_node',
        output='screen', parameters=[robot_config], remappings=[('/imu_data', '/imu/data')]
    )

    # --- EKF LOCALIZATION ---
    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_params], remappings=[('/odometry/filtered', '/odom')]
    )

    # =========================================================
    # 4. NAV2 STACK (GỌI TỪ THƯ VIỆN CHUẨN)
    # =========================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        condition=IfCondition(start_nav2),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'True', # Tối ưu RAM
        }.items()
    )

    # =========================================================
    # 5. KHỞI ĐỘNG TUẦN TỰ (DELAY GROUP)
    # =========================================================
    
    # Hardware lên trước
    hardware_sequence = GroupAction([
        static_tf_group,
        lidar_group,
        TimerAction(period=5.0, actions=[driver_group, imu_node])
    ])

    # Thuật toán lên sau 8s
    algo_sequence = TimerAction(period=8.0, actions=[
        ekf_node,
        nav2_launch # Chạy Nav2 luôn trong file này cho tiện
    ])

    # Trả về danh sách
    return LaunchDescription(args + [hardware_sequence, algo_sequence])