import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('mecanum_bringup')
    
    # --- ARGS (Giữ nguyên) ---
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='true')
    start_lidar_arg = DeclareLaunchArgument('start_lidar', default_value='true')
    use_ekf = LaunchConfiguration('use_ekf')
    start_lidar = LaunchConfiguration('start_lidar')
    
    # --- CONFIG FILES (Giữ nguyên) ---
    robot_config = os.path.join(pkg_bringup, 'config', 'base', 'robot_params.yaml')
    ekf_config = os.path.join(pkg_bringup, 'config', 'base', 'ekf_params.yaml')

    # --- STATIC TF (Giữ nguyên) ---
    static_tf_nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_front',
             arguments=['0.42', '0.29', '0.05', '3.14159', '0', '0', 'base_link', 'lidar_front']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_rear',
             arguments=['-0.42', '-0.29', '0.05', '0', '0', '0', 'base_link', 'lidar_rear']),
    ]

    # --- LIDAR NODES (Giữ nguyên) ---
    lidar_front_node = Node(
        package='sllidar_ros2', executable='sllidar_node',
        name='sllidar_front', output='screen', respawn=True,
        parameters=[robot_config],
        remappings=[('scan', 'scan_front')],
        condition=IfCondition(start_lidar)
    )

    lidar_rear_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='sllidar_ros2', executable='sllidar_node',
                name='sllidar_rear', output='screen', respawn=True,
                parameters=[robot_config],
                remappings=[('scan', 'scan_rear')],
                condition=IfCondition(start_lidar)
            )
        ]
    )

    # --- DRIVER & IMU (PHẦN THAY ĐỔI DUY NHẤT) ---
    
    # [THAY ĐỔI] Dùng 'robot_driver_fuzzy' thay vì 'robot_driver'
    driver_fuzzy_no_ekf = Node(
        package='mecanum_base', 
        executable='robot_driver_fuzzy',  # <--- Dùng file thực thi mới
        name='robot_driver_fuzzy',        # <--- Đặt tên mới cho dễ phân biệt
        output='screen', respawn=True,
        parameters=[robot_config, {'robot.odom.publish_tf': True}], 
        condition=UnlessCondition(use_ekf)
    )

    # [THAY ĐỔI] Dùng 'robot_driver_fuzzy' thay vì 'robot_driver'
    driver_fuzzy_with_ekf = Node(
        package='mecanum_base', 
        executable='robot_driver_fuzzy',  # <--- Dùng file thực thi mới
        name='robot_driver_fuzzy',        # <--- Đặt tên mới
        output='screen', respawn=True,
        parameters=[robot_config, {'robot.odom.publish_tf': False}],
        remappings=[('/odom', '/wheel/odom')],       
        condition=IfCondition(use_ekf)
    )

    imu_node = Node(
        package='imu_hwt911', executable='hwt911_node',
        name='imu_node', output='screen',
        parameters=[robot_config],
        remappings=[('/imu_data', '/imu/data')]
    )

    hardware_group = TimerAction(
        period=5.0,
        actions=[driver_fuzzy_no_ekf, driver_fuzzy_with_ekf, imu_node]
    )

    # --- ALGORITHMS (Giữ nguyên) ---
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')],
        condition=IfCondition(use_ekf)
    )

    merger_node = Node(
        package='mecanum_base', 
        executable='scan_merger_360', 
        name='scan_merger_360', 
        output='screen',
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

    return LaunchDescription([
        use_ekf_arg,
        start_lidar_arg,
        *static_tf_nodes,
        lidar_front_node,
        lidar_rear_node,
        hardware_group,
        algo_group
    ])