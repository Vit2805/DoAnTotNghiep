from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tung',
    maintainer_email='ngmanhtungg28@gmail.com',
    description='Mecanum robot base controller package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Cấu trúc: 'tên_lệnh_ros = tên_package.tên_file_code:main'
            
            # 1. Driver điều khiển STM32
            'robot_driver = mecanum_base.robot_driver:main',
            
            # 2. Node gộp 2 Lidar
            'dual_lidar_merger = mecanum_base.dual_lidar_merger:main',
            
            # 3. Node hiển thị bánh xe (Visualizer)
            'wheel_rotation_tf = mecanum_base.wheel_rotation_tf:main',
            'wheel_tf_node = mecanum_base.wheel_tf_node:main',
            
            # 4. Teleop (Điều khiển bàn phím - nếu bạn có dùng)
            'mecanum_teleop = mecanum_base.mecanum_teleop:main',
        ],
    },
)