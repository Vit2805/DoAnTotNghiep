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
            'robot_driver_fuzzy = mecanum_base.robot_driver_fuzzy:main',
            
            # 2. Node gộp 2 Lidar
            'scan_merger_360 = mecanum_base.scan_merger_360:main',
            
            # 3. Node hiển thị bánh xe (Visualizer)
            'wheel_rotation_tf = mecanum_base.wheel_rotation_tf:main',
            
            # 4. Teleop (Điều khiển bàn phím - nếu bạn có dùng)
            'mecanum_teleop = mecanum_base.mecanum_teleop:main',
        ],
    },
)