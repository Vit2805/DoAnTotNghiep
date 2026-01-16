from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Copy Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Copy Maps (Bản đồ)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        
        # 3. Copy Configs (Cấu hình)
        # Copy các file nằm ngay ngoài thư mục config (như robot_params.yaml nếu bạn để ở ngoài)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # [QUAN TRỌNG] Copy cả các thư mục con trong config (như hình bạn gửi)
        # Nếu không có dòng này, các file trong 'base', 'nav', 'slam' sẽ bị bỏ qua!
        (os.path.join('share', package_name, 'config/base'), glob('config/base/*.yaml')),
        (os.path.join('share', package_name, 'config/nav'), glob('config/nav/*.yaml')),
        (os.path.join('share', package_name, 'config/slam'), glob('config/slam/*.yaml')),

        # 4. Copy RViz (Nếu có)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tung',
    maintainer_email='ngmanhtungg28@gmail.com',
    description='Mecanum robot bringup package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)