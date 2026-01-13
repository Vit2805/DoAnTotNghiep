import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. Khai báo Marker (Hết warning 1)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        # 2. Khai báo package.xml (Hết warning 2)
        ('share/' + package_name, ['package.xml']),
        
        # 3. Các file Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # 4. Các file Config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tung',
    maintainer_email='tung@todo.todo',
    description='Robot Configuration Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver = my_robot_config.robot_driver:main',
            'wheel_tf_node = my_robot_config.wheel_tf_node:main',
            'wheel_rotation_tf = my_robot_config.wheel_rotation_tf:main',
            'mecanum_teleop = my_robot_config.mecanum_teleop:main',
        ],
    },
)
