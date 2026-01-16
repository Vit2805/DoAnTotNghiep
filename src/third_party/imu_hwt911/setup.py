from setuptools import find_packages, setup

package_name = 'imu_hwt911'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tung',
    maintainer_email='tung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
	'console_scripts': [
            'hwt911_node = imu_hwt911.hwt911_node:main', # <-- Dòng quan trọng
        ],
    },
)
