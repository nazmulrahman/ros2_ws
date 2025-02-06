from setuptools import find_packages, setup

package_name = 'pc_serial_communication'

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
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = pc_serial_communication.serial_node:main',
            'cmd_vel_value = pc_serial_communication.cmd_vel_value:main',
            'arduino_to_ros = pc_serial_communication.arduino_to_ros:main',
            'imu_data = pc_serial_communication.imu_data:main',
        ],
    },
)
