from setuptools import find_packages, setup

package_name = 'motor_serial_bridge'

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
    maintainer='arnab',
    maintainer_email='arnab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_bridge_node = motor_serial_bridge.motor_bridge_node:main',
            'cmd_to_serial = motor_serial_bridge.cmd_to_serial:main',
            'cmd_to_serial_and_odom = motor_serial_bridge.cmd_to_serial_and_odom:main',
            'all_direction = motor_serial_bridge.all_direction:main',
        ],
    },
)
