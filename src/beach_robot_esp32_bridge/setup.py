from setuptools import setup

package_name = 'beach_robot_esp32_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/esp32_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Serial bridge between ROS2 and ESP32 for wheel commands and sensors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32_bridge = beach_robot_esp32_bridge.esp32_bridge_node:main',
        ],
    },
)
