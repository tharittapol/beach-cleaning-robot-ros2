from setuptools import find_packages, setup

package_name = 'beach_robot_gnss'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/um982_fix_nema.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'um982_heading = beach_robot_gnss.um982_heading_node:main',
            'gnss_heading_to_imu = beach_robot_gnss.gnss_heading_to_imu_node:main',
            'um982_bridge_node = beach_robot_gnss.bridge_node:main',
            'gps_jitter_node = beach_robot_gnss.gps_jitter_node:main',
        ],
    },
)
