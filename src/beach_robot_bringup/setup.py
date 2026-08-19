from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'beach_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'deploy/udev'), glob('deploy/udev/*')),
        (os.path.join('share', package_name, 'deploy/systemd'), glob('deploy/systemd/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='Bringup package for Beach Robot hardware deployment',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'preflight_check = beach_robot_bringup.tools.preflight_check:main',
            'wheel_response_test = beach_robot_bringup.tools.wheel_response_test:main',
            'wheel_response_analyze = beach_robot_bringup.tools.analyze_wheel_response:main',
            'spin_tune_analyze = beach_robot_bringup.tools.analyze_spin_tune:main',
            'localization_pose_report = beach_robot_bringup.tools.localization_pose_report:main',
            'drive_straight_odom = beach_robot_bringup.tools.drive_straight_odom:main',
            'turn_radius_test = beach_robot_bringup.tools.turn_radius_test:main',
            'turn_radius_analyze = beach_robot_bringup.tools.analyze_turn_radius:main',
            'coverage_bag_report = beach_robot_bringup.tools.coverage_bag_report:main',
        ],
    },
)
